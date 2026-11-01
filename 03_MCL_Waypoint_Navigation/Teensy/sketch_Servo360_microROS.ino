#include <Servo.h>
//---------------------microROS---------------------------------------
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>

int baseRight = 1440;
int baseLeft  = 1585;
int steering = 1500;
int correctionLeft, correctionRight;
bool racing_active = false;

// Kontinuierlicher Fahrmodus
bool driving_active = false;
double drive_target_m = 0.0;
int    drive_correction = 0;  // -100..+100, positiv = nach rechts
int    base_right = 1445;
int    base_left  = 1583;

constexpr int SPOT_TURN_THRESHOLD         = 90;
constexpr int SHARP_CORNER_STEERING_VALUE = 45;
constexpr int OBSTACLE_STOP_VALUE         = 500;
constexpr int OVERTAKE_RIGHT_VALUE        = 200;
constexpr int OVERTAKE_LEFT_VALUE         = -200;

// Odometry constants
const double radDurchmesser  = 0.068;
const double radUmfang       = radDurchmesser * 3.14159;
const double drehDurchmesser = 0.103 * 2;
const double drehUmfang      = drehDurchmesser * 3.14159;

unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight  = 0, lowLeft  = 0;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;
unsigned long cycleTimeRight, cycleTimeLeft;
double thetaRight = 0.0, thetaLeft = 0.0;
double thetaRightPrev = 0.0, thetaLeftPrev = 0.0;
double deltaThetaRight = 0, deltaThetaLeft = 0;
double totalThetaRight = 0, totalThetaLeft = 0;

// IMU yaw from Pi via yaw_feed topic (degrees)
volatile float currentYaw = 0.0;
volatile float yawStart   = 0.0;
volatile bool  yawReady   = false;

// PID constants (from TeensySketchSwitchCase)
const double Kp = 7.0;
const double Ki = 5.0;

void driveRaceTrack(int steering);
void driveSetDistance(double targetDistance_m, int dir);
void turnSetAngle(double targetAngle);
void calculateThetaRight();
void calculateThetaLeft();
void doOvertake(int dir);

// ── microROS entities ────────────────────────────────────────────────
rcl_subscription_t subscriber;         // Int32:   micro_ros_fabinal_subscriber
rcl_subscription_t cmd_subscriber;     // String:  teensy_cmd
rcl_subscription_t yaw_subscriber;     // Float32: yaw_feed
rcl_publisher_t    done_publisher;     // Int32:   overtake_done
rcl_publisher_t    feedback_publisher; // String:  odom_feedback

std_msgs__msg__Int32   msg;
std_msgs__msg__Int32   done_msg;
std_msgs__msg__String  cmd_msg;
std_msgs__msg__String  feedback_msg;
std_msgs__msg__Float32 yaw_msg;

char cmd_buf[64];
char feedback_buf[64];

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)
//-------------------------End of microROS------------------------------

Servo servoRight;
Servo servoLeft;

constexpr int pinFeedbackLeft  = 7;
constexpr int pinFeedbackRight = 15;
constexpr int pinControlLeft   = 8;
constexpr int pinControlRight  = 16;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void motors_move(int right, int left) {
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);
  servoRight.write(right);
  servoLeft.write(left);
  delay(500);
  servoRight.detach();
  servoLeft.detach();
}

void motors_stop() {
  motors_move(94, 94);
}

// ── Callbacks ────────────────────────────────────────────────────────

void subscription_callback(const void * msgin) {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;
  steering = m->data;
  racing_active = true;
}

void yaw_callback(const void * msgin) {
  const std_msgs__msg__Float32 * m = (const std_msgs__msg__Float32 *)msgin;
  currentYaw = m->data;
  yawReady = true;
}

void cmd_callback(const void * msgin) {
  const std_msgs__msg__String * m = (const std_msgs__msg__String *)msgin;
  const char * data = m->data.data;

  if (data[0] == 'S') {
    driving_active = false;
    servoRight.attach(pinControlRight);
    servoLeft.attach(pinControlLeft);
    servoRight.writeMicroseconds(1500);
    servoLeft.writeMicroseconds(1500);
    servoRight.detach();
    servoLeft.detach();
    double actual = (fabs(totalThetaRight) + fabs(totalThetaLeft)) / 2.0 / 360.0 * radUmfang;
    snprintf(feedback_buf, sizeof(feedback_buf), "D:%.4f", actual);

  } else if (data[0] == 'R' && data[1] == ':') {
    // Starte kontinuierliches Fahren: "R:<distanz_m>"
    drive_target_m = atof(data + 2);
    drive_correction = 0;
    calculateThetaRight(); calculateThetaLeft();
    totalThetaRight = 0; totalThetaLeft = 0;
    servoRight.attach(pinControlRight);
    servoLeft.attach(pinControlLeft);
    driving_active = true;
    snprintf(feedback_buf, sizeof(feedback_buf), "R:ok");

  } else if (data[0] == 'C' && data[1] == ':') {
    // Korrekturwert aktualisieren: "C:<-100..100>"
    drive_correction = (int)atof(data + 2);
    snprintf(feedback_buf, sizeof(feedback_buf), "C:ok");
    // kein publish noetig, kein Feedback zurueck
    return;

  } else if (data[0] == 'D' && data[1] == ':') {
    // Format: "D:<dist>:<yaw_deg>" — Ziel-Yaw vom Pi mitgeschickt
    double dist = atof(data + 2);
    const char * colon2 = strchr(data + 2, ':');
    if (colon2 != NULL) {
      yawStart = atof(colon2 + 1);  // Ziel-Yaw vom Pi
    } else {
      yawStart = currentYaw;  // Fallback: aktueller Yaw
    }
    calculateThetaRight(); calculateThetaLeft();
    totalThetaRight = 0; totalThetaLeft = 0;
    driveSetDistance(dist, 1);
    double actual = (fabs(totalThetaRight) + fabs(totalThetaLeft)) / 2.0 / 360.0 * radUmfang;
    snprintf(feedback_buf, sizeof(feedback_buf), "D:%.4f", actual);

  } else if (data[0] == 'T' && data[1] == ':') {
    double angle = atof(data + 2);
    calculateThetaRight(); calculateThetaLeft();
    totalThetaRight = 0; totalThetaLeft = 0;
    turnSetAngle(angle);
    double arcLeft  = fabs(totalThetaLeft)  / 360.0 * radUmfang;
    double arcRight = fabs(totalThetaRight) / 360.0 * radUmfang;
    double actual_deg = (arcLeft + arcRight) / drehUmfang * 360.0;
    if (angle < 0) actual_deg = -actual_deg;
    snprintf(feedback_buf, sizeof(feedback_buf), "T:%.2f", actual_deg);

  } else {
    snprintf(feedback_buf, sizeof(feedback_buf), "ERR:unknown_cmd");
  }

  feedback_msg.data.data     = feedback_buf;
  feedback_msg.data.size     = strlen(feedback_buf);
  feedback_msg.data.capacity = sizeof(feedback_buf);
  rcl_publish(&feedback_publisher, &feedback_msg, NULL);
}

// ── Odometry helpers ─────────────────────────────────────────────────

void calculateThetaRight() {
  do {
    highRight      = pulseIn(pinFeedbackRight, HIGH, 3000UL);
    lowRight       = pulseIn(pinFeedbackRight, LOW,  3000UL);
    cycleTimeRight = highRight + lowRight;
  } while (cycleTimeRight < 1000 || cycleTimeRight > 1200);

  dutyCycleRight = ((double)highRight / (double)cycleTimeRight) * 100.0;
  thetaRightPrev = thetaRight;
  thetaRight = 359.0 - ((dutyCycleRight - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
  if (thetaRight < 0.0)   thetaRight = 0.0;
  if (thetaRight > 359.0) thetaRight = 359.0;

  double diff = thetaRight - thetaRightPrev;
  if (diff < -180.0) diff += 360.0;
  if (diff >  180.0) diff -= 360.0;
  totalThetaRight += diff;
  deltaThetaRight  = diff;
}

void calculateThetaLeft() {
  do {
    highLeft      = pulseIn(pinFeedbackLeft, HIGH, 3000UL);
    lowLeft       = pulseIn(pinFeedbackLeft, LOW,  3000UL);
    cycleTimeLeft = highLeft + lowLeft;
  } while (cycleTimeLeft < 1000 || cycleTimeLeft > 1200);

  dutyCycleLeft = ((double)highLeft / (double)cycleTimeLeft) * 100.0;
  thetaLeftPrev = thetaLeft;
  thetaLeft = 359.0 - ((dutyCycleLeft - 2.9) * 360.0) / (97.1 - 2.9 + 1.0);
  if (thetaLeft < 0.0)   thetaLeft = 0.0;
  if (thetaLeft > 359.0) thetaLeft = 359.0;

  double diff = thetaLeft - thetaLeftPrev;
  if (diff < -180.0) diff += 360.0;
  if (diff >  180.0) diff -= 360.0;
  totalThetaLeft += diff;
  deltaThetaLeft  = diff;
}

// ── Gerade fahren (reine Odometrie) ─────────────────────────────────
void driveSetDistance(double targetDistance_m, int dir) {
  int bRight, bLeft;
  if (dir == 1) {
    bRight = 1443;
    bLeft  = 1580;
  } else {
    bRight = 1565;
    bLeft  = 1444;
  }

  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0;
  totalThetaLeft  = 0;
  double distanceTraveled = 0.0;

  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  while (distanceTraveled < targetDistance_m) {
    calculateThetaRight();
    calculateThetaLeft();
    distanceTraveled = (fabs(totalThetaRight) + fabs(totalThetaLeft)) / 2.0 / 360.0 * radUmfang;
    // Geradeauskorrektur: rechts > links → rechts abbremsen (auto_corr positiv)
    double delta = fabs(totalThetaRight) - fabs(totalThetaLeft);
    int auto_corr = constrain((int)(delta * 3.0), -40, 40);
    servoRight.writeMicroseconds(constrain(bRight + auto_corr, 1400, 1490));
    servoLeft.writeMicroseconds(constrain(bLeft  + auto_corr, 1560, 1630));
  }

  // Sanfter Stopp: schrittweise auf 1500 reduzieren
  int steps = 10;
  for (int i = 1; i <= steps; i++) {
    int r = bRight + (1500 - bRight) * i / steps;
    int l = bLeft  + (1500 - bLeft)  * i / steps;
    servoRight.writeMicroseconds(r);
    servoLeft.writeMicroseconds(l);
    delay(40);
  }
  servoRight.writeMicroseconds(1500);
  servoLeft.writeMicroseconds(1500);
  servoRight.detach();
  servoLeft.detach();
}

// ── Spot-Turn ────────────────────────────────────────────────────────
void turnSetAngle(double targetAngle) {
  double targetDistance_m = (fabs(targetAngle) / 360.0 * drehUmfang) * 1.03;
  double distTraveledLeft  = 0.0;
  double distTraveledRight = 0.0;

  int bRight, bLeft;
  if (targetAngle > 0) {
    // Linksdrehung: rechts vorwaerts, links rueckwaerts
    bRight = 1440;  // unter 1500 = vorwaerts fuer rechts
    bLeft  = 1440;  // unter 1500 = rueckwaerts fuer links
  } else {
    // Rechtsdrehung: rechts rueckwaerts, links vorwaerts
    bRight = 1560;  // ueber 1500 = rueckwaerts fuer rechts
    bLeft  = 1560;  // ueber 1500 = vorwaerts fuer links
  }

  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0;
  totalThetaLeft  = 0;

  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  while (distTraveledLeft + distTraveledRight < targetDistance_m) {
    calculateThetaRight();
    calculateThetaLeft();

    distTraveledLeft  = (fabs(totalThetaLeft)  / 360.0) * radUmfang;
    distTraveledRight = (fabs(totalThetaRight) / 360.0) * radUmfang;

    servoLeft.writeMicroseconds(distTraveledLeft   < targetDistance_m / 2 ? bLeft  : 1500);
    servoRight.writeMicroseconds(distTraveledRight < targetDistance_m / 2 ? bRight : 1500);
  }

  servoRight.writeMicroseconds(1500);
  servoLeft.writeMicroseconds(1500);
  servoRight.detach();
  servoLeft.detach();
}

// ── Overtaking ───────────────────────────────────────────────────────
void doOvertake(int dir) {
  double turnAngle     = 60.0 * dir;
  double driveDistance = 0.2;

  delay(400);
  turnSetAngle(turnAngle);
  delay(200);
  driveSetDistance(driveDistance, 1);
  delay(200);
  turnSetAngle(-turnAngle);
  delay(200);
  driveSetDistance(0.17, 1);
  delay(200);
  turnSetAngle(-turnAngle);
  delay(200);
  driveSetDistance(driveDistance / 1.7, 1);
  delay(200);
  turnSetAngle(turnAngle);

  done_msg.data = 1;
  rcl_publish(&done_publisher, &done_msg, NULL);
  delay(100);
  done_msg.data = 0;
  rcl_publish(&done_publisher, &done_msg, NULL);
}

// ── Normal driving ───────────────────────────────────────────────────
void driveRaceTrack(int steering) {
  if (abs(steering) >= OBSTACLE_STOP_VALUE) {
    servoRight.attach(pinControlRight);
    servoLeft.attach(pinControlLeft);
    servoRight.writeMicroseconds(1500);
    servoLeft.writeMicroseconds(1500);
    return;
  }

  if (steering == OVERTAKE_RIGHT_VALUE) {
    doOvertake(1);
    steering = 0;
    return;
  }
  if (steering == OVERTAKE_LEFT_VALUE) {
    doOvertake(-1);
    steering = 0;
    return;
  }

  if (abs(steering) >= SPOT_TURN_THRESHOLD) {
    servoRight.attach(pinControlRight);
    servoLeft.attach(pinControlLeft);
    int factor = 2.2;
    if (steering > 0) {
      servoLeft.writeMicroseconds(baseLeft + factor * (steering - 20));
      servoRight.writeMicroseconds(baseRight + 50);
    } else {
      servoRight.writeMicroseconds(baseRight + factor * (steering + 20));
      servoLeft.writeMicroseconds(baseLeft - 50);
    }
    return;
  }

  int factor = 2.2;
  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  if (steering > 5) {
    correctionRight = factor * steering * 0 + 30;
    correctionLeft  = factor * steering - 25;
  } else if (steering < -5) {
    correctionRight = factor * steering + 5;
    correctionLeft  = factor * steering * 0 - 37;
  } else {
    correctionRight = -12;
    correctionLeft  = 15;
  }
  servoRight.writeMicroseconds(baseRight + correctionRight);
  servoLeft.writeMicroseconds(baseLeft + correctionLeft);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 4));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  rcl_init_options_fini(&init_options);

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_fabinal_subscriber"));

  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "teensy_cmd"));

  RCCHECK(rclc_subscription_init_default(
    &yaw_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "yaw_feed"));

  RCCHECK(rclc_publisher_init_default(
    &done_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "overtake_done"));

  RCCHECK(rclc_publisher_init_default(
    &feedback_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "odom_feedback"));

  cmd_msg.data.data     = cmd_buf;
  cmd_msg.data.capacity = sizeof(cmd_buf);
  cmd_msg.data.size     = 0;

  // 3 subscriptions now
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                          &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_msg,
                                          &cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &yaw_subscriber, &yaw_msg,
                                          &yaw_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  rcl_subscription_fini(&cmd_subscriber, &node);
  rcl_subscription_fini(&yaw_subscriber, &node);
  rcl_publisher_fini(&done_publisher, &node);
  rcl_publisher_fini(&feedback_publisher, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  pinMode(pinFeedbackRight, INPUT);
  pinMode(pinFeedbackLeft, INPUT);
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      motors_stop();
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, LOW);

    if (driving_active) {
      // corr > 0: nach rechts -> rechts langsamer (+corr naher 1500), links schneller (+corr weg von 1500)
      // corr < 0: nach links  -> rechts schneller, links langsamer
      int corr = constrain(drive_correction, -100, 100);
      // Automatische Geradeauskorrektur via Odometrie-Delta
      double delta = fabs(totalThetaRight) - fabs(totalThetaLeft);
      int auto_corr = constrain((int)(delta * 3.0), -40, 40);
      int total_corr = constrain(corr + auto_corr, -100, 100);
      int pwm_r = constrain(base_right + total_corr, 1400, 1490);
      int pwm_l = constrain(base_left  + total_corr, 1560, 1630);
      servoRight.writeMicroseconds(pwm_r);
      servoLeft.writeMicroseconds(pwm_l);

      // Odometrie nur alle 50ms pruefen
      EXECUTE_EVERY_N_MS(50, {
        calculateThetaRight();
        calculateThetaLeft();
        double traveled = (fabs(totalThetaRight) + fabs(totalThetaLeft)) / 2.0 / 360.0 * radUmfang;
        if (traveled >= drive_target_m) {
          driving_active = false;
          servoRight.writeMicroseconds(1500);
          servoLeft.writeMicroseconds(1500);
          servoRight.detach();
          servoLeft.detach();
          snprintf(feedback_buf, sizeof(feedback_buf), "D:%.4f", traveled);
          feedback_msg.data.data     = feedback_buf;
          feedback_msg.data.size     = strlen(feedback_buf);
          feedback_msg.data.capacity = sizeof(feedback_buf);
          rcl_publish(&feedback_publisher, &feedback_msg, NULL);
        }
      });
    } else if (racing_active) {
      driveRaceTrack(steering);
    }
  } else {
    digitalWrite(LED_PIN, HIGH);
    racing_active = false;
    driving_active = false;
  }
}
