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

int baseRight = 1440;
int baseLeft  = 1585;
int steering;
int correctionLeft, correctionRight;

constexpr int SPOT_TURN_THRESHOLD    = 90;
constexpr int SHARP_CORNER_STEERING_VALUE = 45;
constexpr int OBSTACLE_STOP_VALUE    = 500;
constexpr int OVERTAKE_RIGHT_VALUE   = 200;
constexpr int OVERTAKE_LEFT_VALUE    = -200;

// Odometry constants (from TeensySketchSwitchCase)
const double radDurchmesser = 0.068;
const double radUmfang      = radDurchmesser * 3.14159;
const double drehDurchmesser = 0.103 * 2;
const double drehUmfang     = drehDurchmesser * 3.14159;

unsigned long highRight = 0, highLeft = 0;
unsigned long lowRight  = 0, lowLeft  = 0;
double dutyCycleRight = 0.0, dutyCycleLeft = 0.0;
unsigned long cycleTimeRight, cycleTimeLeft;
double thetaRight = 0.0, thetaLeft = 0.0;
double thetaRightPrev = 0.0, thetaLeftPrev = 0.0;
double deltaThetaRight = 0, deltaThetaLeft = 0;
double totalThetaRight = 0, totalThetaLeft = 0;

void driveRaceTrack(int steering);
void driveSetDistance(double targetDistance_m, int dir);
void turnSetAngle(double targetAngle);
void calculateThetaRight();
void calculateThetaLeft();
void doOvertake(int dir);

rcl_subscription_t subscriber;
rcl_publisher_t    done_publisher;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 done_msg;
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

void subscription_callback(const void * msgin) {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  const std_msgs__msg__Int32 * m = (const std_msgs__msg__Int32 *)msgin;
  steering = m->data;
}

// ── Odometry helpers ────────────────────────────────────────────────

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

// Drive straight for targetDistance_m. dir=1 forward, dir=-1 backward.
void driveSetDistance(double targetDistance_m, int dir) {
  int bRight, bLeft;
  if (dir == 1) {
    bRight = 1440;
    bLeft  = 1568;
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
    distanceTraveled = (abs(totalThetaRight) + abs(totalThetaLeft)) / 2.0 / 360.0 * radUmfang;
    servoRight.writeMicroseconds(bRight);
    servoLeft.writeMicroseconds(bLeft);
  }

  servoRight.writeMicroseconds(1500);
  servoLeft.writeMicroseconds(1500);
  servoRight.detach();
  servoLeft.detach();
}

// Spot turn by targetAngle degrees. Positive = right, negative = left.
void turnSetAngle(double targetAngle) {
  double targetDistance_m = (abs(targetAngle) / 360.0 * drehUmfang) * 1.03;
  double distTraveledLeft  = 0.0;
  double distTraveledRight = 0.0;

  int bRight, bLeft;
  if (targetAngle > 0) {
    bRight = 1560;
    bLeft  = 1565;
  } else {
    bRight = 1440;
    bLeft  = 1445;
  }

  calculateThetaRight();
  calculateThetaLeft();
  totalThetaRight = 0;
  totalThetaLeft  = 0;

  servoRight.attach(pinControlRight);
  servoLeft.attach(pinControlLeft);

  // Stop condition matches original: sum of both wheels < targetDistance_m (not *2)
  while (distTraveledLeft + distTraveledRight < targetDistance_m) {
    calculateThetaRight();
    calculateThetaLeft();

    distTraveledLeft  = (abs(totalThetaLeft)  / 360.0) * radUmfang;
    distTraveledRight = (abs(totalThetaRight) / 360.0) * radUmfang;

    servoLeft.writeMicroseconds(distTraveledLeft   < targetDistance_m / 2 ? bLeft  : 1500);
    servoRight.writeMicroseconds(distTraveledRight < targetDistance_m / 2 ? bRight : 1500);
  }

  servoRight.writeMicroseconds(1500);
  servoLeft.writeMicroseconds(1500);
  servoRight.detach();
  servoLeft.detach();
}

// ── Overtaking maneuver ─────────────────────────────────────────────
// dir: +1 = overtake to the right, -1 = overtake to the left
// Sequence: turn out 60° → drive 0.25m → turn back 60° → drive 0.25m → realign 60°
void doOvertake(int dir) {
  double turnAngle    = 60.0 * dir;
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
  driveSetDistance(driveDistance/1.7, 1);
  delay(200);
  turnSetAngle(turnAngle);     

  // Signal Pi that maneuver is done
  done_msg.data = 1;
  rcl_publish(&done_publisher, &done_msg, NULL);
  delay(100);
  done_msg.data = 0;
  rcl_publish(&done_publisher, &done_msg, NULL);
}

// ── Normal driving ──────────────────────────────────────────────────

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

  RCCHECK(rclc_publisher_init_default(
    &done_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "overtake_done"));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                          &subscription_callback, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  rcl_publisher_fini(&done_publisher, &node);
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
    driveRaceTrack(steering);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}
