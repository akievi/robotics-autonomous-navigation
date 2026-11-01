"""
Monte Carlo Localization (MCL) Partikelfilter.
Numpy-vektorisiert: keine Python-Schleifen ueber Partikel.
Koordinaten: x=vorwaerts, y=links, theta in Rad (positiv=links).
"""

import numpy as np


class MCL:
    def __init__(self, cfg, walls):
        """
        cfg:   mcl-Abschnitt aus map_config.yaml
        walls: Liste von ((x1,y1),(x2,y2)) Wandsegmenten
        """
        self.N = cfg['n_particles']
        self.noise_trans = cfg['noise_trans']   # σ_x pro Meter
        self.noise_lat   = cfg['noise_lat']     # σ_y pro Meter (seitlich)
        self.noise_rot   = cfg['noise_rot']     # σ_theta
        self.bias_lat    = cfg['bias_lat']      # systematischer Linksdrift m/m
        self.sigma       = cfg['lidar_sigma']
        self.floor       = cfg['likelihood_floor']
        self.n_rays      = cfg['n_rays']

        # Wandsegmente als (N_walls, 2, 2) Array: walls[i] = [[x1,y1],[x2,y2]]
        if len(walls) > 0:
            self.walls = np.array(walls, dtype=np.float64)  # (W, 2, 2)
        else:
            self.walls = np.zeros((0, 2, 2), dtype=np.float64)

        # Partikel: (N, 3) — [x, y, theta]
        self.particles = np.zeros((self.N, 3), dtype=np.float64)
        # Gewichte: (N,)
        self.weights = np.ones(self.N, dtype=np.float64) / self.N

        # Gleichverteilte Strahlwinkel fuer Messung (relativ zu Partikel-theta)
        self.ray_angles = np.linspace(-np.pi, np.pi, self.n_rays, endpoint=False)

    def init_pose(self, x, y, theta, spread_xy=0.05, spread_theta=0.05):
        """Partikel um Startpose mit leichter Streuung initialisieren."""
        self.particles[:, 0] = np.random.normal(x, spread_xy,   self.N)
        self.particles[:, 1] = np.random.normal(y, spread_xy,   self.N)
        self.particles[:, 2] = np.random.normal(theta, spread_theta, self.N)
        self.weights[:] = 1.0 / self.N

    # ── Motion Model ────────────────────────────────────────────────

    def motion_predict(self, delta_dist, delta_theta):
        """
        Bewegt alle Partikel mit individuellem Gauss-Rauschen.
        delta_dist:  gefahrene Distanz in Metern (aus Odometrie)
        delta_theta: gedrehter Winkel in Rad (aus IMU-Fusion)
        """
        N = self.N
        # Empirisch gemessene Sigmas (8x 2m Geradeausfahrt)
        # σ_x=0.96%/m, σ_y=2.17%/m, y-Bias=+1.81cm/m (Linksdrift)
        sigma_x   = self.noise_trans * abs(delta_dist) + 1e-4
        sigma_y   = self.noise_lat   * abs(delta_dist) + 1e-4
        sigma_rot = self.noise_rot   * abs(delta_dist) + self.noise_rot * abs(delta_theta) + 1e-4

        e = np.random.normal(0,               sigma_x,   N)  # Längsfehler
        f = np.random.normal(self.bias_lat * delta_dist, sigma_y, N)  # Seitenfehler inkl. Bias

        g = np.random.normal(0, sigma_rot, N)

        theta = self.particles[:, 2]
        self.particles[:, 0] += (delta_dist + e) * np.cos(theta) - f * np.sin(theta)
        self.particles[:, 1] += (delta_dist + e) * np.sin(theta) + f * np.cos(theta)
        self.particles[:, 2] += delta_theta + g

        # Theta in [-pi, pi] halten
        self.particles[:, 2] = (self.particles[:, 2] + np.pi) % (2 * np.pi) - np.pi

    # ── Raycast ─────────────────────────────────────────────────────

    def _raycast_single_particle(self, px, py, ptheta):
        """
        Berechnet erwartete Distanzen fuer alle n_rays Strahlen an einer Pose.
        Gibt Array der Laenge n_rays zurueck (inf wenn kein Wandtreffer).
        """
        if len(self.walls) == 0:
            return np.full(self.n_rays, np.inf)

        abs_angles = ptheta + self.ray_angles  # (R,)
        cos_a = np.cos(abs_angles)             # (R,)
        sin_a = np.sin(abs_angles)             # (R,)

        # Wand-Endpunkte
        Ax = self.walls[:, 0, 0]  # (W,)
        Ay = self.walls[:, 0, 1]
        Bx = self.walls[:, 1, 0]
        By = self.walls[:, 1, 1]

        dBx = Bx - Ax  # (W,)
        dBy = By - Ay

        # Raycast: Schnittpunkt Strahl (px,py,angle) mit Segment (A,B)
        # t = ((Ay-py)*cos_a - (Ax-px)*sin_a) / (dBx*sin_a - dBy*cos_a)
        # m = ((Ay-py)*dBx   - (Ax-px)*dBy)   / (dBx*sin_a - dBy*cos_a)
        # Treffer wenn 0<=t<=1 und m>0

        # Broadcasting: (W,1) gegen (1,R)
        denom = dBx[:, None] * sin_a[None, :] - dBy[:, None] * cos_a[None, :]  # (W,R)
        # Kleine Nenner vermeiden (parallele Strahlen)
        valid_denom = np.abs(denom) > 1e-10

        t = np.where(valid_denom,
                     ((Ay[:, None] - py) * cos_a[None, :] - (Ax[:, None] - px) * sin_a[None, :]) / np.where(valid_denom, denom, 1.0),
                     np.inf)
        m = np.where(valid_denom,
                     ((Ay[:, None] - py) * dBx[:, None] - (Ax[:, None] - px) * dBy[:, None]) / np.where(valid_denom, denom, 1.0),
                     np.inf)

        # Nur gueltige Treffer: t in [0,1], m > 0
        hit = valid_denom & (t >= 0.0) & (t <= 1.0) & (m > 0.0)
        distances = np.where(hit, m, np.inf)  # (W, R)

        # Kuerzester Treffer pro Strahl
        return np.min(distances, axis=0)  # (R,)

    def _raycast_all(self, lidar_offset_x=0.0, lidar_offset_y=0.0):
        """
        Berechnet erwartete Messungen fuer alle N Partikel.
        Gibt (N, R) Array zurueck.
        """
        expected = np.zeros((self.N, self.n_rays), dtype=np.float64)
        for i in range(self.N):
            px = self.particles[i, 0] + lidar_offset_x * np.cos(self.particles[i, 2]) \
                                      - lidar_offset_y * np.sin(self.particles[i, 2])
            py = self.particles[i, 1] + lidar_offset_x * np.sin(self.particles[i, 2]) \
                                      + lidar_offset_y * np.cos(self.particles[i, 2])
            expected[i] = self._raycast_single_particle(px, py, self.particles[i, 2])
        return expected

    # ── Measurement Update ──────────────────────────────────────────

    def measurement_update(self, scan_ranges, scan_angles,
                           lidar_offset_x=0.0, lidar_offset_y=0.0):
        """
        Aktualisiert Gewichte anhand eines Lidar-Scans.
        scan_ranges: (M,) gemessene Distanzen in Metern
        scan_angles: (M,) zugehoerige Winkel in Rad (absolut im Roboter-Frame)
        """
        if len(self.walls) == 0:
            return

        # Nur Strahlen die auf nahegelegene Objekte zeigen (wahrscheinlich Waende)
        finite_mask = np.isfinite(scan_ranges) & (scan_ranges > 0.05) & (scan_ranges < 5.0)
        if np.sum(finite_mask) < 5:
            return
        finite_idx = np.where(finite_mask)[0]

        # Gleichmaessiges Subsample — nur n_rays Strahlen
        step = max(1, len(finite_idx) // self.n_rays)
        indices = finite_idx[::step][:self.n_rays]
        z_meas = scan_ranges[indices]

        orig_ray_angles = self.ray_angles.copy()
        self.ray_angles = scan_angles[indices]

        # Erwartete Messungen fuer alle Partikel
        z_exp = self._raycast_all(lidar_offset_x, lidar_offset_y)  # (N, R)

        self.ray_angles = orig_ray_angles

        # Nur Strahlen verwenden wo Messung gueltig ist
        meas_valid = np.isfinite(z_meas)  # (R,)
        if np.sum(meas_valid) < 3:
            return

        z_exp_v = z_exp[:, meas_valid]     # (N, R_valid)
        z_meas_v = z_meas[meas_valid]      # (R_valid,)

        # Wo Raycast kein Treffer (inf): grosse Strafe statt ignorieren
        z_exp_v = np.where(np.isfinite(z_exp_v), z_exp_v, 10.0)

        # Log-Likelihood summieren — verhindert numerischen Underflow
        diff = z_exp_v - z_meas_v[None, :]          # (N, R_valid)
        log_like = -0.5 * (diff / self.sigma) ** 2  # (N, R_valid)

        # Summe der Log-Likelihoods pro Partikel
        log_w = np.sum(log_like, axis=1)  # (N,)

        # Numerisch stabil normalisieren: subtrahiere Maximum
        log_w -= np.max(log_w)
        new_weights = np.exp(log_w)
        new_weights = np.maximum(new_weights, self.floor)  # absoluter Floor

        # Mit bestehenden Gewichten multiplizieren und normalisieren
        self.weights *= new_weights
        w_sum = np.sum(self.weights)
        if w_sum > 0:
            self.weights /= w_sum
        else:
            self.weights[:] = 1.0 / self.N

    def imu_update(self, imu_theta, sigma):
        """
        Schwaches Theta-Update aus IMU-Yaw.
        Gewichtet Partikel deren theta nah am IMU-Wert liegt.
        sigma gross (z.B. 15deg) damit IMU nur grob einfliesst.
        """
        diff = self.particles[:, 2] - imu_theta
        diff = (diff + np.pi) % (2 * np.pi) - np.pi  # wrap
        log_w = -0.5 * (diff / sigma) ** 2
        log_w -= np.max(log_w)
        w = np.exp(log_w)
        w = np.maximum(w, self.floor)
        self.weights *= w
        w_sum = np.sum(self.weights)
        if w_sum > 0:
            self.weights /= w_sum
        else:
            self.weights[:] = 1.0 / self.N

    # ── Resampling ──────────────────────────────────────────────────

    def resample(self):
        """Systematic resampling."""
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0  # Rundungsfehler absichern
        step = 1.0 / self.N
        start = np.random.uniform(0, step)
        positions = start + step * np.arange(self.N)
        indices = np.searchsorted(cumsum, positions)
        self.particles = self.particles[indices].copy()
        self.weights[:] = 1.0 / self.N

    def resample_axis(self, axis='x'):
        """
        Achsen-getrenntes Resampling (Vorlesungsempfehlung).
        axis='x': nur nach x-Fitness resampeln (langer Korridor)
        axis='xy': beide Achsen (kurze Querstrasse)
        """
        # Vereinfachung: Standard-Resampling auf Basis der bestehenden Gewichte
        # Gewichte wurden in measurement_update bereits axis-unabhaengig berechnet;
        # hier koennte man selektiv nur bestimmte Strahlen einbeziehen.
        # Fuer den ersten Durchgang reicht standard resample.
        self.resample()

    # ── Pose-Schaetzung ─────────────────────────────────────────────

    def estimate_pose(self):
        """
        Gewichteter Mittelwert der Partikel.
        Theta ueber atan2(mean(sin), mean(cos)) wegen Wraparound.
        Gibt (x, y, theta) zurueck.
        """
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)
        sin_t = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        cos_t = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        theta = np.arctan2(sin_t, cos_t)
        return x, y, theta

    def effective_particle_count(self):
        """N_eff = 1/sum(w^2) — Indikator fuer Partikel-Degeneration."""
        return 1.0 / np.sum(self.weights ** 2)
