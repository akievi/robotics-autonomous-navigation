"""
Erzeugt synthetische 360-Grad-Scans aus dem bestehenden Wandmodell (map_config.yaml),
um die Place-Recognition-Logik offline zu validieren -- ohne Hardware.

Nutzt den bestehenden Raycast aus mcl_nav/mcl_nav/mcl.py rein lesend (kein Import
von rclpy noetig, mcl.py haengt nur von numpy ab -- daher ausserhalb einer
ROS2-Laufzeitumgebung importierbar). mcl.py selbst wird NICHT veraendert.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mcl_nav'))
from mcl import MCL  # noqa: E402  (Pfad muss vor dem Import gesetzt werden)


def make_raycaster(walls, n_bins):
    """Minimale MCL-Instanz nur zum Raycasten (1 Partikel, kein Filter-Betrieb).
    ray_angles wird explizit auf [0, 2pi) mit n_bins Schritten gesetzt, sodass
    Bin 0 = Referenzrichtung theta bei jedem Raycast-Aufruf ist."""
    cfg = dict(n_particles=1, noise_trans=0.0, noise_lat=0.0, noise_rot=0.0,
               bias_lat=0.0, lidar_sigma=0.01, likelihood_floor=0.01, n_rays=n_bins)
    raycaster = MCL(cfg, walls)
    raycaster.ray_angles = np.linspace(0.0, 2 * np.pi, n_bins, endpoint=False)
    return raycaster


def synthetic_scan(raycaster, x, y, theta, noise_sigma=0.0, rng=None):
    """Volle 360-Grad-Tiefenmessung an Pose (x,y,theta).
    Bin i entspricht der Weltrichtung theta + i*360/n_bins Grad.
    noise_sigma: Standardabweichung des simulierten Messrauschens in Metern
    (0.0 fuer die rauschfreie 'Wahrheit', z.B. beim Bau der Trainingssignaturen)."""
    ranges = raycaster._raycast_single_particle(x, y, theta)
    if noise_sigma > 0.0:
        rng = rng if rng is not None else np.random.default_rng()
        finite = np.isfinite(ranges)
        ranges = ranges.copy()
        ranges[finite] = ranges[finite] + rng.normal(0.0, noise_sigma, size=np.sum(finite))
        ranges[finite] = np.clip(ranges[finite], 0.02, None)
    return ranges
