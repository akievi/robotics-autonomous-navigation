"""
Stellt eine oder mehrere Signaturen als radiale Linien dar, relativ zum
Bildschirmmittelpunkt (Objective 2.8a der Aufgabenstellung).

Zwei Modi:
- Datei-Modus (Standard): liest gespeicherte .pkl-Signaturen, funktioniert komplett offline.
- `--live`: scannt direkt (RPLidar-Node muss laufen, kein ROS2-Sourcing-Unterschied zu
  learn_location.py) und zeigt das Ergebnis, ohne vorher speichern zu muessen -- deckt
  den Wortlaut der Aufgabenstellung ab ("Programm soll ueberall funktionieren", ohne dass
  vorher ein Ortsname vergeben werden muss).

Nutzung:
    python3 visualize_signature.py signatures/P2.pkl signatures/P4.pkl ...
    python3 visualize_signature.py --live
    python3 visualize_signature.py --live --n-bins 180 --n-scans 5
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from place_signature import LocationSignature, load_signature  # noqa: E402

DEFAULT_N_BINS = 360
DEFAULT_N_SCANS = 10


def plot_signature(sig, ax):
    n = sig.n_bins
    angles = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    ranges = np.where(np.isfinite(sig.angle_ranges), sig.angle_ranges, 0.0)
    ax.plot(angles, ranges, 'b-', linewidth=1)
    ax.set_theta_zero_location('E')  # 0 Grad = jeweilige lokale Fahrtrichtung bei Aufnahme
    ax.set_theta_direction(1)
    ax.set_title(f'{sig.location_id}  ({n} Bins)')


def capture_live_signature(n_bins, n_scans):
    """Nimmt live eine Signatur auf (ueber LocationLearner aus learn_location.py) und
    gibt sie als LocationSignature zurueck, ohne sie zu speichern. Erfordert eine
    laufende ROS2-Umgebung mit /scan-Topic (z.B. lidar_only.launch.py)."""
    import rclpy
    from learn_location import LocationLearner  # noqa: E402  spaeter Import: nur bei --live noetig

    rclpy.init()
    node = LocationLearner(n_bins, n_scans, node_name='visualize_live')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
        ranges = node.averaged_signature()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return LocationSignature(location_id='Live-Scan', angle_ranges=ranges)


def main():
    parser = argparse.ArgumentParser(
        description='Stellt Signaturen als radiale Linien dar (Objective 2.8a).')
    parser.add_argument('paths', nargs='*', help='Pfade zu gespeicherten .pkl-Signaturen')
    parser.add_argument('--live', action='store_true',
                         help='Statt Dateien zu laden: direkt live scannen (RPLidar-Node muss laufen)')
    parser.add_argument('--n-bins', type=int, default=DEFAULT_N_BINS)
    parser.add_argument('--n-scans', type=int, default=DEFAULT_N_SCANS)
    parser.add_argument('--out', default=os.path.join(os.path.dirname(__file__), 'signature_plot.png'))
    args = parser.parse_args()

    if args.live:
        sigs = [capture_live_signature(args.n_bins, args.n_scans)]
    else:
        if not args.paths:
            parser.error('Entweder --live angeben oder mindestens einen Signaturpfad.')
        sigs = [load_signature(p) for p in args.paths]

    fig, axes = plt.subplots(1, len(sigs), subplot_kw=dict(projection='polar'),
                              figsize=(5 * len(sigs), 5))
    if len(sigs) == 1:
        axes = [axes]
    for ax, sig in zip(axes, sigs):
        plot_signature(sig, ax)

    plt.tight_layout()
    plt.savefig(args.out, dpi=120)
    print(f'Plot gespeichert: {args.out}')


if __name__ == '__main__':
    main()