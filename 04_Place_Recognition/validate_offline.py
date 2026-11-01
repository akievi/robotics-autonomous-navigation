"""
Offline-Kalibrierung und -Validierung der Place-Recognition-Logik gegen das
bestehende Wandmodell aus config/map_config.yaml -- ohne Hardware, ohne ROS2.

Liest map_config.yaml NUR LESEND. Schreibt ausschliesslich in diesen eigenen
Ordner (place_recognition_dev/signatures_synthetic/, place_recognition_dev/report.txt).

WICHTIG: Schreibt bewusst NICHT nach signatures/ -- das ist der Ordner, in den
learn_location.py echte, mit Hardware aufgenommene Signaturen speichert. Wuerden beide
Skripte denselben Ordner nutzen, wuerde ein erneuter validate_offline.py-Lauf echte
Trainingsdaten stillschweigend mit synthetischen Platzhaltern ueberschreiben.

Vorgehen:
1. Baut fuer jeden Trainingsort (P2, P4, P6, P9) eine rauschfreie 'Trainings'-
   Signatur bei theta=0 (globale x-Achse/Fahrtrichtung -- siehe Aufgabenstellung
   Objective 2.8b: "in jedem Fall so ausgerichtet, dass der Roboter zu Scan-
   Beginn entlang der x-Achse zeigt").
2. Simuliert pro Ort N_TRIALS_PER_LOCATION verrauschte Testscans mit zufaelliger
   Orientierung (Monte-Carlo), vergleicht sie per rotationsinvariantem Dk-Test
   gegen alle vier Trainingssignaturen und prueft, ob der richtige Ort erkannt
   wird sowie wie genau die Winkelabweichung geschaetzt wird.
3. Wiederholt das fuer mehrere NO_BINS-Werte und schlaegt den Wert mit der
   groessten Trennmarge zwischen richtigem und falschem Match vor, sowie einen
   daraus abgeleiteten Dk-Schwellenwert.

Nutzung:
    python3 validate_offline.py
"""

import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from place_signature import (DEFAULT_DEPTH_BIN_EDGES, LocationSignature, dk,  # noqa: E402
                              estimate_rotation_shift, frequency_histogram, save_signature,
                              shift_to_degrees)
from synthetic_scan import make_raycaster, synthetic_scan  # noqa: E402

CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'map_config.yaml')
SIG_DIR = os.path.join(os.path.dirname(__file__), 'signatures_synthetic')
TRAINING_WAYPOINT_IDS = [2, 4, 6, 9]
CANDIDATE_NO_BINS = [36, 72, 120, 180, 360]
N_TRIALS_PER_LOCATION = 200
DEPTH_BIN_EDGES = DEFAULT_DEPTH_BIN_EDGES


def load_map():
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    walls_raw = cfg.get('walls', [])
    walls = [((w['x1'], w['y1']), (w['x2'], w['y2'])) for w in walls_raw]
    waypoints = {wp['id']: wp for wp in cfg['waypoints']}
    lidar_sigma = cfg['mcl']['lidar_sigma']
    return walls, waypoints, lidar_sigma


def build_training_signatures(walls, waypoints, n_bins):
    raycaster = make_raycaster(walls, n_bins)
    sigs = {}
    for wp_id in TRAINING_WAYPOINT_IDS:
        wp = waypoints[wp_id]
        ranges = synthetic_scan(raycaster, wp['x'], wp['y'], 0.0, noise_sigma=0.0)
        sigs[f'P{wp_id}'] = LocationSignature(location_id=f'P{wp_id}', angle_ranges=ranges)
    return sigs, raycaster


def _angle_err(a, b):
    d = a - b
    return (d + 180.0) % 360.0 - 180.0


def evaluate(n_bins, walls, waypoints, lidar_sigma, n_trials, rng):
    train_sigs, raycaster = build_training_signatures(walls, waypoints, n_bins)
    train_hists = {loc: frequency_histogram(sig.angle_ranges, DEPTH_BIN_EDGES)
                    for loc, sig in train_sigs.items()}

    correct = 0
    correct_dk_values = []      # Dk des RICHTIGEN Orts, in jedem Testlauf (auch wenn er gewinnt)
    closest_wrong_dk_values = []  # kleinster Dk unter den FALSCHEN Orten, in jedem Testlauf
    angle_errors = []

    for wp_id in TRAINING_WAYPOINT_IDS:
        wp = waypoints[wp_id]
        loc = f'P{wp_id}'
        for _ in range(n_trials):
            true_theta = rng.uniform(-np.pi, np.pi)
            test_ranges = synthetic_scan(raycaster, wp['x'], wp['y'], true_theta,
                                          noise_sigma=lidar_sigma, rng=rng)
            test_hist = frequency_histogram(test_ranges, DEPTH_BIN_EDGES)

            scores = {l: dk(test_hist, h) for l, h in train_hists.items()}
            best_loc = min(scores, key=scores.get)

            # Dk des richtigen Orts und kleinster Dk der falschen Orte -- IMMER erfassen,
            # nicht nur wenn tatsaechlich falsch erkannt wurde. Das ist die eigentliche
            # Sicherheitsmarge: wie nah kommt der naechstbeste falsche Ort im schlechtesten Fall?
            correct_dk_values.append(scores[loc])
            closest_wrong_dk_values.append(min(s for l, s in scores.items() if l != loc))

            if best_loc == loc:
                correct += 1
                shift, _, _ = estimate_rotation_shift(test_ranges, train_sigs[loc].angle_ranges)
                est_deg = shift_to_degrees(shift, n_bins)
                true_deg = np.degrees(true_theta)
                err = _angle_err(est_deg, -true_deg)
                angle_errors.append(err)

    total = len(TRAINING_WAYPOINT_IDS) * n_trials
    return dict(
        n_bins=n_bins,
        accuracy=correct / total,
        correct_dk_max=float(np.max(correct_dk_values)),
        wrong_dk_min=float(np.min(closest_wrong_dk_values)),
        mean_abs_angle_err=float(np.mean(np.abs(angle_errors))) if angle_errors else float('nan'),
        max_abs_angle_err=float(np.max(np.abs(angle_errors))) if angle_errors else float('nan'),
    )


def main():
    walls, waypoints, lidar_sigma = load_map()
    rng = np.random.default_rng(42)

    lines = []

    def out(s=''):
        print(s)
        lines.append(s)

    out(f'Wandmodell: {len(walls)} Segmente aus {os.path.relpath(CONFIG_PATH)}')
    out(f'Trainingsorte: {[f"P{i}" for i in TRAINING_WAYPOINT_IDS]}')
    out(f'Lidar-Sigma (aus map_config.yaml, mcl.lidar_sigma): {lidar_sigma} m')
    out(f'Monte-Carlo-Testfahrten pro Ort: {N_TRIALS_PER_LOCATION} (zufaellige Orientierung)')
    out('')
    out(f'{"NO_BINS":>8} {"Genauigkeit":>12} {"Dk korrekt(max)":>17} {"Dk falsch(min)":>16} '
        f'{"Trennmarge":>11} {"Winkelfehler mean/max":>23}')

    results = []
    for n_bins in CANDIDATE_NO_BINS:
        r = evaluate(n_bins, walls, waypoints, lidar_sigma, N_TRIALS_PER_LOCATION, rng)
        margin = r['wrong_dk_min'] - r['correct_dk_max']
        results.append((r, margin))
        out(f'{n_bins:>8} {r["accuracy"]*100:>11.1f}% {r["correct_dk_max"]:>17.4f} '
            f'{r["wrong_dk_min"]:>16.4f} {margin:>11.4f} '
            f'{r["mean_abs_angle_err"]:>10.2f}/{r["max_abs_angle_err"]:<6.2f} deg')

    best, best_margin = max(results, key=lambda rm: rm[1])
    threshold = (best['correct_dk_max'] + best['wrong_dk_min']) / 2.0

    out('')
    out('Empfehlung (groesste Trennmarge zwischen richtigem und naechstbestem falschem Match):')
    out(f'  NO_BINS       = {best["n_bins"]}')
    out(f'  Dk_threshold ~= {threshold:.4f}  (Mittelwert aus groesstem korrektem Dk='
        f'{best["correct_dk_max"]:.4f} und kleinstem falschem Dk={best["wrong_dk_min"]:.4f})')
    out(f'  Genauigkeit bei diesem NO_BINS: {best["accuracy"]*100:.1f}%')
    out(f'  Winkelfehler bei diesem NO_BINS: mean={best["mean_abs_angle_err"]:.2f} deg, '
        f'max={best["max_abs_angle_err"]:.2f} deg')
    out('')
    out('ACHTUNG: Diese Werte basieren auf dem synthetischen Wandmodell + gaussschem Rauschen')
    out('(lidar_sigma aus map_config.yaml). Reale Messungen (Mehrfachreflexionen, dynamische')
    out('Objekte im Sichtfeld, Montageabweichungen) koennen abweichen. Vor Uebernahme dieser')
    out('Werte in map_config.yaml unbedingt mit echten Lidar-Scans an P2/P4/P6/P9 pruefen')
    out('(siehe README.md, "Offene Punkte fuer Hardwaretest").')

    os.makedirs(SIG_DIR, exist_ok=True)
    train_sigs, _ = build_training_signatures(walls, waypoints, best['n_bins'])
    for loc, sig in train_sigs.items():
        save_signature(sig, os.path.join(SIG_DIR, f'{loc}.pkl'))
    out('')
    out(f'Synthetische Trainingssignaturen (NO_BINS={best["n_bins"]}) gespeichert in: '
        f'{os.path.relpath(SIG_DIR)}/')

    report_path = os.path.join(os.path.dirname(__file__), 'report.txt')
    with open(report_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'\nBericht gespeichert: {os.path.relpath(report_path)}')


if __name__ == '__main__':
    main()
