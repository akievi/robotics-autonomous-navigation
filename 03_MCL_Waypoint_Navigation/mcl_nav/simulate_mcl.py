"""
Offline-Test des MCL-Filters ohne Hardware.
Synthetische Scans aus der Karte, verrauschte Odometrie.
Zeigt Partikelwolke in matplotlib.
"""

import numpy as np
import yaml
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def load_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('mcl_nav'), 'config', 'map_config.yaml')
    except Exception:
        path = os.path.join(os.path.dirname(__file__), '..', 'config', 'map_config.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def draw_walls(ax, walls):
    for w in walls:
        ax.plot([w[0][0], w[1][0]], [w[0][1], w[1][1]], 'k-', linewidth=2)


def draw_particles(ax, particles, weights, color='blue', alpha=0.3):
    sc = ax.scatter(particles[:, 0], particles[:, 1],
                    s=2, c=color, alpha=alpha)
    return sc


def main():
    cfg = load_config()
    mcl_cfg = cfg['mcl']
    waypoints = cfg['waypoints']
    walls_raw = cfg.get('walls', [])

    if len(walls_raw) == 0:
        print("WARNUNG: Keine Wandsegmente in map_config.yaml — Raycast deaktiviert.")
        print("MCL laeuft ohne Messung (nur Motion Model).")

    walls = [((w['x1'], w['y1']), (w['x2'], w['y2'])) for w in walls_raw] if walls_raw else []

    from mcl_nav.mcl import MCL
    mcl = MCL(mcl_cfg, walls)

    # Startpose = P1
    start = waypoints[0]
    mcl.init_pose(start['x'], start['y'], np.deg2rad(start['theta']))

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('MCL Simulation')
    draw_walls(ax, walls)

    # Waypoints einzeichnen
    for wp in waypoints:
        ax.plot(wp['x'], wp['y'], 'g*', markersize=10)
        ax.annotate(f"P{wp['id']}", (wp['x'], wp['y']), textcoords='offset points',
                    xytext=(5, 5), fontsize=8)

    # Simulierter Pfad: fahre alle Waypoints ab
    true_x, true_y, true_theta = start['x'], start['y'], np.deg2rad(start['theta'])
    path_x, path_y = [true_x], [true_y]
    est_x,  est_y  = [true_x], [true_y]

    print(f"Start: ({true_x:.2f}, {true_y:.2f}, {np.rad2deg(true_theta):.1f}°)")

    segment_length = mcl_cfg['update_distance']

    for wp in waypoints[1:]:
        target_x = wp['x']
        target_y = wp['y']
        target_theta = np.deg2rad(wp['theta'])

        # Drehe zur Zielrichtung
        desired_theta = np.arctan2(target_y - true_y, target_x - true_x)
        delta_theta_true = desired_theta - true_theta
        delta_theta_true = (delta_theta_true + np.pi) % (2 * np.pi) - np.pi

        # Verrauschte Odometrie fuer Drehung
        delta_theta_odom = delta_theta_true + np.random.normal(0, 0.02 * abs(delta_theta_true) + 0.01)
        mcl.motion_predict(0.0, delta_theta_odom)
        true_theta = desired_theta

        # Fahre in Segmenten
        total_dist = np.hypot(target_x - true_x, target_y - true_y)
        driven = 0.0

        while driven < total_dist:
            step = min(segment_length, total_dist - driven)

            # Wahre Bewegung
            true_x += step * np.cos(true_theta)
            true_y += step * np.sin(true_theta)
            driven += step
            path_x.append(true_x)
            path_y.append(true_y)

            # Verrauschte Odometrie
            step_odom = step + np.random.normal(0, 0.02 * step + 0.001)
            mcl.motion_predict(step_odom, 0.0)

            # Synthetischer Scan (nur wenn Waende vorhanden)
            if len(walls) > 0:
                scan_angles = np.linspace(-np.pi, np.pi, 720, endpoint=False)
                # Erzeuge perfekten Scan an wahrer Pose, dann Rauschen drauf
                mcl_tmp = MCL(mcl_cfg, walls)
                mcl_tmp.particles[0] = [true_x, true_y, true_theta]
                perfect = mcl_tmp._raycast_single_particle(true_x, true_y, true_theta)
                # Interpoliere auf 720 Strahlen
                ray_a = np.linspace(-np.pi, np.pi, len(perfect), endpoint=False)
                scan_ranges = np.interp(scan_angles, ray_a, perfect)
                scan_ranges += np.random.normal(0, 0.02, len(scan_ranges))
                scan_ranges = np.clip(scan_ranges, 0.1, 12.0)

                mcl.measurement_update(scan_ranges, scan_angles)
                mcl.resample()

            ex, ey, et = mcl.estimate_pose()
            est_x.append(ex)
            est_y.append(ey)

        neff = mcl.effective_particle_count()
        ex, ey, et = mcl.estimate_pose()
        err = np.hypot(ex - true_x, ey - true_y)
        print(f"P{wp['id']:2d}: wahr=({true_x:.2f},{true_y:.2f}) "
              f"est=({ex:.2f},{ey:.2f}) err={err*100:.1f}cm N_eff={neff:.0f}")

    # Plot
    ax.plot(path_x, path_y, 'g--', linewidth=1, label='Wahrer Pfad')
    ax.plot(est_x,  est_y,  'r-',  linewidth=1, label='MCL-Schaetzung')
    draw_particles(ax, mcl.particles, mcl.weights)
    ax.legend()
    plt.tight_layout()
    plt.savefig('/tmp/mcl_simulation.png', dpi=150)
    print("\nPlot gespeichert: /tmp/mcl_simulation.png")
    plt.show()


if __name__ == '__main__':
    main()
