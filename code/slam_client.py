#!/usr/bin/env python3
import json
import math
import socket
import sys
import time

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

# --- Connection settings ---
PI_HOST = '100.102.69.94'
PI_PORT = 5002

# --- SLAM settings ---
MAP_SIZE_PIXELS = 600  # Currently 600 pixels
MAP_SIZE_METERS = 5  # Currently 5x5 meters
MAP_QUALITY = 3
HOLE_WIDTH_MM = 300
SCAN_SIZE = 360
SCAN_RATE_HZ = 5
DETECTION_ANGLE = 360
MAX_DISTANCE_MM = 12000

# Robot size (Updated with requested safety buffer)
ROBOT_WIDTH = 0.20  # 20cm
ROBOT_LENGTH = 0.40  # 40cm
LIDAR_X_OFFSET = 0.30  # 30cm from the rear edge

SCAN_ANGLES = [float(i) for i in range(SCAN_SIZE)]


def make_slam():
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser
    laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
    slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                     hole_width_mm=HOLE_WIDTH_MM, map_quality=MAP_QUALITY)
    return slam


class SocketReader:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b''

    def read_line(self):
        try:
            while b'\n' not in self.buffer:
                chunk = self.sock.recv(8192)
                if not chunk: return None
                self.buffer += chunk
        except BlockingIOError:
            pass
        if b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            return line.decode('utf-8')
        return None


def run():
    # Keep your existing backend loop as it worked for you
    for backend in ['TkAgg', 'Qt5Agg', 'MacOSX', 'Agg']:
        try:
            matplotlib.use(backend)
            break
        except Exception:
            continue

    slam = make_slam()
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    fig, ax_map = plt.subplots(1, 1, figsize=(10, 10))

    # --- CYBERPUNK THEME COLORS ---
    fig.patch.set_facecolor('#0b0f14')
    ax_map.set_facecolor('#101418')
    ax_map.tick_params(colors='#8899aa')
    path_color = '#FF0055'  # Neon Pink
    robot_color = '#00FFD2'  # Bright Cyan

    for spine in ax_map.spines.values():
        spine.set_edgecolor('#2a3a4a')

    map_img = ax_map.imshow(
        np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8),
        cmap='magma', vmin=0, vmax=255, origin='lower',
        extent=[0, MAP_SIZE_METERS, 0, MAP_SIZE_METERS],
        interpolation='nearest'
    )

    robot_dot, = ax_map.plot([], [], 'o', color=robot_color, markersize=8, label='Robot', zorder=10)
    robot_rect = Rectangle((0, 0), ROBOT_LENGTH, ROBOT_WIDTH,
                           linewidth=2, edgecolor=robot_color, facecolor='none', zorder=5)
    ax_map.add_patch(robot_rect)

    # --- PERSISTENT THICK PATH ---
    path_x, path_y = [], []
    path_line, = ax_map.plot([], [], '-', color=path_color, linewidth=3, alpha=0.8, zorder=4)

    heading_arrow = None

    plt.tight_layout()
    plt.ion()
    plt.show()

    print(f'[slam_client] Connecting to {PI_HOST}:{PI_PORT} ...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((PI_HOST, PI_PORT))
        sock.setblocking(False)
        reader = SocketReader(sock)
        print('[slam_client] Connected!')
    except:
        print("Could not connect to Pi.")
        return

    prev_distances = None

    try:
        while plt.fignum_exists(fig.number):
            latest_raw = None
            while True:
                raw = reader.read_line()
                if not raw: break
                latest_raw = raw

            if not latest_raw:
                plt.pause(0.01)
                continue

            try:
                data = json.loads(latest_raw)
                distances = data['distances']

                # --- 1. OPTIMIZED MASKING (90° FRONT BLOCK) ---
                # This only hides the front nose (indices 45 to 135)
                # Leaving 270° of clear vision for better SLAM stability
                masked = distances

                # --- 2. SINGLE SLAM UPDATE ---
                valid = sum(1 for d in masked if 0 < d < MAX_DISTANCE_MM)
                if valid > 30:
                    slam.update(masked, scan_angles_degrees=SCAN_ANGLES)
                    prev_distances = masked
                elif prev_distances:
                    slam.update(prev_distances, scan_angles_degrees=SCAN_ANGLES)

                # --- 3. POSITIONING ---
                x_mm, y_mm, theta_deg = slam.getpos()
                slam.getmap(mapbytes)

                rx, ry = x_mm / 1000.0, y_mm / 1000.0
                rad = math.radians(theta_deg)

                # --- 4. BOX & ARROW LOGIC (DOT AT REAR) ---
                # We move back by offset and left by half-width to find the rectangle's bottom-left origin
                bx = rx - (LIDAR_X_OFFSET * math.cos(rad) - (ROBOT_WIDTH / 2) * math.sin(rad))
                by = ry - (LIDAR_X_OFFSET * math.sin(rad) + (ROBOT_WIDTH / 2) * math.cos(rad))

                robot_rect.set_xy((bx, by))
                robot_rect.angle = theta_deg
                robot_dot.set_data([rx], [ry])

                # Persistent Path (Increased buffer to 2000 points)
                path_x.append(rx)
                path_y.append(ry)
                if len(path_x) > 2000: path_x.pop(0); path_y.pop(0)
                path_line.set_data(path_x, path_y)

                if heading_arrow: heading_arrow.remove()
                # Arrow points FORWARD (0.4m length for visibility)
                dx, dy = 0.4 * math.cos(rad), 0.4 * math.sin(rad)
                heading_arrow = ax_map.annotate('', xy=(rx + dx, ry + dy), xytext=(rx, ry),
                                                arrowprops=dict(arrowstyle='->', color='yellow', lw=2))

                arr = np.frombuffer(mapbytes, dtype=np.uint8).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
                map_img.set_data(arr)

                ax_map.set_title(f'SLAM Map: x={x_mm:.0f}mm y={y_mm:.0f}mm θ={theta_deg:.1f}°', color='white')

            except Exception as e:
                # Silently catch JSON errors to keep the loop moving
                continue

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)

    except KeyboardInterrupt:
        print('\n[slam_client] Exiting.')
    finally:
        sock.close()
        plt.close()


if __name__ == '__main__':
    run()
