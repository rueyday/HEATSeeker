import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import zoom
import re

PORT = "/dev/cu.usbmodem103"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

plt.ion()
fig, ax = plt.subplots(figsize=(5,5))

data = np.zeros((8, 8))
interp = zoom(data, 8, order=3)

im = ax.imshow(interp, cmap="inferno", vmin=0, vmax=15)
plt.title("AMG8833 Thermal Camera")
plt.colorbar(im)
plt.tight_layout()

raw_frame = []
target_rc = None

def parse_line(line):
    nums = re.findall(r"-?\d+", line)
    if len(nums) == 8:
        return list(map(int, nums))
    return None

def parse_target(line):
    """
    Parse STM32 target line like:
    [Target]frame max idx=55 val=8, stable idx=55 (row=6 col=7)
    """
    if not line.startswith("[Target]"):
        return None

    m = re.search(r"row=(\d+)\s+col=(\d+)", line)
    if m:
        return int(m.group(1)), int(m.group(2))
    return None

# Draw a cyan 'X' marker on top of the heatmap
target_marker, = ax.plot([], [], marker='x', markersize=12, color='cyan', linewidth=2)

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue

    print("RX:", line)

    # --- Ignore debug lines for parsing ---
    if line.startswith("[debug]"):
        continue

    # --- Detect and update target ---
    rc = parse_target(line)
    if rc:
        target_rc = rc

    # --- Parse thermal rows ---
    row = parse_line(line)
    if row:
        raw_frame.append(row)

        if len(raw_frame) == 8:
            arr = np.array(raw_frame)
            interp = zoom(arr, 8, order=3)
            im.set_data(interp)

            # Draw target marker if we have one
            if target_rc:
                r, c = target_rc
                up_r = r * 8 + 4
                up_c = c * 8 + 4
                target_marker.set_data([up_c], [up_r])
            else:
                target_marker.set_data([], [])

            plt.pause(0.001)
            raw_frame = []

    if "====" in line:
        raw_frame = []
