import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import zoom
import re

PORT = "/dev/cu.usbmodem1103"
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

def parse_line(line):
    nums = re.findall(r"-?\d+", line)
    if len(nums) == 8:
        return list(map(int, nums))
    return None

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue

    print("RX:", line)

    row = parse_line(line)
    if row:
        raw_frame.append(row)

        if len(raw_frame) == 8:
            arr = np.array(raw_frame)

            interp = zoom(arr, 8, order=3)

            im.set_data(interp)
            plt.pause(0.001)

            raw_frame = []
    if "====" in line:
        raw_frame = []
