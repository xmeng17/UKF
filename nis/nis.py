import matplotlib.pyplot as plt
import numpy as np


with open('nis_radar.txt') as f:
    radar = f.read().split("\n")
del(radar[0])
del(radar[-1])

x_radar = [];
nis_radar = [];

for line in radar:
    line_arr = line.split(",")
    if line_arr[0] != 0:
        try:
            x_radar.append(float(line_arr[1]))
            nis_radar.append(float(line_arr[0]))
        except Exception:
            print(line)

with open('nis_lidar.txt') as f:
    lidar = f.read().split("\n")
del(lidar[0])
del(lidar[-1])

x_lidar = [];
nis_lidar = [];

for line in lidar:
    line_arr = line.split(",")
    if line_arr[0] != 0:
        try:
            x_lidar.append(float(line_arr[1]))
            nis_lidar.append(float(line_arr[0]))
        except Exception:
            print(line)

plt.figure(1)
plt.plot(x_radar,nis_radar,label="Radar NIS")
plt.plot(x_radar,[7.815 for i in range(len(x_radar))],label="Radar Chi-Square 95%")
plt.legend()

plt.figure(2)
plt.plot(x_lidar,nis_lidar,label="Lidar NIS")
plt.plot(x_lidar,[5.991 for i in range(len(x_lidar))],label="Lidar Chi-Square 95%")
plt.legend()
plt.show()
