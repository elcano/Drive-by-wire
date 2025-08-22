import os
import ydlidar
import time
import sys
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pandas as pd


RMAX = 32.0


fig = plt.figure()
fig.canvas.set_window_title('YDLidar LIDAR Monitor')
lidar_polar = plt.subplot(polar=True)
lidar_polar.autoscale_view(True,True,True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)
ports = ydlidar.lidarPortList();
port = "/dev/ydlidar";
for key, value in ports.items():
    port = value;
    
laser = ydlidar.CYdLidar();
laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 20);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
laser.setlidaropt(ydlidar.LidarPropMaxRange, 32.0);
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01);
scan = ydlidar.LaserScan()

def find_close_sequences(angle, ran, threshold=0.015):
    i = 0
    angle_sequences = []
    ran_sequences = []

    while i < len(angle) - 1:
        start = i
        while i < len(angle) - 1 and abs(angle[i] - angle[i + 1]) <= threshold:
            i += 1
        if i > start:
            angle_sequences.append(angle[start:i + 1])
            ran_sequences.append(ran[start:i + 1])
        i += 1

    return angle_sequences, ran_sequences

def graph_points(sequences):
    points = []

    for i, seq in enumerate(sequences):
        start = 0
        end = start + len(seq) - 1
        mid = (start + end) // 2
        points.append([start, mid, end])

    return points



def animate(num):
    
    r = laser.doProcessSimple(scan);
    if r:
        angle = []
        ran = []
        intensity = []
        obs = []
        obs2 = []

        for point in scan.points:
            if point.range < 5:
                angle.append(point.angle)
                ran.append(point.range)
                intensity.append(point.intensity)

		
        angle_objects, ran_objects = find_close_sequences(angle, ran, threshold=0.015)
        
        angle_gp = graph_points(angle_objects)
        ran_gp = graph_points(ran_objects)
        
                
        lidar_polar.clear()
        lidar_polar.scatter(angle, ran, c=intensity, cmap='hsv', alpha=0.95)

	if point.range < .01:
		print("Number of objects detected: ", len(angle_objects));
        	print("Object detected, shutting down the program...");

        	for i in range(len(angle_objects)):
         	   lidar_polar.plot(angle_objects[i], ran_objects[i], marker='o', markersize=3, 	linestyle='-', linewidth=1, alpha=0.5)
        	   lidar_polar.plot([angle_gp[i][0], angle_gp[i][1], angle_gp[i][2]], [ran_gp[i][0], ran_gp[i][1], ran_gp[i][2]], marker='o', markersize=3, linestyle='-', linewidth=1, alpha=0.5)
   	     	sys.exit();
        

    dfa = pd.DataFrame({'Angle': angle_objects, 'Range': ran_objects})
    dfa.to_excel('data.xlsx', index=False)



ret = laser.initialize();
if ret:
    ret = laser.turnOn();
    if ret:
        ani = animation.FuncAnimation(fig, animate, interval=50)
        plt.show()
    laser.turnOff();

laser.disconnecting();
plt.close();


