from pymavlink import mavutil
import math
import numpy as np
import time
import scipy

# Initialize MAVLink connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))

def get_yaw(master):
    imu_active = True
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    while imu_active == True:
        if msg:
            yaw_deg = msg.yaw * 180 / np.pi
            if yaw_deg < 0:
                yaw_deg = yaw_deg + 360
                print("From yaw function: ", yaw_deg)
            imu_active = False 
            print(yaw_deg) 
            return yaw_deg
        else:
            print("No attitude data received within timeout. Continuing!!!")
            continue

theta = 90
START_THETA = get_yaw(master) % 360
print("starting: ",START_THETA)
endpointRIGHT = (START_THETA + theta) % 360
print("Right end point: ", endpointRIGHT)

target_not_reached = True
while target_not_reached:
    print(get_yaw(master))
    if 0 <= abs(get_yaw(master) - endpointRIGHT) <= 10:
        print("Target Reached")
        target_not_reached = False
    else:
        continue


