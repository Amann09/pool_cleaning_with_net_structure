from pymavlink import mavutil
import numpy as np

pixhawk_port = ""
baud_rate = 115200
master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
master.wait_heartbeat()
print(master)
print("Pixhawk Connection Established!!")

def get_yaw():
    imu_active = True
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    while imu_active == True:
        if msg:
            yaw_deg = msg.yaw * 180 / np.pi
            imu_active = False  
            return yaw_deg
        else:
            print("No attitude data received within timeout. Continuing!!!")
            continue


start = (get_yaw() * (180/np.pi)) % 360
theta = 90
endpoint = (start + theta) % 360

rotate = True
while rotate:
    reading = (get_yaw() * (180/np.pi)) % 360
    rotate_condition = 0 <= abs(reading - endpoint) <= 10
    if rotate_condition == True:
        print("Target Reached!!!")
        rotate = False
    






