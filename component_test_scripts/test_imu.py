from pymavlink import mavutil
import math
import numpy as np
import time
import scipy

# Initialize MAVLink connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))


while True:
    msg1 = master.recv_match(type='RAW_IMU', blocking=True)
    print(msg1)
    
