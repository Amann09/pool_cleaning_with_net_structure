from pymavlink import mavutil

pixhawk_port = ""
baud_rate = 115200
master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
master.wait_heartbeat()
print(master)
print("Pixhawk Connection Established!!")