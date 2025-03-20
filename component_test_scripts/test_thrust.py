from pymavlink import mavutil
import serial
import time

baud_rate = 115200
arduino_port = ""
ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)
print(ser)
print("Arduino Connection Established!!")

pixhawk_port = ""
baud_rate = 115200
master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
master.wait_heartbeat()
print(master)
print("Pixhawk Connection Established!!")

def get_distance():
        got_distance = False
        while got_distance == False:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                distance_list = [float(i) for i in line.split(" ")]
            if distance_list:
                if (0<=distance_list[0]<=400) and (0<=distance_list[1]<=400) (0<=distance_list[2]<=400) and (0<=distance_list[3]<=400):
                    got_distance = True
                    print(distance_list)
                    return distance_list
                else:
                    continue
            else:
                continue

run_motor = True
channel = 3
pwm = 1200
rc_channel_values = [65535 for _ in range(18)]
while run_motor == True:
    rc_channel_values[channel-1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system, #target_system
        master.target_component, # target_component
        *rc_channel_values # RC channel list, in microseconds
    )

    distance = get_distance()
    if distance[0] < 20:
        run_motor = False