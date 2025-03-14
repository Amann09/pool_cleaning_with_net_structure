from pymavlink import mavutil
import numpy as np
# import math
import time
import serial
import random
from enum import Enum, auto


class Turn(Enum):
    CLOCKWISE = auto()
    ANTI_CLOCKWISE = auto()
    
class Hit(Enum):
    FRONT_BOTH = auto()

    RIGHT_SIDE = auto()
    LEFT_SIDE = auto()

    FRONT_SIDE_RIGHT = auto()
    FRONT_SIDE_LEFT = auto()
    
    RIGHT_FRONT_ONLY = auto()
    LEFT_FRONT_ONLY = auto()


class Controller:
    def __init__(self, master, ser):
        self.master = master
        self.ser = ser

        self.time_horizon = 3600 #in seconds
        self.safe_distance = 70 #in cm
        self.angles = [15, 30, 45, 60, 75, 90, 170]

        self.rc_channel_values = [65535 for _ in range(18)]
    


    def get_distance(self):
        got_distance = False
        while got_distance == False:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                distance_list = [float(i) for i in line.split(" ")]
                
                if (0<=distance_list[0]<=400) and (0<=distance_list[1]<=400) and (0<=distance_list[2]<=400) and (0<=distance_list[3]<=400):
                    got_distance = True
                    return distance_list
                else:
                    continue
    


    def turn_initial(self, channel, pwm, condition):
        run_motor = True
        while run_motor == True:
            self.rc_channel_values[channel-1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system, #target_system
                self.master.target_component, # target_component
                *self.rc_channel_values # RC channel list, in microseconds
            )
            if condition == False:
                run_motor = False
        return



    def turn_rotate(self, channel, pwm, endpoint):
        run_motor = True
        while run_motor:
            self.rc_channel_values[channel-1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system, #target_system
                self.master.target_component, # target_component
                *self.rc_channel_values # RC channel list, in microseconds
            )
            reading = (self.get_yaw()) % 360
            if 0 <= abs(reading - endpoint) <= 10:
                run_motor = False
        return

    

    def get_yaw(self):
        imu_active = True
        msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        while imu_active == True:
            if msg:
                yaw_deg = msg.yaw * 180 / np.pi
                if yaw_deg < 0:
                    yaw_deg = yaw_deg + 360
                imu_active = False  
                return yaw_deg
            else:
                print("No attitude data received within timeout. Continuing!!!")
                continue


    
    def sign(self):
        hit = None
        print(f"Sensor values: Front right - {self.get_distance()[0]}, Front left - {self.get_distance()[1]}, Right - {self.get_distance()[2]}, Left - {self.get_distance()[3]}")

        if (self.get_distance()[0] < self.safe_distance + 30) and (self.get_distance()[2] < self.safe_distance + 30):
            hit = Hit.FRONT_SIDE_RIGHT
            print(f"hit: {hit}")
            return -1

        elif (self.get_distance()[1] < self.safe_distance + 30) and (self.get_distance()[3] < self.safe_distance + 30):
            hit = Hit.FRONT_SIDE_LEFT
            print(f"hit: {hit}")
            return 1
        
        elif (self.get_distance()[0] < self.safe_distance + 30) and (self.get_distance()[1] < self.safe_distance + 30):
            hit = Hit.FRONT_BOTH
            print(f"hit: {hit}")
            return 1
        
        elif self.get_distance()[0] < self.safe_distance + 30:
            hit = Hit.RIGHT_FRONT_ONLY
            print(f"hit: {hit}")
            return -1
        
        elif self.get_distance()[1] < self.safe_distance + 30:
            hit = Hit.LEFT_FRONT_ONLY
            print(f"hit: {hit}")
            return 1
        
        elif self.get_distance()[2] < self.safe_distance + 30:
            hit = Hit.RIGHT_SIDE
            print(f"hit: {hit}")
            return -1
        
        elif self.get_distance()[3] < self.safe_distance + 30:
            hit = Hit.LEFT_SIDE
            print(f"hit: {hit}")
            return 1

        else: 
            return 1

        # if hit == Hit.FRONT_SIDE_RIGHT:
        #     return -1
        # elif hit == Hit.FRONT_SIDE_LEFT:
        #     return +1
        # elif hit == Hit.FRONT_BOTH:
        #     return +1
        # elif hit == Hit.RIGHT_FRONT_ONLY:
        #     return -1
        # elif hit == Hit.LEFT_FRONT_ONLY:
        #     return +1
        # elif hit == Hit.RIGHT_SIDE:
        #     return -1
        # elif hit == Hit.LEFT_SIDE:
        #     return +1
        # else:
        #     return +1



    def rotate(self, theta):
        if theta > 0:
            turn = Turn.CLOCKWISE
        else:
            turn = Turn.ANTI_CLOCKWISE

        if turn == Turn.CLOCKWISE:
            START_THETA = (self.get_yaw()) % 360
            endpointRIGHT = (START_THETA + theta) % 360
            endpointLEFT = (START_THETA - theta) % 360

            print(f"Rotating the vehicle in {turn} direction")
            print(f"Rotating: Sensor Values: (Right_Front, Left_Front, Right, Left) -> ({self.get_distance()[0]}, {self.get_distance()[1]}, {self.get_distance()[2]}, {self.get_distance()[3]})")

            # reading = (self.get_yaw() * (180/np.pi)) % 360
            # print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            # print(f' [turning right {theta} degrees]')

            reading = (self.get_yaw()) % 360
            print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            print(f' [turning right {theta} degrees]')

            rotate_condition = 0 <= abs(reading - endpointRIGHT) <= 10
            channel = 1
            pwm = 1700
            
            self.turn_rotate(channel, pwm, endpointRIGHT)

            # rotate = True
            # while rotate:
            #     print("Turning right!!")
            #     self.turn_rotate(channel, pwm, endpointRIGHT)
            #     if rotate_condition == True:
            #         print("Turning right done!!")
            #         rotate = False
            #         self.initial()
            #     else:
            #         continue

            # rotating = True
            # while rotating == True:
            #     reading = (self.get_yaw() * (180/np.pi)) % 360
            #     print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            #     print(f' [turning right {theta} degrees]')
            #     rotate_condition = 0 <= abs(reading - endpointRIGHT) <= 10
            #     self.turn_rotate(channel, pwm, endpointRIGHT)
            #     if rotate_condition == True:
            #         rotating = False


            # if rotate_condition == True:
            #     print("Turning right complete!")
            #     self.initial()
            # elif rotate_condition == False:
            #     print("Turning Right!!")
            #     self.turn_rotate(channel, pwm, endpointRIGHT)

                # run_motor = True
                # while run_motor:
                #     self.rc_channel_values[channel-1] = pwm
                #     self.master.mav.rc_channels_override_send(
                #         self.master.target_system, #target_system
                #         self.master.target_component, # target_component
                #         *self.rc_channel_values # RC channel list, in microseconds
                #     )
                #     reading = (self.get_yaw() * (180/np.pi)) % 360
                #     print("Reading: ", reading, " EndpointRight: ", endpointRIGHT)
                #     if 0 <= abs(reading - endpointRIGHT) <= 10:
                #         run_motor = False


        if turn == Turn.ANTI_CLOCKWISE:
            START_THETA = (self.get_yaw()) % 360
            endpointRIGHT = (START_THETA + theta) % 360
            endpointLEFT = (START_THETA - theta) % 360

            print(f"Rotating the vehicle in {turn} direction")
            print(f"Rotating: Sensor Values: (Right_Front, Left_Front, Right, Left) -> ({self.get_distance()[0]}, {self.get_distance()[1]}, {self.get_distance()[2]}, {self.get_distance()[3]})")

            reading = (self.get_yaw()) % 360
            print(f"(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            print(f' [turning left {theta} degrees]')

            rotate_condition = 0 <= abs(reading - endpointLEFT) <= 10
            channel = 1
            pwm = 1200

            self.turn_rotate(channel, pwm, endpointLEFT)
            
            # rotate = True
            # while rotate:
            #     print("Turning left!!")
            #     self.turn_rotate(channel, pwm, endpointLEFT)
            #     if rotate_condition == True:
            #         print("Turning left done!!")
            #         rotate = False
            #         self.initial()
            #     else:
            #         continue

            # if rotate_condition == True:
            #     print("Turning left complete!")
            #     self.initial()
            # elif rotate_condition == False:
            #     print("Turning Left!!")
            #     self.turn_rotate(channel, pwm, endpointLEFT)

                # run_motor = True
                # while run_motor:
                #     self.rc_channel_values[channel-1] = pwm
                #     self.master.mav.rc_channels_override_send(
                #         self.master.target_system, #target_system
                #         self.master.target_component, # target_component
                #         *self.rc_channel_values # RC channel list, in microseconds
                #     )
                #     reading = (self.get_yaw() * (180/np.pi)) % 360
                #     print("Reading: ", reading, " EndpointLeft: ", endpointLEFT)
                #     if 0 <= abs(reading - endpointLEFT) <= 10:
                #         run_motor = False



    def initial(self):
        start_time = time.time()
        while (time.time() - start_time) < self.time_horizon:
            print(f"Sensor values: Front right - {self.get_distance()[0]}, Front left - {self.get_distance()[1]}, Right - {self.get_distance()[2]}, Left - {self.get_distance()[3]}")
            
            conditions = [
                        (self.get_distance()[0] < self.safe_distance) or (self.get_distance()[1] < self.safe_distance),
                        (self.get_distance()[3] < self.safe_distance - 30),
                        (self.get_distance()[2] < self.safe_distance - 30),
                        (self.get_distance()[0] > self.safe_distance) or (self.get_distance()[1] > self.safe_distance) and (self.get_distance()[3] > self.safe_distance - 50) and (self.get_distance()[2] > self.safe_distance - 50)
                        ]
                           
            if conditions[0] == True:
                angles = self.angles
                random.shuffle(angles)
                for angle in angles:
                    factor = self.sign()
                    self.rotate(factor*angle)
            
            if conditions[1] == True:
                print(f"Left wall detected at distance {self.get_distance()[3]}")
                pwm = 1700
                channel = 1
                self.turn_initial(channel, pwm, conditions[1])
                # run_motor = True
                # while run_motor == True:
                #     self.rc_channel_values[channel-1] = pwm
                #     self.master.mav.rc_channels_override_send(
                #         self.master.target_system, #target_system
                #         self.master.target_component, # target_component
                #         *self.rc_channel_values # RC channel list, in microseconds
                #     )
                #     if conditions[1] == False:
                #         run_motor = False

            elif conditions[2] == True:
                print(f"Right wall detected at distance {self.get_distance()[2]}")
                pwm = 1200
                channel = 1
                self.turn_initial(channel, pwm, conditions[2])
                run_motor = True
                # while run_motor == True:
                #     self.rc_channel_values[channel-1] = pwm
                #     self.master.mav.rc_channels_override_send(
                #         self.master.target_system, #target_system
                #         self.master.target_component, # target_component
                #         *self.rc_channel_values # RC channel list, in microseconds
                #     )
                #     if conditions[2] == False:
                #         run_motor = False

            elif conditions[3] == True:
                print("Going straight")
                pwm = 1600
                channel = 3
                self.turn_initial(channel, pwm, conditions[3])
                # run_motor = True
                # while run_motor == True:
                #     self.rc_channel_values[channel-1] = pwm
                #     self.master.mav.rc_channels_override_send(
                #         self.master.target_system, #target_system
                #         self.master.target_component, # target_component
                #         *self.rc_channel_values # RC channel list, in microseconds
                #     )
                #     if conditions[3] == False:
                #         run_motor = False



def main():
    baud_rate = 115200
    pixhawk_port = "/dev/ttyACM0"
    master = mavutil.mavlink_connection(pixhawk_port, baud=baud_rate)
    master.wait_heartbeat()
    print("Heartbeat recieved from the system!!")
    print(f"Connected to pixhawk at {pixhawk_port}")

    arduino_port = "/dev/ttyACM1"
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    print(f"Connected to Arduino Mega at {arduino_port}")

    print("All connections established!")

    control = Controller(master, ser)
    control.initial()


if __name__=='__main__':
    main()