"""robot_controller_2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import pi
from enum import Enum, auto
import random

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

# create the Robot instance.
robot = Robot()
pi = round(pi, 3)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep()) # timestep = 32
max_speed = 6.28

safe_distance = 0.35 # in meters


# Motors
FRONT_SIDE_RIGHT_motor = robot.getMotor('front_motor_1')
FRONT_SIDE_LEFT_motor = robot.getMotor('front_motor_2')

FRONT_SIDE_RIGHT_motor.setPosition(float('inf'))
FRONT_SIDE_RIGHT_motor.setVelocity(0.0)
    
FRONT_SIDE_LEFT_motor.setPosition(float('inf'))
FRONT_SIDE_LEFT_motor.setVelocity(0.0)


back_right_motor = robot.getMotor('back_motor_1')
back_left_motor = robot.getMotor('back_motor_2')
    
back_right_motor.setPosition(float('inf'))
back_right_motor.setVelocity(0.0)
    
back_left_motor.setPosition(float('inf'))
back_left_motor.setVelocity(0.0)


# Distance Sensors
right_front_distance_sensor = robot.getDevice("distance_sensor_rightFront")
left_front_distance_sensor = robot.getDevice("distance_sensor_leftFront")
side_right_distance_sensor = robot.getDevice("distance_sensor_side_right")
side_left_distance_sensor = robot.getDevice("distance_sensor_side_left")

right_front_distance_sensor.enable(timestep)
left_front_distance_sensor.enable(timestep)
side_right_distance_sensor.enable(timestep)
side_left_distance_sensor.enable(timestep)

# Inertial Unit
imu = robot.getDevice('inertial unit')
imu.enable(timestep)


def sign():
    hit = None
    while robot.step(timestep) != -1:
        right_front_value = right_front_distance_sensor.getValue()
        left_front_value = left_front_distance_sensor.getValue()
        side_right_value = side_right_distance_sensor.getValue()
        side_left_value = side_left_distance_sensor.getValue()
        
        print(f"Angles\n(Right_Front, Left_Front, Right, Left) -> ({right_front_value}, {left_front_value}, {side_right_value}, {side_left_value})")
        print(' ')
        
        if right_front_value < 0.5 and side_right_value < 0.5:
            hit = Hit.FRONT_SIDE_RIGHT 
            print(f"hit: {hit}")
        elif left_front_value < 0.5 and side_left_value < 0.5:
            hit = Hit.FRONT_SIDE_LEFT
            print(f"hit: {hit}")
        elif right_front_value < 0.5 and left_front_value < 0.5:
            hit = Hit.FRONT_BOTH
            print(f"hit: {hit}")
        elif right_front_value < 0.5:
            hit = Hit.RIGHT_FRONT_ONLY
            print(f"hit: {hit}")
        elif left_front_value < 0.5:
            hit = Hit.LEFT_FRONT_ONLY
            print(f"hit: {hit}")
        elif side_right_value < 0.5:
            hit = Hit.RIGHT_SIDE
            print(f"hit: {hit}")
        elif side_left_value < 0.5:
            hit = Hit.LEFT_SIDE
            print(f"hit: {hit}")
            
            
        if hit == Hit.FRONT_SIDE_RIGHT:
            return -1
        elif hit == Hit.FRONT_SIDE_LEFT:
            return +1
        elif hit == Hit.FRONT_BOTH:
            return +1
        elif hit == Hit.RIGHT_FRONT_ONLY:
            return -1
        elif hit == Hit.LEFT_FRONT_ONLY:
            return +1
        elif hit == Hit.RIGHT_SIDE:
            return -1
        elif hit == Hit.LEFT_SIDE:
            return +1
        else:
            return +1


def rotate(theta):
    if theta > 0:
        turn = Turn.CLOCKWISE
    else:
        turn = Turn.ANTI_CLOCKWISE
    
    START_THETA = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
    endpointRIGHT = (START_THETA + theta) % 360
    endpointLEFT = (START_THETA - theta) % 360
    
    print(f"Rotating the vehicle {turn}")
    print(' ')


    if turn == Turn.CLOCKWISE:
        endpointRIGHT = (START_THETA - theta) % 360
        endpointLEFT = (START_THETA + theta) % 360
        
        while robot.step(timestep) != -1:
            right_front_value = right_front_distance_sensor.getValue()
            left_front_value = left_front_distance_sensor.getValue()
            side_right_value = side_right_distance_sensor.getValue()
            side_left_value = side_left_distance_sensor.getValue()
            
            print(f"Rotate\n(Right_Front, Left_Front, Right, Left) -> ({right_front_value}, {left_front_value}, {side_right_value}, {side_left_value})")
            
            reading = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
            print(f"\n(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            
            print(f' [turning right {theta} degrees]')
            FRONT_SIDE_LEFT_motor.setVelocity(max_speed)
            FRONT_SIDE_RIGHT_motor.setVelocity(-max_speed)
    
            back_left_motor.setVelocity(max_speed)
            back_right_motor.setVelocity(-max_speed)
            
            if 0 <= abs(reading - endpointRIGHT) <= 10:
            # if reading < endpointRIGHT:
                print("Turning Right complete")
                forward()
                    
    if turn == Turn.ANTI_CLOCKWISE:
        endpointRIGHT = (START_THETA + theta) % 360
        endpointLEFT = (START_THETA - theta) % 360

        while robot.step(timestep) != -1:
            right_front_value = right_front_distance_sensor.getValue()
            left_front_value = left_front_distance_sensor.getValue()
            side_right_value = side_right_distance_sensor.getValue()
            side_left_value = side_left_distance_sensor.getValue()
            
            print(f"Rotate\n(Right_Front, Left_Front, Right, Left) -> ({right_front_value}, {left_front_value}, {side_right_value}, {side_left_value})")
            
            reading = (imu.getRollPitchYaw()[2] * (180/pi)) % 360
            print(f"\n(START_THETA, Reading, endPointRight, endPointLeft) -> ({START_THETA}, {reading}, {endpointRIGHT}, {endpointLEFT})")
            
            print(f' [turning left {theta} degrees]')
            FRONT_SIDE_LEFT_motor.setVelocity(-max_speed)
            FRONT_SIDE_RIGHT_motor.setVelocity(max_speed)
    
            back_left_motor.setVelocity(-max_speed)
            back_right_motor.setVelocity(max_speed)  
            
            # if reading > endpointLEFT:
                # print("Turning Left complete")
                # forward()

            if 0 <= abs(reading - endpointLEFT) <= 10:
               print("Turning Left complete")
               forward()
               

def stop():

    left_speed = 0
    right_speed = 0
    
    FRONT_SIDE_RIGHT_motor.setVelocity(right_speed)
    FRONT_SIDE_LEFT_motor.setVelocity(left_speed)
        
    back_right_motor.setVelocity(right_speed)
    back_left_motor.setVelocity(left_speed)


def forward():
    while robot.step(timestep) != -1:
        right_front_value = right_front_distance_sensor.getValue()
        left_front_value = left_front_distance_sensor.getValue()
        side_right_value = side_right_distance_sensor.getValue()
        side_left_value = side_left_distance_sensor.getValue()
        
        print(f"Forward\n(Right_Front, Left_Front, Right, Left) -> ({right_front_value}, {left_front_value}, {side_right_value}, {side_left_value})")
        # print("----------------------------------------------------------------------------------")
        # print(f"IMU Roll Pitch Yaw: {imu.getRollPitchYaw()}")
        print(' ')

        left_speed = max_speed
        right_speed = max_speed
        
        if right_front_value < safe_distance or left_front_value < safe_distance:
            angles = [15, 30, 45, 60, 75, 90, 180]
            random.shuffle(angles)
            for angle in angles:
                x = sign()
                rotate(x*angle)


            
        if side_left_value < safe_distance - 0.1:
            print('--->  LEFT WALL DETECTED ' + str(side_left_value) + ' [turning slight right...]')
            left_speed = max_speed
            right_speed = max_speed - .20
        elif side_right_value < safe_distance - 0.1:
            print('--->  RIGHT WALL DETECTED ' + str(side_right_value) + ' [turning slight left...]')
            left_speed = max_speed - .20
            right_speed = max_speed
        else:
            if left_speed > max_speed:
                print('TOO HIGH => setting velocity to 6.28')
                left_speed = max_speed
            elif left_speed < -max_speed:
                print('TOO LOW => setting velocity to -6.28')
                left_speed = -max_speed  
            else: 
                left_speed = left_speed
                right_speed = left_speed

        FRONT_SIDE_RIGHT_motor.setVelocity(right_speed)
        FRONT_SIDE_LEFT_motor.setVelocity(left_speed)
        
        back_right_motor.setVelocity(right_speed)
        back_left_motor.setVelocity(left_speed)
    

# Main loop:
while robot.step(timestep) != -1:
    forward()
    