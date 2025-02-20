"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from enum import Enum, auto
import math

PI = math.pi
max_speed = 6.28
wheel_radius = 0.025
linear_velocity = wheel_radius * max_speed
distance_between_wheels = 0.156
rr = 1
rate_of_rotation = (rr * linear_velocity) / distance_between_wheels 

num = 0

class State(Enum):
    FORWARD = auto()
    ROTATE = auto()
    
class Turn(Enum):
    BOTTOM_RIGHT = auto()
    BOTTOM_LEFT = auto()
    BOTTOM = auto()
    
state = State.FORWARD
turn = None

def forward():
    return max_speed, max_speed
    
def shuffle_angles(a):
    random.shuffle(a)
    return a


if __name__ == "__main__":
    robot = Robot()
    
    # timestep = int(robot.getBasicTimeStep())
    timestep = 64
    # max_speed = 6.28
    
    # Created Motor Instances
    front_right_motor = robot.getMotor('front_motor_1')
    front_left_motor = robot.getMotor('front_motor_2')
   
    back_right_motor = robot.getMotor('back_motor_1')
    back_left_motor = robot.getMotor('back_motor_2')
   
   
    front_right_motor.setPosition(float('inf'))
    front_right_motor.setVelocity(0.0)
   
    front_left_motor.setPosition(float('inf'))
    front_left_motor.setVelocity(0.0)
   
   
    back_right_motor.setPosition(float('inf'))
    back_right_motor.setVelocity(0.0)
   
    back_left_motor.setPosition(float('inf'))
    back_left_motor.setVelocity(0.0)
    
    # Created DistanceSensor Instances
    front_distance_sensor = robot.getDevice("distance_sensor_front")
    front_distance_sensor.enable(timestep)
   
    side_right_distance_sensor = robot.getDevice("distance_sensor_side_right")
    side_right_distance_sensor.enable(timestep)
   
    side_left_distance_sensor = robot.getDevice("distance_sensor_side_left")
    side_left_distance_sensor.enable(timestep)
    
    
    
    angles = [PI/6, PI/4, PI/3, PI/2, PI]
    left_angles = [((3*PI)/2) - a for a in angles]
    right_angles = [(2*PI) - a for a in angles]
    
    def rotate(theta, start_time, speed_fraction=0.5):
        duration_turn = theta / rate_of_rotation
        rot_start_time = robot.getTime()
        rot_end_time = rot_start_time + duration_turn
        
        while robot.step(timestep) != -1:
            current_time = robot.getTime()
        
            arc_length = (theta/2) * distance_between_wheels
            
            wheel_speed = max_speed * speed_fraction  
            time_required = arc_length / wheel_speed
            # print(wheel_speed)
            
            if rot_start_time < current_time < rot_end_time:
                left_speed = wheel_speed if theta > 0 else -wheel_speed
                right_speed = -wheel_speed if theta > 0 else wheel_speed
            else:
                break
    
            return (left_speed, right_speed)

    start_time = robot.getTime()

    # Main loop:
    while robot.step(timestep) != -1:
    
        front_value = front_distance_sensor.getValue()
        side_right_value = side_right_distance_sensor.getValue()
        side_left_value = side_left_distance_sensor.getValue()
        
        print(f"(Front, Right, Left) -> ({front_value}, {side_right_value}, {side_left_value})")
        
        if front_value < 250 or side_left_value < 990:
            if num % 2 == 0:
                left_speed, right_speed = rotate(PI/2, start_time)
                # left_speed, right_speed = rotate(PI/2)
            else:
                left_speed, right_speed = rotate(PI/2, start_time)
                left_speed , right_speed = -left_speed, -right_speed 
                
        else:
            left_speed, right_speed = forward()

                
        front_right_motor.setVelocity(right_speed)
        front_left_motor.setVelocity(left_speed)
       
        back_right_motor.setVelocity(right_speed)
        back_left_motor.setVelocity(left_speed)
                
                
    

    