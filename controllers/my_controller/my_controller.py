"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from enum import Enum, auto
import math

PI = math.pi
# max_speed = 6.28

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
    
def rotate(theta, speed_fraction=0.5):
    arc_length = (theta/2) * distance_between_wheels
            
    wheel_speed = max_speed * speed_fraction  
    time_required = arc_length / wheel_speed
    # print(wheel_speed)
            
    # Set left and right wheel speeds (opposite direction for rotation)
    left_speed = wheel_speed if theta > 0 else -wheel_speed
    right_speed = -wheel_speed if theta > 0 else wheel_speed
    
    return (left_speed, right_speed)


if __name__ == "__main__":
    robot = Robot()
    
    # timestep = int(robot.getBasicTimeStep())
    timestep = 64
    max_speed = 6.28
    
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
    
    # def rotate(theta, speed_fraction=0.5, turn):
        # arc_length = (theta/2) * distance_between_wheels
        
        # wheel_speed = max_speed * speed_fraction  
        # time_required = arc_length / wheel_speed
        # print(wheel_speed)
        
        # Set left and right wheel speeds (opposite direction for rotation)
        # left_speed = wheel_speed if theta > 0 else -wheel_speed
        # right_speed = -wheel_speed if theta > 0 else wheel_speed
        
        # if turn == Turn.BOTTOM_LEFT:

        
        

        # return (left_speed, right_speed)

    i, j, k = 0, 0, 0
    i_prime = 0
    # Main loop:
    while robot.step(timestep) != -1:
    
        front_value = front_distance_sensor.getValue()
        side_right_value = side_right_distance_sensor.getValue()
        side_left_value = side_left_distance_sensor.getValue()
        
        print(f"(Front, Right, Left) -> ({front_value}, {side_right_value}, {side_left_value})")
        
        if state == State.FORWARD:
            print(f"state: FORWARD")
            left_speed, right_speed = forward()
            if front_value < 250 or side_right_value < 990 or side_left_value < 990:
                state = State.ROTATE
                turn = Turn.BOTTOM
                print(f"i: {i+1}")
                i += 1
            elif front_value < 990 and side_right_value < 990:
                state = State.ROTATE
                turn = Turn.BOTTOM_LEFT
                print(f"j: {j+1}")
                j += 1
            elif front_value < 990 and side_left_value < 990:
                state = State.ROTATE
                turn = Turn.BOTTOM_RIGHT
                print(f"k: {k+1}")
                k += 1
                
        if state == State.ROTATE:
            print(f"state: ROTATE")
            print(f"i_prime: {i_prime+1}")
            i_prime += 1
            

                
        front_right_motor.setVelocity(right_speed)
        front_left_motor.setVelocity(left_speed)
       
        back_right_motor.setVelocity(right_speed)
        back_left_motor.setVelocity(left_speed)
                
                
    

    