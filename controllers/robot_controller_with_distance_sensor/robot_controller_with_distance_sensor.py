"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor
import random
import math
import time

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    PI = math.pi
   
    # get the time step of the current world.
    # timestep = int(robot.getBasicTimeStep())
    timestep = 64
    max_speed = 6.2831  # angular_velocity
    
    left_speed = max_speed
    right_speed = max_speed
   
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
   
    # Driving the Robot in a Polygon Shape
    num_side = 4
    length_side = 0.5  # 0.25
    # rr = 1
   
    # wheel_radius = 0.025
    # linear_velocity = wheel_radius * max_speed
   
    # duration_side = length_side / linear_velocity
   
    # start_time = robot.getTime()
   
    # angle_of_rotation = 6.28/num_side
    distance_between_wheels = 0.156
    # rate_of_rotation = (rr * linear_velocity) / distance_between_wheels
    # duration_turn = angle_of_rotation / rate_of_rotation
   
    # rot_start_time = start_time + duration_side
    # rot_end_time = rot_start_time + duration_turn
   
    front_distance_sensor = robot.getDevice("distance_sensor_front")
    front_distance_sensor.enable(timestep)
   
   
    side_right_distance_sensor = robot.getDevice("distance_sensor_side_right")
    side_right_distance_sensor.enable(timestep)
   
   
    side_left_distance_sensor = robot.getDevice("distance_sensor_side_left")
    side_left_distance_sensor.enable(timestep)
   
    def pos_or_neg():
        sign = random.randint(0, 1)
        if sign == 0:
            return -1
        else:
            return +1
            
    angles = [PI/6, PI/4, PI/3, PI/2, PI]
   
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
        
    def forward():
        return (max_speed, max_speed)
    
    
  
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:

        front_value = front_distance_sensor.getValue()
        print("Front value is: ", front_value)
       
        side_right_value = side_right_distance_sensor.getValue()
        print("Side Right value is: ", side_right_value)
       
        side_left_value = side_left_distance_sensor.getValue()
        print("Side Left Value is:", side_left_value)
       
       
       # x = round(random.uniform(0.4, 0.9), 2)
       
        if (front_value < 199) and ((side_right_value > 990) or (side_left_value > 990)):
            # turn to any random angle and move forward
            working_angles = shuffle_angles(angles)
            print(f"Angle: {angles[0]}")
            # sign = pos_or_neg()
            left_speed, right_speed = rotate(working_angles[0], 1)
            
        elif (450 < front_value < 990) and (450 < side_right_value < 900):
            # turn to random angle of left side and move forward
            pass
            
        elif (450 < front_value < 990) and (450 < side_left_value < 900):
            # turn to random angle of right side and move forward
            pass
            
        elif (front_value < 450) and (side_right_value < 450):
            # turn to random angle of left side and move forward (quickly)
            pass
            
        elif (front_value < 450) and (side_left_value < 450):
            # turn to random angle of right side and move forward (quickly)
            pass
            
        elif (front_value > 990) and ((450 < side_right_value <= 1000) or (450 < side_left_value <= 1000)):
            left_speed, right_speed = forward()
            
        elif (side_right_value < 450 ):
            # turn to random angle of left side and move 
            pass
        elif (side_left_value < 450 ):
            # turn to random angle of right side and move forward
            pass 
        else:
            pass
       
        front_right_motor.setVelocity(right_speed)
        front_left_motor.setVelocity(left_speed)
       
        back_right_motor.setVelocity(right_speed)
        back_left_motor.setVelocity(left_speed)
           
       

   # elif (450 < right_1_value < 900) or (450 < right_2_value < 900) or (450 < left_1_value < 900) or (450 < left_2_value < 900):
            # left_speed = max_speed
            # right_speed = max_speed
        # elif (right_1_value < 450 ) or (right_2_value < 450) or (left_1_value < 450 ) or (left_2_value < 450):
            # x = round(random.uniform(0.4, 0.9), 2)
            # sign = pos_or_neg()
            # sign = 1
            # left_speed = (-1 * sign) * x * max_speed
            # right_speed = (+1 * sign) * x * max_speed

       
    # Enter here exit cleanup code.
