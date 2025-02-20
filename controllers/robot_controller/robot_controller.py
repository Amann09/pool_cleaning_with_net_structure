"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    # timestep = int(robot.getBasicTimeStep())
    timestep = 64
    max_speed = 6.2831  # angular_velocity
    
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
    num_side = 6
    length_side = 0.5  # 0.25
    rr = 1
    
    wheel_radius = 0.025
    linear_velocity = wheel_radius * max_speed
    
    duration_side = length_side / linear_velocity
    
    start_time = robot.getTime()
    
    angles = [0.5236, 0.7854, 1.0472, 1.309, 1.5708, 3.1416]
    # angles = [ x for i in range(180+1)]
    
    
    angle_of_rotation = 6.28/num_side
    distance_between_wheels = 0.156
    
    rate_of_rotation = (rr * linear_velocity) / distance_between_wheels 
    duration_turn = angle_of_rotation / rate_of_rotation 
    
    rot_start_time = start_time + duration_side
    rot_end_time = rot_start_time + duration_turn
   
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    
        current_time = robot.getTime()
    
        left_speed = max_speed
        right_speed = max_speed
        
        if rot_start_time < current_time < rot_end_time:
            left_speed = -max_speed
            right_speed = max_speed
            
        elif current_time > rot_end_time:
            rot_start_time = current_time + duration_side
            rot_end_time = rot_start_time + duration_turn
        
        
        front_right_motor.setVelocity(right_speed)
        front_left_motor.setVelocity(left_speed)
        
        back_right_motor.setVelocity(right_speed)
        back_left_motor.setVelocity(left_speed)
        
        
        
    # Enter here exit cleanup code.
