from controller import Robot
import random

# Initialize the robot
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize sensors
ps = []
ps_names = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

# Function to set motor speeds
def set_speed(left_speed, right_speed):
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# Function to check for obstacles
def check_obstacles():
    distances = [sensor.getValue() for sensor in ps]
    front_left = distances[7]
    front_right = distances[0]
    side_left = distances[5]
    side_right = distances[2]
    
    return front_left, front_right, side_left, side_right

# Main loop
while robot.step(timestep) != -1:
    front_left, front_right, side_left, side_right = check_obstacles()
    
    # Default speeds
    left_speed = 5.0
    right_speed = 5.0

    # Obstacle avoidance behavior
    if front_left > 80.0 or front_right > 80.0:
        # Random turn
        if random.choice([True, False]):
            left_speed = 5.0
            right_speed = -5.0
        else:
            left_speed = -5.0
            right_speed = 5.0
    elif side_left > 80.0:
        left_speed = 5.0
        right_speed = -5.0
    elif side_right > 80.0:
        left_speed = -5.0
        right_speed = 5.0

    set_speed(left_speed, right_speed)
