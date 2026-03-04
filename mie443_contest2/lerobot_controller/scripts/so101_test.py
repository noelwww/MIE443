from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
import time

robot_config = SO101FollowerConfig(
    port='/dev/ttyACM0',
    id='follower',
    use_degrees=True
)

robot = SO101Follower(robot_config)
robot.connect()

while True:
    print(robot.get_observation())
    time.sleep(0.5)