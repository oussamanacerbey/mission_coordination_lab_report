#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag
import time
import math

# Flag positions
flag_positions = {
    "robot_1": (-21.21320344, 21.21320344),# Flag 1 
    "robot_2": (21.21320344, 21.21320344), # Flag 2 
    "robot_3": (0, -30)   }           

class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        self.target_flag = flag_positions[robot_name]
        
        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        """
        :param kp
        :param ki
        :param kd
        :param output_limit
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error):
        """
        calculate output
        :param error
        :return: control output
        """
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time

        dt = current_time - self.last_time
        self.last_time = current_time

        # PID
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limit:
            output = max(self.output_limit[0], min(output, self.output_limit[1]))

        self.prev_error = error
        return output


def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")

    # Initialisiing the PID controller
    a_pid = PIDController(kp=0.5, ki=0.0, kd=0.05, output_limit=(-1.0, 1.0))  # the control angle 
    v_pid = PIDController(kp=0.5, ki=0.0, kd=0.05, output_limit=(0.0, 2.0))  #  the control velocity

    # we set some Obstacle avoidance parameters
    safe_d = 4     # Minimum safe distance to avoid obstacles
    avoid_V = 1.2     # we reduce the velocity when avoiding obstacles
    avoid_A = math.pi/2    # we set the angle of turning when avoiding obstacles
    flag_x, flag_y = robot.target_flag
    
    while not rospy.is_shutdown():
        # we use the sonar to get the distance to the closest obstacle
        sonar_dist =robot.get_sonar()

        if sonar_dist < safe_d:  # if an obstacle is detected do:
            
            print(f"{robot_name} detected obstacle at distance = {sonar_dist}")
            
            # we set the avoiding angle and velocity 
            velocity = avoid_V
            angle = avoid_A 
            robot.set_speed_angle(velocity, angle)
            rospy.sleep(0.3)
            continue 

        # Strategy 
        
        distance_to_flag = robot.getDistanceToFlag()
        # get robot's pose
        robot_x, robot_y, current_angle= robot.get_robot_pose()
        # the coordinates of the flag
        #flag_x = -21.21320344 
        #flag_y = 21.21320344  

        # Calculating dx and dy relative to the robot's position
        dx = flag_x-robot_x
        dy = flag_y-robot_y
        # calculate distance and target angle
        distance_to_flag = math.sqrt((dx)*2 + (dy)*2)  
        target_angle = math.atan2((dy), (dx)) 
        
        # calculating the angle error
        angle_error = target_angle - current_angle

        # calculating the velocity and angle using the pid
        velocity = v_pid.compute(distance_to_flag)  
        angle = a_pid.compute(angle_error)  

        print(f"{robot_name} x_cordinates flag_to_robot = {dx}")
        print(f"{robot_name} y_cordinates flag_to_robot = {dy}")
        print(f"{robot_name} distance to the flag = {distance_to_flag}")
        print(f"{robot_name} target angle = {math.degrees(target_angle)}")
        print(f"{robot_name} current angle = {math.degrees(current_angle)}")
        
        if abs(distance_to_flag) < 0.5:
            velocity = 0
            angle = 0
            robot.set_speed_angle(velocity, angle)
            print(f"{robot_name} goal reached !")
            break
        # DO NOT TOUCH.
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.1)
        
            


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()
