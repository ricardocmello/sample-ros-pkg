#!/usr/bin/env python2.7
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose as TurtlePose
from std_msgs.msg import Float32

from math import cos, pi, sin, tanh, radians

class TurtleControl():
    def __init__(self):
        self.rospy = rospy
        self.rospy.init_node("TurtleControl", anonymous = True)
        self.rospy.loginfo("Starting TurtleControl")
        
        self.init_parameters()
        self.init_variables()

        self.initSubscribers()
        self.initPublishers()

        while self.pose_count == 0:
            pass
        self.main()

    def init_parameters(self):
        self.debug = True

        self.pose_topic = self.rospy.get_param("~pose_topic", "/turtle1/pose")
        self.twist_topic = self.rospy.get_param("~twist_topic", "/turtle1/cmd_vel")
        self.control_rate = self.rospy.get_param("~rate", 100)
        self.goal_tolerances = {"xy": self.rospy.get_param("~goal_tolerances/xy", 0.3),
                                "theta": self.rospy.get_param("~goal_tolerances/theta", 0.45)}
        self.period = self.rospy.get_param("~trajectory_period", 10.0) #Leminiscate trajectory period
        self.last_trajectory_x, self.last_trajectory_y = -0.1, -0.1
        self.started = False

        self.i_x, self.i_y = 1, 1
        self.k_x, self.k_y = 0.5, 0.5
        self.l_x, self.l_y = 1, 1
        self.a = 0.2
        return

    def initSubscribers(self):
        self.sub_pose = self.rospy.Subscriber(self.pose_topic, TurtlePose, self.pose_callback)
        return

    def initPublishers(self):
        self.pub_goal = self.rospy.Publisher(self.twist_topic, Twist, queue_size = 5)
        #if self.debug:
        self.pub_debug_trajectory = self.rospy.Publisher("debug_trajectory", PoseStamped, queue_size = 5)
        self.pub_debug_pose = self.rospy.Publisher("debug_pose", PoseStamped, queue_size = 5)
        self.pub_debug_error = self.rospy.Publisher("debug_error", Float32, queue_size = 5)
        return

    def init_variables(self):
        self.pose_count = 0
        self.current_pose = TurtlePose()
        self.rate = self.rospy.Rate(self.control_rate)
        return

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_count += 1
        #if self.debug:
        debug_pose = PoseStamped()
        debug_pose.header.stamp = rospy.Time.now()
        debug_pose.header.frame_id = "odom"
        debug_pose.pose.position.x = self.current_pose.x
        debug_pose.pose.position.y = self.current_pose.y
        self.pub_debug_pose.publish(debug_pose)
        return

    def get_current_goal(self, t):
        # Considering a leminiscate trajectory
        self.trajectory_x = 5.54 + (-2 * cos(2 * pi* t / self.period) / (sin(2 * pi * t / self.period) ** 2 + 1))
        self.trajectory_y = 5.54 + (-2 * sin(2 * pi* t / self.period) * cos(2 * pi* t / self.period) / (sin(2 * pi * t / self.period) ** 2 + 1))
        #if self.debug:
        debug_traj = PoseStamped()
        debug_traj.header.stamp = rospy.Time.now()
        debug_traj.header.frame_id = "goal"
        debug_traj.pose.position.x = self.trajectory_x
        debug_traj.pose.position.y = self.trajectory_y
        self.pub_debug_trajectory.publish(debug_traj)
        return 

    def calculate_position_error(self):
        self.error_x = self.trajectory_x - self.current_pose.x
        self.error_y = self.trajectory_y - self.current_pose.y
        error = Float32()
        error.data = math.sqrt(self.error_x**2+self.error_y**2)
        self.pub_debug_error.publish(error)
        return 

    def controller(self):
        if not self.started:
            self.time_offset = rospy.Time.now()
            current_time = 0.0
            self.last_time = -0.1
            self.started = True
        else:
            current_time = (rospy.Time.now() - self.time_offset).to_sec()
        
        self.get_current_goal(current_time)
        self.calculate_position_error()

        delta_time = current_time - self.last_time
        delta_trajectory_x = (self.trajectory_x - self.last_trajectory_x) / delta_time
        delta_trajectory_y = (self.trajectory_y - self.last_trajectory_y) / delta_time 

        control_law_x = delta_trajectory_x + self.i_x * tanh(self.error_x * (self.k_x / self.l_x))
        control_law_y = delta_trajectory_y + self.i_y * tanh(self.error_y * (self.k_y / self.l_y))

        #self.lin_vel_ref = cos(radians(self.current_pose.theta)) * control_law_x + sin(radians(self.current_pose.theta))* control_law_y
        #self.ang_vel_ref = (-1/self.a)*sin(radians(self.current_pose.theta)) * control_law_x + (1/self.a)*cos(radians(self.current_pose.theta)) * control_law_y
        self.lin_vel_ref = cos(self.current_pose.theta) * control_law_x + sin(self.current_pose.theta)* control_law_y
        self.ang_vel_ref = (-1/self.a)*sin(self.current_pose.theta) * control_law_x + (1/self.a)*cos(self.current_pose.theta) * control_law_y
        
        self.last_trajectory_x, self.last_trajectory_y = self.trajectory_x, self.trajectory_y
        self.last_time = current_time
        return

    def pub_twist(self):
        msg = Twist()
        msg.linear.x = self.lin_vel_ref
        msg.angular.z = self.ang_vel_ref
        self.pub_goal.publish(msg)


    def main(self):
        while not rospy.is_shutdown():
            self.controller()
            self.pub_twist()
            self.rate.sleep()
        
if __name__ == '__main__':
    try:
        pg = TurtleControl()
    except rospy.ROSInterruptException:
        pass
