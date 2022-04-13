#coding=UTF-8
import sys
from time import sleep
import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
    #等待目标点
    def wait_for_goal(self):
        raw_goal_data = None
        print('waiting goal...')
        while raw_goal_data is None:
            try:
                raw_goal_data = rospy.wait_for_message('move_base_simple/goal',PoseStamped, timeout=100)
            except:
                pass
        self.goal_x = raw_goal_data.pose.position.x
        self.goal_y = raw_goal_data.pose.position.y
        return self.goal_x, self.goal_y
    #计算目标距离
    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        return goal_distance
    #获得里程计并计算
    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)
        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi
        elif heading < -pi:
            heading += 2 * pi
        self.heading = round(heading, 2)
    #获取激光雷达数据
    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.1
        done = False
        print(len(scan.ranges))
        for i in range(0, len(scan.ranges), 23):
            if scan.ranges[i] == float('Inf')or scan.ranges[i] ==0 :#or scan.ranges[i] == 0:
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
        obstacle_min_range = round(min(scan_range), 2)
        self.obstacle_min_range =obstacle_min_range
        obstacle_angle = np.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = True
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True
        return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done
    #设置奖励
    def setReward(self, state, done, action):
        yaw_reward = []
        obstacle_min_range = state[-2]
        current_distance = state[-3]
        heading = state[-4]
        for i in range(5):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)
        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) 
        if done:
            rospy.loginfo("Collision!!")
            reward = -500
            self.pub_cmd_vel.publish(Twist())
            done =False
        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 1000
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.wait_for_goal()
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False
        return reward
    #设置动作
    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass
        state, done = self.getState(data)
        reward = self.setReward(state, done, action)
        return np.asarray(state), reward, done
    #重置目标点，并获取激光雷达数据
    def reset(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass
        if self.initGoal:
            self.goal_x, self.goal_y = self.wait_for_goal()
            self.initGoal = False
        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)
        return np.asarray(state)