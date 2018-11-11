#!/usr/bin/env python

import gym
from gym import error, spaces, utils
from gym.utils import seeding
import roslib # TODO: load manifest?
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class FoodEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, hz = 120):
        # initialize state & action
        # P_x, P_y, P_z, O_x, O_y, O_z
        self.state = np.zeros(6)
        # V_px, V_py, V_pz, V_ox, V_oy, V_oz
        self.action = np.zeros(6)

        # save t (from Hanjun)
        self.t = np.array([-7.792, -274.703, -9.899])/1000

        # initialize ros node
        rospy.init_node('rollout')

        # save refresh rate
        self.hz = hz

        # initialize timer
        timer = rospy.Timer(rospy.Duration.from_sec(1.0/float(self.hz)), self.callback)

        # initialize marker, leaving shaft & head at default for now
        base_frame = '/world'
        self.marker = Marker()
        self.marker.header.frame_id = base_frame
        self.marker.ns = 'Arrow'
        self.marker.id = 0
        self.marker.action = self.marker.ADD
        self.marker.type = self.marker.ARROW
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.points.extend((Point(0, 0, 0), Point(0, 0, 0)))

        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.04

        # initialize publisher
        self.pub = rospy.Publisher('forque_pose', Marker, queue_size=10)

        # initialize rate
        self.rate = rospy.Rate(self.hz)

        self.done = False

        self.reset()

    def reset(self):
        self.state, self.action = np.zeros(6), np.zeros(6)
        self.done = False
        return self.state


    def publish_marker(self):
        pose = self.state[:3]
        # build rotation matrix
        ox, oy, oz = self.state[3], self.state[4], self.state[5]
        sinox, cosox = math.sin(ox), math.cos(ox)
        sinoy, cosoy = math.sin(oy), math.cos(oy)
        sinoz, cosoz = math.sin(oz), math.cos(oz)
        R_x = np.array([[1, 0, 0],
                        [0, cosox, -sinox],
                        [0, sinox, cosox]])
        R_y = np.array([[cosoy, 0, sinoy],
                        [0, 1, 0],
                        [-sinoy, 0, cosoy]])
        R_z = np.array([[cosoz, -sinoz, 0],
                        [sinoz, cosoz, 0],
                        [0, 0, 1]])
        R = np.matmul(R_z, np.matmul(R_y,R_x))
        # update marker
        base = pose - np.matmul(R, self.t)
        self.marker.points[0] = Point(base[0], base[1], base[2])
        self.marker.points[1] = Point(pose[0], pose[1], pose[2])
        self.pub.publish(self.marker)


    def callback(self, event):
        self.state += (self.action / self.hz)
        self.publish_marker()
        if self.state[1] > 0.07: # when forque is more than 7 cm over food
            self.done = True


    def step(self, action): # new action, return state
        assert (action.shape[0] == 6), "Action not in R^6"
        self.action = action
        self.rate.sleep()
        return self.state, 0, self.done, None

    
    def render(self):
        pass
    
if __name__ == '__main__':
    rollout = Rollout()
