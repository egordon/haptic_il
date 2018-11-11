#!/usr/bin/env python

import gym
import gym_food
import numpy as np
import rospy

if __name__ == '__main__':
    env = gym.make('FoodManip-v0')
    env.reset()

    episode_count = 100
    done = False
    action = np.array([0, 0.035, 0, 0.1, 0, 0])

    while not done:
        ob, _, done, _ = env.step(action)
        done = done or rospy.is_shutdown()

    print("Done!")