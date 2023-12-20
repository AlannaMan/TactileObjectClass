#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 23 16:23:55 2023

@author: alannamanfredini
"""

# %%

import pybullet as pb
import os
import pybullet_data
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

import collections


VEL = np.ones(6) * 0.0001
RESULT = []
RESULT_norm = []

z = 0
y = 0

# create a deque to hold the most recent 5 positions. This allows me to return to a previous position if it gets stuck
DEQUE = collections.deque(maxlen=1000)

# FUTURE WORK    add finger to end of robot


# %% Movement Function

def collis_method():
    # Check for collision
    collision = pb.getContactPoints(xarm, boxId, 6)

    if len(collision) > 0:
        print("collis")

        # Add the collision results to array
        RESULT.append(collision[0][6])
        RESULT_norm.append(collision[0][7])

        # If it is the first collision, change the global coords to collision location...
        # acts as baseline loc of object for future mvt
        if len(RESULT) == 1:
            global x
            global y
            global z

            x = RESULT[-1][0]
            y = RESULT[-1][1]
            z = RESULT[-1][2]

        # Get norm of collision
        collis_norm = np.array(RESULT_norm[-1])

        # Find link cartesian coord of collision
        # to minimise drift, set the z value to the world z value for next mvt
        centroid_loc_during_collision = np.array(pb.getLinkState(xarm, 6)[4])
        centroid_loc_during_collision[2] = z

        # Use the object norm as a guideline to back away from object 0.1units
        moving(centroid_loc_during_collision + 0.3 * collis_norm)
        print("backed up")

        return collision  # centroid_loc_during_collision, collis_norm
    return collision


def moving(goal_loc):
    print("moving")
    print(goal_loc)
    goal_joint = pb.calculateInverseKinematics(xarm, 6, goal_loc)

    # move robot towards target - FUTURE WORK, target needs to be relevant during traverse
    pb.setJointMotorControlArray(xarm, \
                                 [1, 2, 3, 4, 5, 6], pb.POSITION_CONTROL, \
                                 targetPositions=goal_joint)  # , targetVelocities = VEL

    prev_loc = np.array([np.infty, np.infty, np.infty])
    while (True):
        # print("stepping")
        # for i in range(2):
        # print("inside")

        # print(goal_loc)

        for i in range(5):
            pb.stepSimulation()
            DEQUE.append(pb.getJointStates(xarm, [1, 2, 3, 4, 5, 6]))

        curr_loc = np.array(pb.getLinkState(xarm, 6)[4])

        dist_from_desi_des_raw = goal_loc - curr_loc
        dist_from_desi_des = np.dot(dist_from_desi_des_raw, dist_from_desi_des_raw) ** 0.5
        # print(dist_from_desi_des)
        # print(curr_joint())
        # print(pb.getLinkState(xarm, 6)[4])

        change_in_loc_raw = prev_loc - curr_loc
        change_in_loc = np.dot(change_in_loc_raw, change_in_loc_raw) ** 0.1
        # print(change_in_loc)

        prev_loc = np.array(pb.getLinkState(xarm, 6)[4])
        # REASONS TO STOP MOVING

        # checks if made contact, adds to RESULT and backs up
        collision = collis_method()
        if len(collision) > 0:
            return 'collide'

        # reached desired location
        if dist_from_desi_des < 0.1: print("mvm"); return 'movement complete'

        # extended arm too much or cannot move
        if change_in_loc < 0.1: print("no change");  return 'no change'
        print(change_in_loc)


# %% States


class State:  # (object)
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        self.TARGET_POS = [0.5, 0, 1.5]
        self.TARGET_POS_JOINT = pb.calculateInverseKinematics(xarm, 5, self.TARGET_POS)
        self.curr_norm = 0

        pb.performCollisionDetection()
        print('Initialising state:', str(self))

    def on_event(self, event):
        print("state on ev")
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        print("state repr")
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        print("state str")
        """
        Returns the name of the State.
        """
        return self.__class__.__name__


class front_left_trav(State):
    def on_event(self):
        print('front L on')

        global x
        global y
        global z

        position = 'first'
        counter = 0
        while True:
            print("move left")
            # Move left set amount and at set height
            y += 0.1

            move_left = np.array(pb.getLinkState(xarm, 6)[4])
            move_left[1] = y
            move_left[2] = z

            # Move left
            position = moving(move_left)

            if position == 'no change': return unstick().on_event('front_left')

            # if position == 'no change' : up().on_event(pb.getLinkState(xarm, 6)[4], 'front_left'); break

            print("move towards obj")
            # move forward in norm direction - slightly more than it backed up
            pos_goal = np.array(pb.getLinkState(xarm, 6)[4]) - np.array(RESULT_norm[-1]) * 0.35
            pos_goal[2] = z

            # move until collision and backup
            position = moving(pos_goal)

            if position == 'movement complete': return up().on_event(move_left, 'front_left')
            if position == 'no change': return unstick().on_event('front_left')


class unstick():
    def on_event(self, prev):
        print("unsticking")
        for items in range(len(DEQUE)):
            joint_states = DEQUE[-1 - items]
            pos_rob = []
            for values in joint_states:
                pos_rob = np.concatenate((pos_rob, [values[0]]))
            pb.setJointMotorControlArray(xarm, \
                                         [1, 2, 3, 4, 5, 6], pb.POSITION_CONTROL, \
                                         targetPositions=pos_rob)
            for i in range(10):
                pb.stepSimulation()
        if prev == 'front_left': front_right_trav().on_event()
        if prev == 'front_right': front_left_trav().on_event()


class up(State):
    def on_event(self, backup_goal, prev_state):
        print('up')

        global z
        z += 0.1

        # Get clear of object by returning to previous position
        position = moving(backup_goal)

        up_align_goal = np.array(RESULT[-1])
        up_align_goal[0] = pb.getLinkState(xarm, 6)[4][0]
        up_align_goal[2] = z

        position = moving(up_align_goal)

        collide_goal = np.array(RESULT[-1])
        collide_goal[2] = z
        position = moving(collide_goal)

        if prev_state == 'front_left': front_right_trav().on_event()
        if prev_state == 'front_right': front_left_trav().on_event()


class front_right_trav(State):

    def on_event(self):
        print('front R on')

        global x
        global y
        global z

        position = 'first'
        counter = 0
        while True:
            print("move right")
            # Move left set amount and at set height
            y -= 0.1

            move_left = np.array(pb.getLinkState(xarm, 6)[4])
            move_left[1] = y
            move_left[2] = z

            # Move left
            position = moving(move_left)

            if position == 'no change': return unstick().on_event('front_right')

            # if position == 'no change' : up().on_event(pb.getLinkState(xarm, 6)[4], 'front_left'); break

            print("move towards obj")
            # move forward in norm direction - slightly more than it backed up
            pos_goal = np.array(pb.getLinkState(xarm, 6)[4]) - np.array(RESULT_norm[-1]) * 0.35
            pos_goal[2] = z

            # move until collision and backup
            position = moving(pos_goal)

            if position == 'movement complete': return up().on_event(move_left, 'front_right')
            if position == 'no change': return unstick().on_event('front_right')


class turning(State):
    def on_event(self, pos_goal):
        # Reset Robot Positions
        for i in range(1, 7):
            pb.resetJointState(xarm, i, 0.0)

        print("Searching")
        # Create initial position
        # chosen to be close to base but to have link 6 protruding from rest of robot
        des_pos = np.array(pb.calculateInverseKinematics(xarm, 6, \
                                                         (0.2, 0, 0.1)))

        des_pos[1] = 0.6
        des_pos[2] = -0.3
        des_pos[-2] = -1.0

        pb.setJointMotorControlArray(xarm, \
                                     [1, 2, 3, 4, 5, 6], pb.POSITION_CONTROL, \
                                     targetPositions=des_pos)

        # Move to iniitial position
        for i in range(100):
            pb.stepSimulation()
            if len(collis_method()) > 0: front_left_trav().on_event()

        # Record initial start positions
        base_pos = curr_joint()[0]

        base_joint_min = 0  # pb.getJointInfo(xarm, 1)[8] = -6.28
        base_joint_max = pb.getJointInfo(xarm, 1)[9]

        # Set initial movement to CCW
        mvt = 'left'

        # Start searching
        while True:
            # Ensure desired loc is withing joint range of base
            while (base_pos < base_joint_max) and (base_pos > base_joint_min):
                # Make the base link update to move in a circle
                move_to = curr_joint()
                move_to[0] = base_pos

                pb.setJointMotorControlArray(xarm, \
                                             [1, 2, 3, 4, 5, 6], pb.POSITION_CONTROL, \
                                             targetPositions=move_to)

                for i in range(10):
                    pb.stepSimulation()
                    if len(collis_method()) > 0:
                        if mvt == 'left': return front_left_trav().on_event()
                        if mvt == 'right': return front_right_trav().on_event()

                # if joint range is exceeded swap direction
                if mvt == 'left': base_pos += 0.05
                if mvt == 'right': base_pos -= 0.05
            # update value and movement direction for next pass
            if base_pos > base_joint_max: mvt = 'right'; base_pos -= 0.05
            if base_pos < base_joint_min: mvt = 'left'; base_pos += 0.05

            # move the next traverse in a wider radius circle (and lower to ground)
            curr_pos_5 = np.asarray(pb.getLinkState(xarm, 6)[4])

            curr_pos_5[0] += 0.1  # x
            curr_pos_5[1] += 0.1  # y
            if curr_pos_5[2] > 0.1:
                curr_pos_5[2] -= 0.05  # z

            col = moving(curr_pos_5)
            # If there is a collision moving forward, go into main program
            if col == 'collide': return front_left_trav().on_event()


class SimpleDevice(object):
    print("simple device")
    """ 
    A simple state machine that mimics the functionality of a device from a 
    high level.
    """

    def __init__(self):
        print("simp dev init")
        """ Initialize the components. """

        # Start with a default state.
        self.state = turning()

    def on_event(self, event):
        print("simp ev on")
        """
        This is the bread and butter of the state machine. Incoming events are
        delegated to the given states which then handle the event. The result is
        then assigned as the new state.
        """

        # The next state will be the result of the on_event function.
        self.state = self.state.on_event([0.7, 0, 0.1])


# %%
unstick().on_event('hi')

# %%
print(DEQUE)

# %%
test = np.array([1, 2, 3])
tes = np.concatenate((test[:2], [5]))
print(tes)

# %%%
print(RESULT)
df = pd.DataFrame(RESULT)
print(df[:, 0])

# %% Creating joint boundaries

joint = [1, 2, 3, 4, 5, 6]
joint_min = []
joint_max = []

for values in joint:
    joint_min = np.concatenate((joint_min, [pb.getJointInfo(xarm, values)[8]]))
    joint_max = np.concatenate((joint_max, [pb.getJointInfo(xarm, values)[9]]))

print(joint_min)
print(pb.getLinkState(xarm, 6))
print(pb.getLinkState(xarm, 5))
print(pb.getJointState(xarm, 6))

# %%

VEL = np.ones(6) * 0.0001
RESULT = []
RESULT_norm = []

z = 0
y = 0

# create a deque to hold the most recent 5 positions. This allows me to return to a previous position if it gets stuck
DEQUE = collections.deque(maxlen=1000)

device = SimpleDevice()

device.on_event("hi")

# %% Variables -- copied in reset

VEL = np.ones(6) * 0.0001
RESULT = []

z = 0

# create a deque to hold the most recent 5 positions. This allows me to return to a previous position if it gets stuck
DEQUE = collections.deque(maxlen=20)

# %% Connect

pb.connect(pb.GUI)

# %% Reset simulation

pb.resetSimulation()

# %% Importing data

pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# %% Import plane

plane = pb.loadURDF("plane.urdf")

# %% Robot import

flags = pb.URDF_INITIALIZE_SAT_FEATURES
useFixedBase = True

xarm = pb.loadURDF("xarm/xarm6_robot.urdf", flags=flags, useFixedBase=useFixedBase)

# %% Make environment

pb.setGravity(0, 0, -9.81)
pb.setTimeStep(0.0001)
pb.setRealTimeSimulation(0)

# %% Camera setup

pb.resetDebugVisualizerCamera(1, -30, -50, cameraTargetPosition=[0.5, 0, 0])

# %% Import a cube

startPos = np.array([0.75, -0.1, 0.25])
startOrientation = pb.getQuaternionFromEuler([0, 0, 0])
boxId = pb.loadURDF("cube.urdf", startPos, startOrientation, globalScaling=0.5, useFixedBase=True)


# %% Find curr positions of joints

def curr_joint():
    joint_states = pb.getJointStates(xarm, [1, 2, 3, 4, 5, 6])
    pos_rob = np.array([])
    for values in joint_states:
        pos_rob = np.concatenate((pos_rob, [values[0]]))
    return pos_rob


# %% Plot points
df = pd.DataFrame(RESULT)
print(df)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter(df[0], df[1], df[2])

# %% Reset original robot conditionss


for i in range(1, 7):
    pb.resetJointState(xarm, i, 0.0)






