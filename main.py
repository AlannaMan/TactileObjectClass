import gymnasium as gym
from gymnasium import spaces

import pybullet as p
import pybullet_data
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3 import DQN

import sys
import time

state = 'coll'
RESULTS = []
TRAIN_MODE = False

class XArmEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # Set up the simulation environment
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Create Environment
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.0001)
        p.setRealTimeSimulation(0)

        p.performCollisionDetection()

        # Load the robot
        self.robot = p.loadURDF("xarm/xarm6_robot.urdf")
        initial_joint_positions = p.calculateInverseKinematics(self.robot, 6, np.array(
            [0.75, 0.0, 0.25]))  # Define your initial joint positions
        for i, joint_position in enumerate(initial_joint_positions):
            p.resetJointState(self.robot, i + 1, joint_position)
        # Store the initial joint states
        self.initial_joint_states = [p.getJointState(self.robot, i)[0] for i in range(6)]
        self.initial_robot_pos, self.initial_robot_orn = p.getBasePositionAndOrientation(self.robot)

        p.resetDebugVisualizerCamera(1, -30, -50, cameraTargetPosition=[0.5, 0, 0])

        # Load a box or define a target
        start_pos = np.array([0.75, -0.1, 0.25])
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        self.box = p.loadURDF("cube.urdf", start_pos, start_orientation, globalScaling=0.5, useFixedBase=True)
        self.plane = p.loadURDF("plane.urdf")

        # Define observation and action spaces
        self.action_space = spaces.Box(low=-0.77, high=0.77, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-10.0, high=10.0, shape=(9,), dtype=np.float32)

    def step(self, action):
        global state
        #print("****** STEP")
        #print(state)
        done = False
        """
        Parameters:
        - action: a list or array of floats representing the change in each joint angle

        Returns:
        - observation: the new state of the environment after taking the action
        - reward: the reward for taking the action
        - done: a boolean indicating whether the episode is finished
        - info: a dictionary with additional information for debugging
        """

        # Ensure action is within the action space
        assert self.action_space.contains(action), "Action is invalid"

        action_joint = p.calculateInverseKinematics(self.robot, 6, action)
        # Apply the action to the robot by setting new joint positions
        p.setJointMotorControlArray(self.robot,
                                    [1, 2, 3, 4, 5, 6], p.POSITION_CONTROL,
                                    targetPositions=action_joint)
        p.stepSimulation()
        """
        for i, joint_action in enumerate(action_joint):
            p.setJointMotorControl2(self.robot,
                                    jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=joint_action)


            p.setJointMotorControl2(self.robot,
                                    jointIndex=i,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=joint_action)
            """
        # while True:
        #    p.stepSimulation()

        # Get the new observation state
        end_effector_pos = p.getLinkState(self.robot, 6)[0]
        box_pos = p.getBasePositionAndOrientation(self.box)[0]

        collision = p.getContactPoints(self.robot, self.box, 6)

        if collision[0][6]

        if state == "no coll":
            if len(collision) > 0:
                reward = 1
                state = "coll"
                RESULTS.append(collision[0][6])
            else:
                reward = -np.linalg.norm(np.array(end_effector_pos) - np.array(box_pos))

        if state == "coll":
            if len(collision) == 0:
                reward = 1
                state = "no coll"
            else:
                reward = np.linalg.norm(np.array(end_effector_pos) - np.array(box_pos))

        if len(RESULTS) > 0:
            curr_loc = np.array(p.getLinkState(self.robot, 6)[4])
            dist_from_desi_des_raw = np.linalg.norm(np.array(RESULTS[-1]) - np.array(curr_loc))
            reward -= dist_from_desi_des_raw * 10
            #print("RES")

        if (len(p.getContactPoints(self.robot, self.box, 1)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 2)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 3)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 4)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 5)) > 0):
            reward -= 5
            #print("COLLISION BAD")

        # Optionally add additional info
        info = {'distance': 7}
        observation = self._get_initial_observation

        return observation, reward, done, False, info

    def reset(self, **kwargs):
        #print("*******RESET")
        # Reset the environment to an initial state

        # Reset the robot to its initial position and orientation
        p.resetBasePositionAndOrientation(self.robot, self.initial_robot_pos, self.initial_robot_orn)

        # Reset the joint states
        for i, joint_state in enumerate(self.initial_joint_states):
            p.resetJointState(self.robot, i, joint_state)

        """
        p.resetSimulation()

        # Create Environment
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.0001)
        p.setRealTimeSimulation(0)

        p.performCollisionDetection()

        # Load the robot
        self.robot = p.loadURDF("xarm/xarm6_robot.urdf")

        # Load a box or define a target
        start_pos = np.array([0.75, -0.1, 0.25])
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        self.box = p.loadURDF("cube.urdf", start_pos, start_orientation, globalScaling=0.5, useFixedBase=True)
        self.plane = p.loadURDF("plane.urdf")

        # Define observation and action spaces
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-10.0, high=10.0, shape=(12,), dtype=np.float32)
        """
        observation = self._get_initial_observation
        #print(observation)
        return observation, {}  # return the information which is relevant for the robot making the next step

    def render(self, mode='human'):
        # Environment rendering
        pass

    def close(self):
        p.disconnect()

    @property
    def _get_initial_observation(self):
        # Get end-effector position and orientation
        end_effector_state = p.getLinkState(self.robot, 6)
        end_effector_pos = tuple(end_effector_state[0])  # (x, y, z) position
        end_effector_orn = tuple(end_effector_state[1])  # orientation (quaternion or Euler)

        # Get position of the box
        box_pos = p.getBasePositionAndOrientation(self.box)[0]

        # Optionally get joint states (positions, velocities)
        joint_states = p.getJointStates(self.robot, range(6))
        joint_positions = [state[0] for state in joint_states]  # Just positions
        joint_positions = tuple(joint_positions)

        # Combine into a single observation array
        # observation = np.array(end_effector_pos + end_effector_orn + box_pos + joint_positions, dtype=np.float32)
        observation = np.array(end_effector_pos + joint_positions, dtype=np.float32)

        return observation


if TRAIN_MODE:


    # Create the environment
    env = XArmEnv()
    #print("next")
    env.step(np.array([0.75, 0.0, 0.0], dtype=np.float32))

    # Initialize the agent
    # model = PPO("MlpPolicy", env, learning_rate = 0.0005, verbose=1)
    model = PPO("MlpPolicy", env, verbose=1)
    #print("to learning")
    # Train the agent
    for iter in range(100):
        print("Iteration ", iter)
        model.learn(total_timesteps=1000)
    model.save("/Users/alannamanfredini/Documents/* Robot Learning/RL_mod_diff_spot")



# Load the trained model
model = PPO.load("/Users/alannamanfredini/Documents/* Robot Learning/RL_mod_diff_spot")

env = XArmEnv()

# Evaluate the model
total_episodes = 10
for episode in range(total_episodes):
    obs = env.reset()[0]
    done = False
    total_reward = 0

    while not done:
        #print("start")
        #print(np.array(obs))
        action, _states = model.predict(np.array(obs), deterministic=True)
        obs, reward, done, trunc, info = env.step(action)
        total_reward += reward

    print(f"Episode: {episode + 1}, Total Reward: {total_reward}")

env.close()


