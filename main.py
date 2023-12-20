import gymnasium as gym
import pandas as pd
from gymnasium import spaces

import pybullet as p
import pybullet_data
import numpy as np

from stable_baselines3 import PPO

import matplotlib.pyplot as plt

state = 'coll'
RESULTS = []
TRAIN_MODE = False
TEST_MODE = False  # not TRAIN_MODE


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
        rew = 0
        # print("****** STEP")
        # print(state)
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

        collision = p.getContactPoints(self.robot, self.box, 6)

        if state == "no coll":
            if len(collision) > 0:
                if collision[0][6] in RESULTS:
                    rew -= 1
                else:
                    rew = 5
                    state = "coll"
                    RESULTS.append(collision[0][6])

        if state == "coll":
            if len(collision) == 0:
                rew = 4
                state = "no coll"

        if (len(p.getContactPoints(self.robot, self.box, 1)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 2)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 3)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 4)) > 0) or (
                len(p.getContactPoints(self.robot, self.box, 5)) > 0):
            rew -= 3
            # print("COLLISION BAD")

        # Optionally add additional info
        info = {'distance': 7}
        observation = self._get_initial_observation

        return observation, rew, done, False, info

    def reset(self, **kwargs):
        # print("*******RESET")
        # Reset the environment to an initial state

        # Reset the robot to its initial position and orientation
        p.resetBasePositionAndOrientation(self.robot, self.initial_robot_pos, self.initial_robot_orn)
        for i, joint_state in enumerate(self.initial_joint_states):
            p.resetJointState(self.robot, i, joint_state)

        observation = self._get_initial_observation
        # print(observation)
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

        # Optionally get joint states (positions, velocities)
        joint_states = p.getJointStates(self.robot, range(6))
        joint_positions = [state[0] for state in joint_states]  # Just positions
        joint_positions = tuple(joint_positions)

        # Combine into a single observation array
        observation = np.array(end_effector_pos + joint_positions, dtype=np.float32)

        return observation


if TRAIN_MODE:

    # Create the environment
    env = XArmEnv()
    # print("next")
    env.step(np.array([0.75, 0.0, 0.0], dtype=np.float32))

    # Initialize the agent
    # model = PPO("MlpPolicy", env, learning_rate = 0.0005, verbose=1)
    model = PPO("MlpPolicy", env, verbose=1)
    # print("to learning")
    # Train the agent
    for iteration in range(50):
        print("Iteration ", iteration)
        model.learn(total_timesteps=1000)
        model.save("/Users/alannamanfredini/Documents/* Robot Learning/RL_mod_1")
    RESULTS_df = pd.DataFrame(np.array(RESULTS))
    RESULTS_df.columns = ['x', 'y', 'z']
    RESULTS_df.to_csv("/Users/alannamanfredini/Documents/* Robot Learning/Results.csv")
    env.close()

if TEST_MODE:
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
            # print("start")
            # print(np.array(obs))
            action, _states = model.predict(np.array(obs), deterministic=True)
            obs, reward, done, trunc, info = env.step(action)
            total_reward += reward

        print(f"Episode: {episode + 1}, Total Reward: {total_reward}")
    env.close()

data = pd.read_csv("/Users/alannamanfredini/Documents/* Robot Learning/Results.csv")


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(data['x'], data['y'], data['z'], 'o')
plt.show()
