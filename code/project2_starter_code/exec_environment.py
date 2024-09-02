"""
    This code communicates with the coppeliaSim software and simulates shaking a container to mix objects of different color

    Install dependencies:
    https://www.coppeliarobotics.com/helpFiles/en/zmqRemoteApiOverview.htm

    MacOS: coppeliaSim.app/Contents/MacOS/coppeliaSim -GzmqRemoteApi.rpcPort=23004 ~/path/to/file/mix_Intro_to_AI.ttt
    Ubuntu: ./coppeliaSim.sh -GzmqRemoteApi.rpcPort=23004 ~/path/to/file/mix_Intro_to_AI.ttt
"""

import sys
import os
from itertools import product

# Change to the path of your ZMQ python API
sys.path.append("/app/zmq/")
import numpy as np
from zmqRemoteApi import RemoteAPIClient
import time
import random


class Simulation:
    def __init__(self, sim_port=23000):
        self.sim_port = sim_port
        self.directions = ["Up", "Down", "Left", "Right"]
        self.initializeSim()

    def initializeSim(self):
        self.client = RemoteAPIClient("localhost", port=self.sim_port)
        self.client.setStepping(True)
        self.sim = self.client.getObject("sim")

        # When simulation is not running, ZMQ message handling could be a bit
        # slow, since the idle loop runs at 8 Hz by default. So let's make
        # sure that the idle loop runs at full speed for this program:
        self.defaultIdleFps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)

        self.getObjectHandles()
        self.sim.startSimulation()
        self.dropObjects()
        self.getObjectsInBoxHandles()

    def getObjectHandles(self):
        self.tableHandle = self.sim.getObject("/Table")
        self.boxHandle = self.sim.getObject("/Table/Box")

    def dropObjects(self):
        self.blocks = 18
        frictionCube = 0.06
        frictionCup = 0.8
        blockLength = 0.016
        massOfBlock = 14.375e-03

        self.scriptHandle = self.sim.getScript(
            self.sim.scripttype_childscript, self.tableHandle
        )
        self.client.step()
        retInts, retFloats, retStrings = self.sim.callScriptFunction(
            "setNumberOfBlocks",
            self.scriptHandle,
            [self.blocks],
            [massOfBlock, blockLength, frictionCube, frictionCup],
            ["cylinder"],
        )

        print("Wait until blocks finish dropping")
        while True:
            self.client.step()
            signalValue = self.sim.getFloatSignal("toPython")
            if signalValue == 99:
                loop = 20
                while loop > 0:
                    self.client.step()
                    loop -= 1
                break

    def getObjectsInBoxHandles(self):
        self.object_shapes_handles = []
        self.obj_type = "Cylinder"
        for obj_idx in range(self.blocks):
            obj_handle = self.sim.getObjectHandle(f"{self.obj_type}{obj_idx}")
            self.object_shapes_handles.append(obj_handle)

    def getObjectsPositions(self):
        pos_step = []
        box_position = self.sim.getObjectPosition(self.boxHandle, self.sim.handle_world)
        for obj_handle in self.object_shapes_handles:
            # get the starting position of source
            obj_position = self.sim.getObjectPosition(obj_handle, self.sim.handle_world)
            obj_position = np.array(obj_position) - np.array(box_position)
            pos_step.append(list(obj_position[:2]))
        return pos_step

    def action(self, direction=None):
        if direction not in self.directions:
            print(
                f"Direction: {direction} invalid, please choose one from {self.directions}"
            )
            return
        box_position = self.sim.getObjectPosition(self.boxHandle, self.sim.handle_world)
        _box_position = box_position
        span = 0.02
        steps = 5
        if direction == "Up":
            idx = 1
            dirs = [1, -1]
        elif direction == "Down":
            idx = 1
            dirs = [-1, 1]
        elif direction == "Right":
            idx = 0
            dirs = [1, -1]
        elif direction == "Left":
            idx = 0
            dirs = [-1, 1]

        for _dir in dirs:
            for _ in range(steps):
                _box_position[idx] += _dir * span / steps
                self.sim.setObjectPosition(
                    self.boxHandle, self.sim.handle_world, _box_position
                )
                self.stepSim()

    def chooseAction(self):
        return np.random.choice(self.directions)

    def stepSim(self):
        self.client.step()

    def stopSim(self):
        self.sim.stopSimulation()

    def RewardCalculation(self, positions):
        blue_objects = positions[:9]
        red_objects = positions[9:]
        distances = [
            np.linalg.norm(np.array(blue) - np.array(red))
            for blue, red in product(blue_objects, red_objects)
        ]
        avg_distance = np.mean(distances) if distances else 0
        reward = 1 / (1 + avg_distance)
        return reward

    def UpdateQtable(
        self,
        state,
        action,
        reward,
        next_state,
        qTable,
        learning_rate=0.1,
        discount_factor=0.9,
    ):
        # print("state:", state)
        # print("action:", action)
        # print("q-table", self.q_table)
        # current_q = self.q_table[max(state), action]
        # max_next_q = np.max(self.q_table[next_state])
        n_state = []
        # for i in state:
        action = self.directions.index(action)
        # print("action", action)
        # print("state", state)
        qTable[state][action] = (1 - learning_rate) * qTable[state][
            action
        ] + learning_rate * (reward + discount_factor * np.max(qTable[next_state]))

    def map_position_to_bin_index(self, pos, n_Bins):
        scaled_pos = (pos + 1) / 2  # Scale position to range [0, 1]
        return min(int(scaled_pos * n_Bins), n_Bins - 1)

    def ConvertPositionsToState(self, positions):
        n_Bins = 5
        discretized_state = [
            self.map_position_to_bin_index(pos[0], n_Bins) for pos in positions
        ]
        state_sum = 0
        for i, index in enumerate(discretized_state):
            state_sum += index * (n_Bins**i)
        return state_sum % 18


def calculateDistance(point1, point2):
    return sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)) ** 0.5


def calculate_centroid(objs):
    return [sum(coord) / len(coord) for coord in zip(*objs)]


def successfullMix(positions):
    blue_objs, red_objs = positions[:9], positions[9:]
    blueCentroid = calculate_centroid(blue_objs)
    redCentroid = calculate_centroid(red_objs)
    distance_between_centroids = calculateDistance(blueCentroid, redCentroid)
    mix_success_threshold = 0.03
    return distance_between_centroids < mix_success_threshold


def qTrainAgent(episodes, steps, qTable):
    # print("hello-")
    totalRewards = []
    for i in range(episodes):
        print(f"Running episode: {i + 1}")
        env = Simulation()
        state = random.randint(0, 17)
        # print(state)
        for step in range(steps):
            action = np.random.choice(env.directions)
            env.action(direction=action)
            positions = env.getObjectsPositions()
            # new_state = random.randint(0, 17)
            new_state = env.ConvertPositionsToState(positions)
            rewards = env.RewardCalculation(positions)
            totalRewards.append(rewards)
            env.UpdateQtable(state, action, rewards, new_state, qTable)
            state = new_state
        env.stopSim()
        if not os.path.exists("output"):
            os.makedirs("output")
        f = open("output/rewards_episode_" + str(i + 1) + ".txt", "w")
        for i in totalRewards:
            f.write("%s\n" % i)
        f.close()
    # print(qTable)

    f = open("output/q_table.txt", "w")
    for i in qTable:
        f.write("%s\n" % i)
    f.close()


def QTesting(episodes, steps, qTable):
    i = 0
    while i < episodes:
        print(f"Running episode: {i + 1}")
        c = 0
        i = i + 1
        env = Simulation()
        state = random.randint(0, 17)
        step = 0
        while step < steps:
            # print(qTable[state])
            action = np.argmax(qTable[state])
            env.action(direction=env.directions[action])
            # new_state = random.randint(0, 17)
            positions = env.getObjectsPositions()
            new_state = env.ConvertPositionsToState(positions)
            rewards = env.RewardCalculation(positions)
            env.UpdateQtable(state, env.directions[action], rewards, new_state, qTable)
            state = new_state
            # print(qTable)
            step = step + 1
            if successfullMix(positions):
                c = c + 1
        env.stopSim()
        print("successful mix for Q_learning ", c)


def randomaction(episodes, steps):
    for episode in range(episodes):
        print(f"Running episode: {episode + 1}")
        env = Simulation()
        c = 0
        for _ in range(steps):
            direction = np.random.choice(env.directions)
            env.action(direction=direction)
            positions = env.getObjectsPositions()
            blue_objs = positions[:9]
            red_objs = positions[9:]
            if successfullMix(positions):
                c = c + 1
        print("successful mix for randoms", c)
        env.stopSim()


def main():
    noOfState = 18
    noOfactions = 4
    # print("hi")

    # training
    qTable = np.zeros((noOfState, noOfactions))
    qTrainAgent(15, 10, qTable)
    print(qTable)

    # testing
    # time.sleep(10)
    start = time.time()
    QTesting(1, 10, qTable)
    end = time.time() - start
    print("end time for Qtesting", end)

    # random
    # time.sleep(10)
    start = time.time()
    randomaction(1, 10)
    end = time.time() - start

    print("end time for random actions", end)


if __name__ == "__main__":
    main()
