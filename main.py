# This is a sample Python script.

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.

# Packages needed for this assignment
import gym
import numpy as np
import time
import matplotlib.pyplot as plt
from IPython.display import clear_output  # Used to clear the ouput of a Jupyter cell.

env = gym.make('Taxi-v3').env
env.reset(seed=328)


# state = env.reset()
# print('Initial state:', state)

# env.render()

# env.reset() # reset environment to a new, random state
# env.render()
#
# print("Action Space {}".format(env.action_space))
# print("State Space {}".format(env.observation_space))

# new_state, reward, done, info = env.step(1) # Take action 1 (north)
# env.render()
# print("New state:", new_state)
# print("Reward:", reward)
# print("Done:", done)
# print("Info:", info)


def randomAgent(start_state):
    env.s = start_state  # set environment to illustration's state
    epochs = 0
    penalties, reward = 0, 0

    frames = []  # for animation
    visited = []
    rewards = 0
    done = False
    while not done:
        action = env.action_space.sample()
        state, reward, done, info = env.step(action)

        while state in visited:
            action = env.action_space.sample()
            state, reward, done, info = env.step(action)

        visited.append(state)
        rewards += reward
        if reward == -10:
            penalties += 1

        # Put each rendered frame into dict for animation
        frames.append({
            'frame': env.render(mode='ansi'),
            'state': state,
            'action': action,
            'reward': reward
        }
        )
        print_frames([frames[-1]])

        epochs += 1
    return (frames, rewards)


# frames, reward = randomAgent(328)
# print("Reward incurred: {}".format(reward))


def print_frames(frames):
    for i, frame in enumerate(frames):
        clear_output(wait=True)
        print(frame['frame'])
        print(f"Timestep: {i + 1}")
        print(f"State: {frame['state']}")
        print(f"Action: {frame['action']}")
        print(f"Reward: {frame['reward']}")
        time.sleep(0.1)


# BFS node:
class BFSNode:
    def __init__(self, frame=None, state=None, action=None, reward=None, done=None, info=None):
        self.frame = frame
        self.state = state
        self.action = action
        self.reward = reward
        self.done = done
        self.info = info

    def getNeigbours(node):
        env.s = node.state
        neighbours = []
        for action in range(6):
            new_state, reward, done, info = env.step(action)  # Take action i
            new_node = BFSNode(env.render(mode='ansi'), new_state, action, reward, done, info)
            neighbours.append(new_node)
            env.s = node.state
        return neighbours


def isNodeStateInFront(nodeState, front):
    for node in front:
        if node.state == nodeState:
            return True
    return False


def bfs(first_state):
    # add implementation
    frames = []
    reward = 0
    front = []
    visited = []
    count = 0
    done = False
    front.append(BFSNode(env.render(mode='ansi'), first_state, None, reward))
    while True:
        curr = front[0]
        visited.append(curr.state)
        front.remove(curr)
        env.s = curr.frame
        frames.append({
            'frame': curr.frame,
            'state': curr.state,
            'action': curr.action,
            'reward': curr.reward
        })
        for node in curr.getNeigbours():
            # if node.state not in visited:
            #     # print("V")
            # if node not in front:
            #     # print("F")
            if (node.state not in visited) and (not isNodeStateInFront(node.state, front)):
                if node.done is True:
                    # print("Win bitch")
                    return frames, node.reward
                # print("front len before: ", len(front))
                front.append(node)
                # print("front len after: ", len(front))
        if not front:
            return frames, 0  # need to change to []
        # for elem in front:
        #     env.s = elem.frame
        #     frames.append({
        #         'frame': elem.frame,
        #         'state': elem.state,
        #         'action': elem.action,
        #         'reward': elem.reward
        #     })
        # return frames, -1


""
# driver code - for your use only, do not submit these lines!
env.s = 328
frames, reward = bfs(env.s)
print_frames(frames)
print(reward)
""
