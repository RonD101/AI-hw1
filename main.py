# This is a sample Python script.

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.

# Packages needed for this assignment
import gym
import numpy as np
import time
import matplotlib.pyplot as plt
from IPython.display import clear_output # Used to clear the ouput of a Jupyter cell.

env = gym.make('Taxi-v3').env
state = env.reset()
print('Initial state:', state)

env.render()

env.reset() # reset environment to a new, random state
env.render()

print("Action Space {}".format(env.action_space))
print("State Space {}".format(env.observation_space))


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

        epochs += 1
    return (frames, rewards)


frames, reward = randomAgent(328)
print("Reward incurred: {}".format(reward))


def print_frames(frames):
    for i, frame in enumerate(frames):
        clear_output(wait=True)
        print(frame['frame'])
        print(f"Timestep: {i + 1}")
        print(f"State: {frame['state']}")
        print(f"Action: {frame['action']}")
        print(f"Reward: {frame['reward']}")
        time.sleep(0.1)


print_frames(frames)


# def print_hi(name):
#     # Use a breakpoint in the code line below to debug your script.
#     print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.
#
#
# # Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     print_hi('PyCharm')
#
# # See PyCharm help at https://www.jetbrains.com/help/pycharm/
