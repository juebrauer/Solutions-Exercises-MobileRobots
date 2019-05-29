# Example how to use Q-Learning in order to solve
# the cartpole problem
# - one of many challenges from the OpenAI gym world
#
# "CartPole-v0
# A pole is attached by an un-actuated joint to a cart,
# which moves along a frictionless track. The system
# is controlled by applying a force of +1 or -1 to the
# cart. The pendulum starts upright, and the goal is
# to prevent it from falling over. A reward of +1 is
# provided for every timestep that the pole remains
# upright. The episode ends when the pole is more
# than 15 degrees from vertical, or the cart moves
# more than 2.4 units from the center."
#
# Source: https://gym.openai.com/envs/CartPole-v0/
#
# For getting started with OpenAI gym:
# see https://gym.openai.com/docs/
#

import numpy as np

# for time.sleep()
import time

import gym

# For infos about the CartPole observation and action space see e.g.
# https://github.com/openai/gym/wiki/CartPole-v0
#
# Observation:
# ------------
# Type: Box(4)
# Num 	Observation 	Min 	Max
# 0 	Cart Position 	-2.4 	2.4
# 1 	Cart Velocity 	-Inf 	Inf
# 2 	Pole Angle 	~ -0.418 	~ 0.418
# 3 	Pole Velocity At Tip 	-Inf 	Inf
#
# When running 50.000 episodes till they ended (i.d. done=True)
# I got the following minimum and maximum values for the individual
# entries in the observation vector:
#   observations_min: [-2.3860441  -3.27420053 -0.20943904 -3.2074541 ]
#   observations_max: [2.38207215 2.87269719 0.20943357 3.28998744]
#
#
# Actions:
# --------
# Type: Discrete(2)
# Num 	Action
# 0 	Push cart to the left
# 1 	Push cart to the right
#
# Reward:
# -------
# Reward is 1 for every step taken, including the termination step
#
# Episode Termination:
# --------------------
#  - Pole Angle is more than ±12°
#  - Cart Position is more than ±2.4 (center of the cart reaches the edge of the display)
#  - Episode length is greater than 200
#

def show_information_about_observation_space(your_env):

    print("\nObservation space:")
    print("\tType:", your_env.observation_space)

    print("\tHere are the upper values of the observation box:",
           your_env.observation_space.high)

    print("\tHere are the lower values of the observation box:",
          your_env.observation_space.low)


def show_information_about_action_space(your_env):

    print("\nAction space:")
    print("\tType", your_env.action_space)
    print("\t10 example actions:", [your_env.action_space.sample() for i in range(10)] )
    print("\tTotal number of possible actions:", your_env.action_space.n)


def map_observation_to_state_vector(observation):


    return []




def main():

    # set hyper-parameters:
    observations_min = [-2.3860441, -3.27420053, -0.20943904, -3.2074541]
    observations_max = [+2.38207215, +2.87269719, +0.20943357, +3.28998744]
    NR_EPISODES = 1
    MAX_STEPS_PER_EPISODE = 1000

    my_env = gym.make('CartPole-v1')
    # env = gym.make('MountainCar-v0')

    show_information_about_observation_space(my_env)
    show_information_about_action_space(my_env)

    # Only needed when we determine min/max observation values
    #observations_min = np.zeros(4)
    #observations_max = np.zeros(4)

    for episode_nr in range(NR_EPISODES):

        # get initial observation
        observation = my_env.reset()

        for step_nr in range(MAX_STEPS_PER_EPISODE):

            # show the world before we take the action
            my_env.render()

            # sample a random action from the action space
            action = my_env.action_space.sample()

            # do the action
            observation, reward, done, info = my_env.step(action)

            # map observation to state vector
            state_vector = map_observation_to_state_vector(observation)

            # show observation data
            # observation data is [position of cart, velocity of cart, angle of pole, rotation rate of pole]
            print("step {}: new observation={}, action={}, reward={}, done={}"
                  .format(step_nr,observation,action,reward,done))

            # is the episode at the end?
            if done:
                print("Episode {} finished after {} timesteps"
                      .format(episode_nr, step_nr))
                break

            # determine minimum and maximum values
            # observation vectors
            observations_min = np.minimum(observation, observations_min)
            observations_max = np.maximum(observation, observations_max)

            #time.sleep(0.1)

        # end-for (step_nr)

    # end-for (episode_nr)

    print("observations_min:", observations_min)
    print("observations_max:", observations_max)

    my_env.close()

main()
