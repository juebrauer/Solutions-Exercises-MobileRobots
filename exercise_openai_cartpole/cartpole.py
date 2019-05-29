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
# -->
observations_min = [-2.3860441, -3.27420053, -0.20943904, -3.2074541]
observations_max = [+2.38207215, +2.87269719, +0.20943357, +3.28998744]
nr_bins_per_observation_dimension = 10

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

    global nr_bins_per_observation_dimension

    # 1. start with an empty state vector
    state_vector = []

    # 2. map each sensor value to a state number
    for sensor_val_nr,sensor_val in enumerate(observation):

        # 2.1 what is the minimum and maximum possible sensor value?
        min_sensor_val = observations_min[sensor_val_nr]
        max_sensor_val = observations_max[sensor_val_nr]

        # 2.2 how large is the sensor value range?
        range = max_sensor_val - min_sensor_val

        # 2.3 shift sensor value from, e.g. [-0.75,1.25] to [0.0, 2.0]
        shift = -min_sensor_val
        shifted_sensor_val = sensor_val+shift

        # 2.4 compute into which bin the shifted sensor value falls
        binned_sensor_val =\
            int((shifted_sensor_val/range)*(nr_bins_per_observation_dimension-1))

        # 2.5
        # make sure, we really use only nr_bins_per_observation_dimension
        # many bins and not nr_bins_per_observation_dimension+1 many!
        if binned_sensor_val >= nr_bins_per_observation_dimension:

            # Note: these lines should never be reached!
            print("ATTENTION! Value out of bins error!")
            input()

        # 2.6 add that binned sensor value to state vector
        state_vector.append(binned_sensor_val)

    # 3.
    # we have mapped the observation vector
    # to a state vector. Return this state vector!
    return state_vector




def main():

    # 1. show information about min/max values of
    #    arguments of the observation vector
    global observations_min,observations_max
    global nr_bins_per_observation_dimension
    print("observations_min:", observations_min)
    print("observations_max:", observations_max)


    # 2. set hyper-parameters:
    NR_EPISODES = 1
    MAX_STEPS_PER_EPISODE = 1000


    # 3. load the environment
    my_env = gym.make('CartPole-v1')
    # env = gym.make('MountainCar-v0')


    # 4. which type has the observation space?
    show_information_about_observation_space(my_env)


    # 5. observations will be mapped to states:
    #    show information about the state space
    print("Each dimension of the observation space is binned into"
          " {} bins.".format(nr_bins_per_observation_dimension) )
    observation = my_env.reset()
    dim = len(observation)
    nr_states = pow(nr_bins_per_observation_dimension, dim)
    print("So there are {}^{}={} many states"
          .format(nr_bins_per_observation_dimension, dim, nr_states))


    # 6. which type has the action space?
    show_information_about_action_space(my_env)


    # 7. Only needed when we determine min/max observation values
    #observations_min = np.zeros(4)
    #observations_max = np.zeros(4)


    # 8. Q-Learning:
    for episode_nr in range(NR_EPISODES):

        # get initial observation
        observation = my_env.reset()

        # for all steps in the current episode...
        for step_nr in range(MAX_STEPS_PER_EPISODE):

            # 8.1 show the world before we take the action
            my_env.render()


            # 8.2 sample a random action from the action space
            action = my_env.action_space.sample()


            # 8.3 do the action
            observation, reward, done, info = my_env.step(action)


            # 8.4 map observation to state vector
            state_vector = map_observation_to_state_vector(observation)


            # 8.5 show observation data and state vector
            # observation data is [position of cart, velocity of cart, angle of pole, rotation rate of pole]
            print("step {}: new observation={}, action={}, reward={}, done={}"
                  .format(step_nr,observation,action,reward,done))
            print(state_vector)


            # 8.6 is the episode at the end?
            if done:
                print("Episode {} finished after {} timesteps"
                      .format(episode_nr, step_nr))
                break


            # 8.7
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
