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
import matplotlib.pyplot as plt

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

    global nr_actions

    print("\nAction space:")
    print("\tType", your_env.action_space)
    print("\t10 example actions:", [your_env.action_space.sample() for i in range(10)] )

    nr_actions = your_env.action_space.n
    print("\tTotal number of possible actions:", nr_actions)


#
# observation (continuous) vector --> state vector (discrete)
#
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
        # This could happen, if the actual sensor values are smaller or larger
        # than the recorded minimum/maximum values
        if binned_sensor_val < 0:
            binned_sensor_val = 0
        if binned_sensor_val >= nr_bins_per_observation_dimension:
            binned_sensor_val = nr_bins_per_observation_dimension-1

        # 2.6 add that binned sensor value to state vector
        state_vector.append(binned_sensor_val)

    # 3.
    # we have mapped the observation vector
    # to a state vector. Return this state vector!
    return state_vector


#
# make sure, before we use Q(s,a), that we have
# made an entry in the Q-Table for (s,a):
def make_sure_key_exists(s):

    global Q, nr_actions

    # check whether there is already an entry for that
    # state s in the dictionary for the key (s,0)?
    key = (str(s), 0)

    if not key in Q:

        # no! there is no entry!
        # so all they keys (s,0) and (s,1) do not exist
        # for this, generate the initial entries
        for a in range(0, nr_actions):
            Q[(str(s), a)] = 0.0

#
# Given the current Q-values for (state,action) pairs and
# current state-action pair (s,a)
# Which is the best action a* that we can take in state s?
# i.e. a* = argmax Q(s,a)
#              a
#
def get_best_action(s):

    global Q, nr_actions

    make_sure_key_exists(s)

    # Now we are sure, that there are entries for state
    # s and all actions a in the Q-value table Q(s,a).
    # We can therefore search for the action a, that
    # maximizes the Q-value:
    best_action = 0
    for a in range(1,nr_actions):

        # found larger Q-value
        if Q[(str(s),a)] > Q[(str(s),best_action)]:
            best_action = a

    # 3. return the action a which has maximum Q(s,a) value
    return best_action


# Given a state s,
# search for the best action a,
# that will lead to the most future reward.
# This is known as the utility of the state s
def get_state_utility(s):

    global Q

    best_action = get_best_action(s)
    utility_of_state_s = Q[(str(s),best_action)]
    return utility_of_state_s



# global variables:
nr_actions = -1
Q = {}

def main():

    # 1. show information about min/max values of
    #    arguments of the observation vector
    global observations_min,observations_max
    global nr_bins_per_observation_dimension
    global Q
    print("observations_min:", observations_min)
    print("observations_max:", observations_max)


    # 2. set hyper-parameters:
    NR_EPISODES = 1000000
    MAX_STEPS_PER_EPISODE = 1000
    SHOW_ENVIRONMENT = False

    # greedy exploitation or more exploration?
    EPSILON_EXPLORATION_VS_EXPLOITATION = 0.05

    # learn rate
    ETA = 0.5

    # discounting factor: how much do future rewards matter?
    GAMMA = 0.9

    # for development
    SHOW_DEBUG_INFO = False

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
    episode_lengths = []
    N = 200
    mean_episode_lengths = [0 for i in range(0, N)]
    plt.xlabel("Episode nr")
    plt.ylabel("Episode length")


    for episode_nr in range(NR_EPISODES):

        # get initial observation
        observation_s1 = my_env.reset()
        state_vector_s1 = map_observation_to_state_vector(observation_s1)

        action_counters = [0,0]

        # for all steps in the current episode...
        for step_nr in range(MAX_STEPS_PER_EPISODE):

            # 8.1 show the world before we take the action
            if SHOW_ENVIRONMENT:
                my_env.render()


            # 8.2 determine action to perform
            rnd_number = np.random.uniform(0.0, 1.0)
            if rnd_number<EPSILON_EXPLORATION_VS_EXPLOITATION:
                # Exploration mode:
                # sample a random action from the action space
                a = my_env.action_space.sample()
            else:
                # (Greedy) Exploitation mode
                a = get_best_action( state_vector_s1 )
            action_counters[a] +=1

            # 8.3 do the action
            observation_s2, reward, done, info = my_env.step(a)
            if done:
                reward = -1.0


            # 8.4 map observation to state vector
            state_vector_s2 = map_observation_to_state_vector(observation_s2)


            # 8.5 show observation data and state vector
            # observation data is [position of cart, velocity of cart, angle of pole, rotation rate of pole]
            if SHOW_DEBUG_INFO:
                print("step {}: new observation={}, action={}, reward={}, done={}"
                      .format(step_nr,observation_s2,a,reward,done))
                print(state_vector_s2)


            # 8.6 Q-learning happens here!
            s = state_vector_s1
            future_reward = get_state_utility( state_vector_s2 )
            make_sure_key_exists(s)
            Q[(str(s),a)] = Q[(str(s),a)] +\
                            ETA *(reward + GAMMA*future_reward - Q[(str(s),a)])
            #                     ||||||||||||||||||||||||||||   XXXXXXXXXXXXX
            #                     new estimated utility for      old estimated
            #                     Q(s,a)                         utility of Q(s,a)
            #
            #                     the difference between | and X is an error signal
            #                     that shows us in which direction to correct our
            #                     estimation of the utility of Q(s,a)


            # 8.7 is the episode at the end?
            if done:

                episode_lengths.append(step_nr)

                # given the episode lengths,
                # compute the average of each N
                # episode lengths
                M = len(episode_lengths)
                if M>N:
                    i = M-N
                    sum = 0
                    for j in range(i,i+N):
                        sum += episode_lengths[j]
                    avg_episode_len = sum/N
                    mean_episode_lengths.append(avg_episode_len)

                print("Episode {} finished after {} timesteps. "
                      "Action counters: {} "
                      "Mean episode length: {} "
                      .format(episode_nr, step_nr,
                              action_counters,
                              mean_episode_lengths[-1]))

                break


            # 8.8
            # determine minimum and maximum values
            # observation vectors
            observations_min = np.minimum(observation, observations_min)
            observations_max = np.maximum(observation, observations_max)

            # 8.9
            # current state s2 becomes start state s1 for next step
            state_vector_s1 = state_vector_s2

            # 8.10
            # wait for some time to see the behavior of the agent
            # in the world
            if SHOW_ENVIRONMENT:
                time.sleep(0.02)

        # end-for (step_nr)


        # plot episode lengths and mean episode lengths        
        if episode_nr % 100 == 0:
            plt.clf()         
            plt.plot(episode_lengths, color="red", label="lengths", marker="+", linestyle="")
            plt.plot(mean_episode_lengths, color="green", label="avg lengths")
            plt.legend()
            plt.draw()
            plt.title("Episode lengths")
            plt.ion()
            plt.show(block=False)
            time.sleep(0.01)
            plt.pause(1)

    # end-for (episode_nr)

    print("observations_min:", observations_min)
    print("observations_max:", observations_max)
    
    my_env.close()

main()
