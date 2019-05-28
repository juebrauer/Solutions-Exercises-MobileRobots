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

import gym
my_env = gym.make('CartPole-v0')
#env = gym.make('MountainCar-v0')

# For infos about the CartPole observation and action space see e.g.
# https://github.com/openai/gym/wiki/CartPole-v0
#
# However, when printing
# your_env.observation_space.high and
# your_env.observation_space.low
# I get other value ranges, namely these:
#
# Observation:
# ------------
# Type: Box(4)
# Num 	Observation 	Min 	Max
# 0 	Cart Position 	-4.8 	4.8
# 1 	Cart Velocity 	-Inf 	Inf
# 2 	Pole Angle 	~ -0.418 	~ 0.418
# 3 	Pole Velocity At Tip 	-Inf 	Inf
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




def main():

    show_information_about_observation_space(my_env)
    show_information_about_action_space(my_env)

    NR_EPISODES = 1
    MAX_STEPS_PER_EPISODE = 1000

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

            # show observation data
            # observation data is [position of cart, velocity of cart, angle of pole, rotation rate of pole]
            print("step {}: new state s={}, action={}, reward={}, done={}"
                  .format(step_nr,observation,action,reward,done))

            # is the episode at the end?
            if done:
                print("Episode {} finished after {} timesteps"
                      .format(episode_nr, step_nr))
                break

        # end-for (step_nr)

    # end-for (episode_nr)

    my_env.close()

main()
