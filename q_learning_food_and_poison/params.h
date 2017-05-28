#pragma once

#define USE_PROGRAMMED_BEHAVIOR false

#define SHOW_INFOS false

#define NR_ACTIONS 3

#define FOOD_SENSOR_RADIUS 75

#define FOOD_RADIUS 8
#define NR_FOOD_PIECES 400

#define REWARD_BUMPING_INTO_WALL -1000.0
#define REWARD_GREEN_FOOD +1000.0
#define REWARD_RED_FOOD -1000.0
#define REWARD_ONE_STEP -1.0

#define Q_VALUES_FILENAME "q_values/q_values_food_and_poison.txt"

#define EPSILON_EXPLORATION 0.05
#define GAMMA 0.9
#define LEARN_RATE 0.25
