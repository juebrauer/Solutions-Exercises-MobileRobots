#pragma once

#define WAIT_FOR_KEYPRESS_AFTER_EACH_SIMU_STEP false

#define SHOW_INFOS true

#define NR_ACTIONS 4

#define FOOD_RADIUS 8
#define NR_FOOD_PIECES 700

#define REWARD_GREEN_FOOD +1.0
#define REWARD_RED_FOOD -1.0
#define REWARD_ENERGY_LOSS -0.02

#define FOODTYPE_GREEN 0
#define FOODTYPE_RED 1

#define LEARN_RATE 0.01

#define SAVE_IMAGES 0
#define SAVE_FOLDER "~/tmp"
#define PLAY_SOUND false


#define OBJECT_TYPE_WALL 0
#define OBJECT_TYPE_GREEN_FOOD 1
#define OBJECT_TYPE_RED_FOOD 2
#define OBJECT_TYPE_OPEN_SPACE 3

#define STATE_ACTION_HISTORY_MAX_LEN 5
#define EPSILON_EXPLORATION 0.01

#define ACTION_MOVE_LEFT 0
#define ACTION_MOVE_RIGHT 1
#define ACTION_MOVE_TOP 2
#define ACTION_MOVE_DOWN 3

#define ROBOT_DISTANCE_PER_MOVE 10

static string action_str[4] = { "LEFT", "RIGHT", "TOP", "DOWN" };

static string food_names[2] = { "GREEN-FOOD", "RED-FOOD" };

#define NR_DIFFERENT_ANGLES 10

#define LIMIT_SCORES false

