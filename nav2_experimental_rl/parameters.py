# dqn parameters
GAMMA = 0.90
LEARNING_RATE = 0.005
MEMORY_SIZE = 1000000
BATCH_SIZE = 64
EXPLORATION_MIN = 0.25
EXPLORATION_MAX = 1.0
EXPLORATION_TARGET_STEP = 1000

# model train parameters
TARGET_MODEL_UPDATE_STEP = 100 # Number of steps before updating the target model
EPISODES = 10000 # Desired number of episodes to run
LOOP_RATE = 0.2 # Control loop rate in seconds

# Turtlebot 3 environment parameters
LINEAR_FWD_VELOCITY = 0.16
ANGULAR_FWD_VELOCITY = 0.0
LINEAR_STR_VELOCITY = 0.06
ANGULAR_VELOCITY = 0.6