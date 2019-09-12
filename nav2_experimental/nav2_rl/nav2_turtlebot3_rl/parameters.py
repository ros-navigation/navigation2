# dqn parameters
GAMMA = 0.99
LEARNING_RATE = 0.0001
MEMORY_SIZE = 10000000
BATCH_SIZE = 64
EXPLORATION_MIN = 0.15
EXPLORATION_MAX = 1.0
EXPLORATION_TARGET_STEP = 10000


# model train parameters
TARGET_MODEL_UPDATE_STEP = 40  # Number of steps before updating the target model
EXPERIENCE_REPLAY_STEP = 10  # Number of episodes before calling experience replay
EPISODES = 100000  # Desired number of episodes to run
LOOP_RATE = 0.2  # Control loop rate in seconds

# Turtlebot 3 environment parameters
LINEAR_FWD_VELOCITY = 0.16
ANGULAR_FWD_VELOCITY = 0.0
LINEAR_STR_VELOCITY = 0.06
ANGULAR_VELOCITY = 0.6

ZERO = 0.0
FWD_VELOCITY = 0.16
SPIN_VELOCITY = 0.16
STEER_FWD_VELOCITY = 0.08
STEER_VELOCITY = 0.08
