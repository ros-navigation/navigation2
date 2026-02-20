# AMCL
Adaptive Monte Carlo Localization (AMCL) is a probabilistic localization module which estimates the position and orientation (i.e. Pose) of a robot in a given known map using a 2D laser scanner. This is largely a refactored port from ROS 1 without any algorithmic changes.

See the [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-amcl.html) for more details about configurable settings and their meanings.

## Parameter note: `random_seed`

AMCL supports a `random_seed` parameter to control the particle filter RNG seeding.

- `random_seed >= 0`: seed the RNG with the provided value (repeatable runs).
- `random_seed < 0` (default): seed the RNG from time (preserves historical behavior).
