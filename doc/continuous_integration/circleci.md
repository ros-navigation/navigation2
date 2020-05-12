https://circleci.com/gh/ros-planning/navigation2

CircleCI is configured via the [config.yml](/.circleci/config.yml) yaml file within the `.circleci/` folder at the root of the GitHub repo. Further references on configuring CircleCI, such as syntax and structure can be found here:

* https://circleci.com/docs/2.0/writing-yaml
* https://circleci.com/docs/2.0/configuration-reference

The config file for this project is self contained and thus densely structured, yet written in a functional style to remain DRY and modular. This keeps it easily generalizable for other ROS packages or for adjusting overlayed workspace. Given the anchors and references in yaml, it's consequently best approached by reading the config file from bottom to top in order of abstraction hierarchy.

## Workflows

## Jobs

## Executors

## Commands

## References

### Common Environment

### Common Commands

#### Caching

#### Building

#### Testing

### Checkout

### Workspaces

### Code Coverage