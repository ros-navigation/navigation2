Dockerfiles are 

configured via the [Dockerfile](/Dockerfile) yaml file within the `.circleci/` folder at the root of the GitHub repo. Further references on configuring CircleCI, such as syntax and structure can be found here:

* https://circleci.com/docs/2.0/writing-yaml
* https://circleci.com/docs/2.0/configuration-reference

https://docs.docker.com/engine/reference/builder/


* https://hub.docker.com/_/ros
* https://github.com/osrf/docker_images


* [colcon](https://colcon.readthedocs.io/en/released)
  * CLI tool to build sets of software packages
* [rosdep](http://wiki.ros.org/rosdep)
  * CLI tool for installing system dependencies
* [vcstool](https://github.com/dirk-thomas/vcstool)
  * CLI for working with multiple repositories easier



* [ccache](https://ccache.dev)
  * Compiler cache for speeding up recompilation
* [lcov](http://ltp.sourceforge.net/coverage/lcov.php)
  * Front-end for GCC's coverage testing tool gcov

* https://docs.docker.com/develop/develop-images/build_enhancements/
* https://github.com/moby/buildkit


[cache apt packages](https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/experimental.md#example-cache-apt-packages) to avoid unnecessarily re-downloading the same packages from over the network, even if the docker image layer cache for that directive in the Dockerfile is busted.
