# Continuous Integration Documentation
Documentation on the existing CI for the project is resides here.

## Overview
The existing CI is composed of multiple integration services that together help provide maintainers a fast and scalable testing environment. To help detect upstream breakages quickly as well, the existing CI allows for changes to be evaluated using the latest development dependencies (e.g. using ROS2 master branches). In light of the large dependency footprint a high-level ROS2 navigation stack necessitates, the use of each integration service is optimized to maximize caching of environmental setup and increase workflow throughput. As these optimizations add complexity to the CI configuration, this documentation provides further explanations and reasoning behind each configuration.

![pipeline](figs/pipeline.svg)

The figure above is a high level diagram on how the integration services described below are composed.

## Integrations

The following links document each integration and are best approached in the same order presented.

### GitHub

GitHub is used for hosting the source repo, tickets and PRs, as well for managing the OAuth and configs for the rest of the other integration services in the CI pipeline.

### [Dockerfile](dockerfile.md)

Dockerfiles are used for generating the docker images for building and testing the project. They also self document the setup and configuration of upstream dependencies, ensuring contributors have a reproducible and repeatable development environment for collaboration.

### [DockerHub](dockerhub.md)

DockerHub is used to build and host the repo of tagged docker images, so that downstream services in the CI pipeline can quickly download and bootstrap the latest up-to-date development environment.

### [CircleCI](circleci.md)

CircleCI is used to checkout, build, and test the project via docker containers spawned from the tagged docker images. Triggered by scheduled or GitHub events like pushed commits branches or pull requests, it deterministically retains a warm build cache while generating logs and test result artifacts.

### [CodeCov](codecov.md)

CodeCov is used to help monitor code quality by rendering test artifacts from the upstream pipeline into interactive analytics, improving the visibility of the project's health and feedback for contributions.

### [Future Work](future_work.md)

The CI has room for improvement and may still evolve over time, as alternate integration options become more viable, and the pros and cons of each shift.
