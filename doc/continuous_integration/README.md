# Continuous Integration Documentation
Documentation on the existing CI for the project is resides here.

## Overview
The existing CI is composed of multiple integration services that together help provide maintainers a fast and scalable testing environment. To help detect upstream breakages quickly as well, the existing CI allows for changes to be evaluated using the latest development dependencies (e.g. using ROS2 master branches). In light of the large dependency footprint a high-level ROS2 navigation stack necessitates, the use of each integration service is optimized to maximize caching of environmental setup and increase workflow throughput. As these optimizations add complexity to the CI configuration, this documentation provides further explanations and reasoning behind each configuration.

![pipeline](figs/pipeline.svg)

The figure above is a high level diagram on how the integration services described below are composed.

## Integrations

The following links document each integration and are best approached in the same order presented.

### [GitHub](github.md)

GitHub is used for hosting the source repo, tickets and PRs, as well for managing the OAuth and configs for the rest of the other integration services in the CI pipeline.

### [Dockerfile](dockerfile.md)

Dockerfiles are used for generating the docker images for building and testing the project. They also self document the setup and configuration of upstream dependencies, ensuring contributors have a reproducible and repeatable development environment for collaboration.

### [DockerHub](dockerhub.md)

DockerHub is used to build and host the repo of tagged docker images, so that downstream services in the CI pipeline can quickly download and bootstrap the latest up-to-date development environment.

### [CircleCI](circleci.md)

CircleCI is used to checkout, build, and test the project via docker containers spawned from the tagged docker images. Triggered by scheduled or GitHub events like pushed commits branches or pull requests, it deterministically retains a warm build cache while generating logs and test result artifacts.

### [CodeCov](codecov.md)

CodeCov is used to help monitor code quality by rendering test artifacts from the upstream pipeline into interactive analytics, improving the visibility of the project's health and feedback for contributions.

## Future Work

The CI has room for improvement and may still evolve over time. The following notes alterative integration options, including current pros and cons for each.

###  [GitHub Actions](https://github.com/features/actions)

Github actions is an emerging container based CI service that tightly integrates with the rest of GitHub's service offerings. With a growing ecosystem of official and federated 3rd party actions available, one can compose custom and extensive CI/CD workflows. 

#### Pros:

* Self hosted runners
  * Optionally run workflows form on site, not just cloud VMs
  * https://docs.github.com/en/free-pro-team@latest/actions/hosting-your-own-runners
  * Leverage local hardware, e.g: GPUs, persistent storage, robot sensors, etc.

#### Cons:

* No test introspection
  * One must still roll there own test result reporting
  * https://github.community/t/publishing-test-results/16215/12
  * Xunit test results are not rendered, aggregated, nor summarized
* Restricted caching
  * Caching with runners is less ergonomic than other CI providers
  * https://github.com/microsoft/azure-pipelines-agent/issues/2043
  * Implementation inherits same limitation from azure-pipelines-agent
* No job level parallelism
  * No equivalent parallelism for splitting tests via timing data
  * https://circleci.com/docs/2.0/parallelism-faster-jobs
  * Parameterizable parallelism without adding jobs to workflow
* No RAM Disk access
  * Useful to improve file IO performance
  * https://circleci.com/docs/2.0/executor-types/#ram-disks
  * Applicable for frequent reads/writes, e.g. ccache
