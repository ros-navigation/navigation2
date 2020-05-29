# DockerHub Documentation

DockerHub is a service used to build docker images from the project's Dockerfiles, as well as a registry used to host tagged images built. Using a docker registry permits the project to offload much of the environmental setup from the rest of the CI pipeline. More info on DockerHub can be found here:

* [DockerHub](https://hub.docker.com/)
* [Docker Hub Quickstart](https://docs.docker.com/docker-hub)

The tagged images on the project's repo in the registry are used by the CI pipeline to spawn containers to build and test the project. The advantage of hosting such images in a registry to readly pull from rather than merely re-building the Dockerfiles in CI tests enables frontloading much of the principled environmental setup prior to the start of CI jobs. This saves build time, spares resources/credits for other jobs, and helps to accelerate the development cycle. The project's DockerHub repo can be found here:

* [Navigation2 on DockerHub](https://hub.docker.com/r/rosplanning/navigation2)

While DockerHub does not require the use of configuration files in the source repo, the Dockerfiles and respective scripts used to customize automated builds of images are tracked in the [`.dockerhub`](/.dockerhub) directory. These scripts include custom build phase hooks used by DockerHub during automated builds. More info on automated builds can be found here:

* [How Automated Builds work](https://docs.docker.com/docker-hub/builds)
* [Advanced options for Autobuild and Autotest](https://docs.docker.com/docker-hub/builds/advanced)

Automated Builds must be enabled via the build configurations menu within the repo's administrative console on DockerHub. For reference, a figure of the project's build configurations is shown here:

![DockerHub Build Configurations](figs/dockerhub_build_configurations.png)

Here the repo's source repository is linked to the project's GitHub repo, while Autotest is disabled given a dedicated CI service is separately used to run test jobs. Repository linking is enabled so that whenever the base image is updated on DockerHub, it will also trigger a build for the project's DockerHub repository. Note that this only works for non-official images. Two build rules are added for the master branch, providing both a release and debug tag for CI to pull from. The paths to the relative Dockerfiles are designated, while the build context is intentionally left empty. This ensures that the build phase hooks within the same paths are used appropriately. Build caching is also enabled to shorten image turnaround time if multiple rebuilds a day are triggered. 

The build hooks, e.g. [build](/.dockerhub/debug/hooks/build) , serve a few purposes. The first being to customize docker commands to override build arguments denoted in the Dockerfile via the `ARG` directives, such as adjusting the colcon mixins for each workspace to enable code coverage, or disabling fail on build failure so the breakages in master don't block the building of CI images for master.

The build hooks secondly allow for the redirection of target Dockerfile and build context, given the use of `ARG` to parameterize the parent image to build `FROM` inhibits the normal use of repository links. This is a limitation of the Dockerfile parsing used by DockerHub to infer what repo/tag the build rule should trigger from. 

As a workaround, the build rule points to a dummy Docker file that includes the desired parent image tag to build `FROM`, while the build hook then uses the main Dockerfile with the same `FROM_IMAGE`. This allows the slower debug CI to build and test only using the default RMW, while allowing the faster release CI jobs to build and test from an image with more RMWs installed.

## Alternatives

While the free registry for hosting is great, allowing project gigabytes of free storage/bandwidth to cache pre-configured images for various branches or CI scenario, that would otherwise take free tier CI instances far longer to rebuild from scratch rather than pull from registry, the automated build integration isn't as easily configurable for advance use cases. Also, given that automated build rules are configured out-of-band of the source repo for example, forking the project would not self contain all the configuration needed to simply mirror the CI setup.

There are a few alternatives such as triggering builds via APIs or building the images remotely from different CI providers and pushing them back to docker registry. Although each has its pros and cons.

### Build Trigger API

Rather than configuring the build rules on DockerHub, a build trigger URL can be generated for the linked repo and used to programmatically specify build parameters such as: tag names, context path, source branch or version, etc. Although it seems this API is now less documented. More info on Build Trigger can be found here:

* [Remote Build Triggers](https://github.com/docker/docker.github.io/blob/v17.06-release/docker-hub/builds.md#remote-build-triggers)
  * Legacy docs on using build triggers
* [Example GitHub Action](https://github.com/osrf/docker_images/blob/master/.github/workflows/trigger_nightly.yaml)
  * Scheduled cron job used to rebuild a nightly image
* [Example Build Hook](https://github.com/osrf/docker_images/blob/master/ros2/nightly/nightly/hooks/post_push)
  * A hook to rebuild a child image post push or parent tag

However, triggering builds via API rather than relying on the DockerHub build rules also means forgoing the convenience of repository linking, where a repo's image can be sure to be rebuilt as soon as a new version of the parent image tag is pushed to the registry without needing to monitor the parent image repos oneself. This helps keep the CI environment uptodate and in sync with upstream development. 

### Outsource Image Builds

Instead of using DockerHub as the remote builder, any CI that supports Docker can similarly be used to build and push images to DockerHub's registry. As of writing however, aside from username/password authentication, DockerHub only provides personal access tokens tied to individual usernames rather than an organizations. This is a bit tedious, as to manage the CI as an organization, a separate machine account must be created and delegated with user permissions to push to the DockerHub repo. More info on CI integration with Docker can be found here:

* [Docker Hub: Managing access tokens](https://docs.docker.com/docker-hub/access-tokens/)
* [Docker + GitHub Actions](https://github.com/marketplace/actions/build-and-push-docker-images)
* [Docker + CircleCI Orbs](https://circleci.com/orbs/registry/orb/circleci/docker)
* [Docker + Azure Tasks](https://docs.microsoft.com/en-us/azure/devops/pipelines/tasks/build/docker?view=azure-devops#build-and-push)

Additionally, local build caching is often a premium feature for most other services, thus to benefit from docker build caching, one must manage a build agent for docker builds, or pony up to upgrade from a conventional open source free tier CI plan.
