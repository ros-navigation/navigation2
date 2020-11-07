# Dockerfile Documentation

Dockerfiles, denoted via the `(<name>.)Dockerfile` file name extension, provides repeatable and reproducible means to build and test the project, as well as build images for running container based CI services. Further references on writing and building Dockerfiles, such as syntax and tooling can be found here:

* [Dockerfile reference](https://docs.docker.com/engine/reference/builder)
* [Best practices for writing Dockerfiles](https://docs.docker.com/develop/develop-images/dockerfile_best-practices)

The Dockerfiles for this project are built upon parent images from upstream repos on DockerHub, thus abbreviating environmental setup and build time, yet written in a parameterized style to remain ROS2 distro agnostic. This keeps them easily generalizable for future ROS2 releases or for switching between custom parent images. When choosing the parent image, a tradeoff may persist between choosing a larger tag with more than what you need pre-installed, saving time building the image locally, vs. choosing a smaller tag without anything you don't need, saving time pulling or pushing the image remotely. Given the use of multiple build stages, they're consequently best approached by reading from top to bottom in the order in which image layers are appended. More info on upstream repos on DockerHub can be found here:

* [ROS Docker Images](https://hub.docker.com/_/ros)
  * DockerHub repo for official images
* [ROS Dockerfiles](https://github.com/osrf/docker_images)
  * GitHub repo for OSRF Dockerfiles
* [Official Images on Docker Hub](https://docs.docker.com/docker-hub/official_images)

While the main [`Dockerfile`](/Dockerfile) at the root of the repo is used for development and continuous integration, the [`.dockerhub/`](/.dockerhub) directory contains additional Dockerfiles that can be used for building the project entirely from scratch, include the minimal spanning set of recursive ROS2 dependencies from source, or building the project from a released ROS2 distro using available pre-built binary dependencies. These are particularly helpful for developers needing to build/test the project using a custom ROS2 branch, or for a users building with an alternate ROS2 base image, but are not used for the CI pipeline. We'll walk through the main Dockerfile here, although all of them follow the same basic pattern.

## Global Arguments

The Dockerfile first declares a number of optional `ARG` values and respective defaults to specify the parent image to build `FROM` and workspace paths. Here the Dockerfiles assume all workspaces are nested within the `/opt` directory. These `ARG`s can be accessed similarly to `ENV`s, but must be declared in a stage's scope before they can be used, and unlike `ENV` only exist at build time of that stage and do not persist in the resulting image. For multi-stage builds, the last stage is what is tagged as the final image. More info on multi-stage builds can be found here: 

* [Use multi-stage builds](https://docs.docker.com/develop/develop-images/multistage-build)
  * Optimize while keeping Dockerfiles readable and maintainable

Here the parent image to build `FROM` is set to `osrf/ros2:nightly` by default, allowing the master branch to simply build against the bleeding edge of ROS2 core development. This allows project maintainers to spot breaking API changes and regressions as soon as they are merged upstream. Alternatively, any image tag based on a released ROS2 distro image, e.g. `ros:<ROS2_DISTRO>`, could also be substituted to compile the project, say for quickly experimenting with planners ontop of complex reinforcement or deep learning framework dependencies.

## Cacher Stage

A `cacher` stage is then started to gather together the necessary source files to eventually build. A directory for the underlay workspace is then created and populated using `vcs` with the respective `.repos` file that defines the relevant repositories to pull and particular versions to checkout into the source directory. More info on vcstool can be found here: 

* [vcstool](https://github.com/dirk-thomas/vcstool)
  * CLI for working with multiple repositories easier

The `.repos` file is not copied directly into the `src` folder to avoid any restructuring of the yaml data from unintentionally busting the docker build cache in later stages. The ephemeral files within `.git` repo folders are similarly removed and help bolster deterministic builds. 

The overlay workspace is then also created and populated using all the files in the docker build context, (e.g. the root directory of the branch being built). This is done after the underlay is cloned to avoid re-downloading underlay dependency source files if the `.repos` file is unchanged in the branch. However, if the `.repos` file is changed, and different source files are cloned, this will bust the docker build cache and clean rebuild. Other project files are safely ignored using the [`.dockerignore`](/.dockerignore) config. If the docker build's cache is somehow stale, using the docker build flag `--no-cache` may be used to freshly build anew.

Finally the `cacher` stage copies all manifest related files in place within the `/opt` directory into a temporary mirrored directory that later stages can copy from without unnecessarily busting it's docker build cache. The [`source.Dockerfile`](/.dockerhub/source.Dockerfile) provides an advance example of avoiding ignored packages, or packages that are unnecessary as overlay dependencies.

## Builder Stage

A `builder` stage is then started to install external dependencies and compile the respective workspaces. Static CI dependencies are first installed before any later potential cache busting directives to optimize rebuilds. These include:

* [ccache](https://ccache.dev)
  * Compiler cache for speeding up recompilation
* [lcov](http://ltp.sourceforge.net/coverage/lcov.php)
  * Front-end for GCC's coverage testing tool gcov

### Install Dependencies

Dependencies for the underlay workspace are then installed using `rosdep` by pointing to the manifest files within the mirrored source directory copied from the `cacher` stage. This ensures the lengthy process of downloading, unpacking, and installing any external dependencies can be skipped using the docker build cache as long as the manifest files within the underlay remain unchanged. More info on rosdep can be found here:

* [rosdep](http://wiki.ros.org/rosdep)
  * CLI tool for installing system dependencies 

The sourcing of the ROS setup file is done to permit rosdep to find additional packages within the installed in ament index via `AMENT_PREFIX_PATH` environment variable, or potentially vendored packages unregistered in ament index via the legacy `ROS_PACKAGE_PATH` env. Cleanup of the apt list directory is done as a best practice in Docker to prevent from ever using stale apt list caches.

### Build Source

The underlay workspace is then built using `colcon` by first copying over the rest of the source files from the original `src` directory from the `cacher` stage. The colcon flag `--symlink-install` is used to avoid the duplication of files for smaller image sizes, while mixin argument is also parameterized as a Dockerfile `ARG` to programmatically switch between `debug` or `release` builds in CI. More info on colcon can be found here:

* [colcon](https://colcon.readthedocs.io)
  * CLI tool to build sets of software packages
* [colcon-mixin](https://github.com/colcon/colcon-mixin)
  * An extension to fetch and manage CLI mixins from repositories
* [colcon-mixin-repository](https://github.com/colcon/colcon-mixin-repository)
  * Repository of common colcon CLI mixins

The addition of the `ccache` mixin is used to pre-bake a warm ccache directory into the image as well as a pre-built underlay workspace. This will help speedup consecutive builds should later steps in the CI or maintainers have need to rebuild the underlay using the final image. The `console_direct` event handler is used to avoid CI timeout from inactive stdout for slower package builds while the `FAIL_ON_BUILD_FAILURE` env is used to control whether the docker image build should fail to complete upon encountering errors durring colcon build.

## Overlay Workspace

The overlay workspace is then set up in a similar manner where the same steps are repeated, only now sourcing the underlay setup file, and by building the overlay directory. The separation of underlay vs overlay workspace helps split caching of compilation across the two major points of change; that of external dependencies that change infrequently upon new releases vs local project source files that perpetually change during development. The overlay mixins are parameterized via `ARG` as well to allow the underlay and overlay to be independently configured by CI or local developers. This pattern can be repeated to chain together workspaces in one or multiple Dockerfiles; practically useful when working with a stack of related projects with deep recursive dependencies.

### Setup Entrypoint

The default entrypoint `ros_entrypoint.sh` inherited from the parent image is then updated to only source the top level overlay instead. The configured `ARG`s defining the paths used to the underlay and overlay are also exported to `ENV`s to persist in the final image as a developer convenience.

### Testing Overlay

The overlay may then be optionaly tested using the same additional mixins. The results of the test may also be used to optionaly fail the entire build; useful if the return code from `docker build` command itself is used as a primitive form of CI, or demonstrating to new contributors on how to locally test pull requests by invoking the colcon CLI.

## Buildkit

A difference for other Dockerfiles, not needing to be built by DockerHub and thus not limited in backwards compatibility, is the use of newer mount syntax options in buildkit, allowing for persistent caching of downloaded apt packages and ccache files across successful builds of the same Dockerfile. More info on buildkit can be found here: 

* [Build images with BuildKit](https://docs.docker.com/develop/develop-images/build_enhancements)
* [Buildkit repo](https://github.com/moby/buildkit)

The [`distro.Dockerfile`](/.dockerhub/distro.Dockerfile) provides once such example of this. More info on using mounts for caching data across docker builds can be found here:

* [cache apt packages](https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/experimental.md#example-cache-apt-packages)
  * avoid unnecessarily re-downloading the same packages over the network, even if the docker image layer cache for that `RUN` directive in the Dockerfile is busted

### Advanced Optimizations

With Buildkit's concurrent dependency resolution, multistage builds become parallelizable, assisting in shorter over all image build times. Granular expansion of the Directed Acyclic Graph (DAG) of workspace build steps into separate stages can be used to exploit this parallelism further, as well to maximize caching. This is exemplified in  [`source.Dockerfile`](/.dockerhub/source.Dockerfile). The figure bellow depicts how the multiple stages are composed to exploit the DAG of workspaces.

![pipeline](figs/multistage.svg)

This composition of stages follows a few basic principles:

* Enforce Determinism
  * Filter workspace source files down to what's essential
  * E.g. `cacher` stage prunes underlay packages irrelevant for overlays
* Maximize Caching
  * Leverage dependency build order when forming DAG
  * E.g. prevent `builder` stages from invalidating `depender` stages
* Optimize Layers
  * Lazily COPY and build FROM other stages as by-need
  * E.g. Avoid dependencies between `tester` stages to build in parallel

The table below is compares the finish build times between sequential (one stage one at a time) and multistage (many stages at once) builds, with and without caching (a warm and valid cache available).

|   | w/o Caching | w/ Caching |
|---|---|---|
| Sequential Build | 1h:22m:38s | 0h:49m:30s |
| Multistage Build | 0h:53m:49s | 0h:27m:44s |

For reference, Sequential Build without Caching is equivalent to building a dockerfile without the use of multistages nor Buildkit.
