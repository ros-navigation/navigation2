# CircleCI Documentation

CircleCI is a service used to build and test the project's codebase to catch regressions as well as to check pull request submissions. Using continuous integration helps maintainers sustain high code quality while providing external contributors a well defined evaluation method with which to polish contributions, without even needing to stand up entire development environment locally. More info on CircleCI can be found here:

* [CircleCI](https://circleci.com/)
* [Navigation2 on CircleCI](https://circleci.com/gh/ros-planning/navigation2)

For this particular CI, Docker is used to build and test the project within containers derived from images pulled from DockerHub, bootstrapping the CI with a development environment including pre configured dependencies and warm workspace build caches. View the accompanying DockerFile and DockerHub documentation for more info on this accompanying CI setup.

* [Dockerfile](dockerfile.md)
* [DockerHub](dockerhub.md)

CircleCI is configured via the [config.yml](/.circleci/config.yml) yaml file within the `.circleci/` folder at the root of the GitHub repo.  The config file for this project is self-contained and thus densely structured, yet written in a functional style to remain DRY and modular. This keeps it easily generalizable for other ROS packages or for adjusting overlayed workspaces. Given the anchors and references in yaml, it's consequently best approached by reading the config file from bottom to top in order of abstraction hierarchy. Further references on configuring CircleCI, such as syntax and structure can be found here:

* [Writing YAML](https://circleci.com/docs/2.0/writing-yaml)
* [Configuring CircleCI](https://circleci.com/docs/2.0/configuration-reference)

## Workflows

The CI config consists of two main [workflows](https://circleci.com/docs/2.0/configuration-reference/#workflows). One workflow is for checking PR events, essentially triggered by any pushed commits towards the repo, by building and testing the project both in release and debug mode for accurate performance benchmarking or generating test coverage results. The second is cron scheduled to check the main repo branches daily, while additionally testing a matrix of supported RMW vendors not normally tested within day-to-day PRs. This helps prioritize CI to quickly check new contributions, while simultaneously keeping tabs on the health of existing code.

The order in which jobs are executed is conditional upon the completion of those is [requares](https://circleci.com/docs/2.0/configuration-reference/#requires), forming a conventional direct acyclic graph of dependent jobs. Unrelated jobs may of course be parallelized in the CI pipeline; so by splitting the build and test jobs in two, multiple jobs such as the matrix of RMW tests may commence as soon as the dependent build job completes, avoiding unnecessarily re-building the same codebase for each RMW vendor.

## Jobs

The list of [jobs](https://circleci.com/docs/2.0/configuration-reference/#jobs) referenced within the workflows consist of either release or debug build/test jobs. These jobs are largely similar with the exception of using different executors, and that debug test jobs include additional command steps for reporting code coverage analytics. The build jobs simply checkout the project, set up the build environment, and compile the project. The test jobs in turn pick up from where the build job left off by restoring the build, and then running the test on that build.

In addition to enabling the parallelization of different types of test jobs, the bifurcation build and test jobs enables efficiently controlling the [parallelism](https://circleci.com/docs/2.0/configuration-reference/#parallelism) within each job itself, leveraged later for test splitting. However, given the workspace caching and principled builds, parallelism within build jobs is not as easily applied nor justified, thus parallelism here is applied only to testing.

## Executors

Two different [executors](https://circleci.com/docs/2.0/configuration-reference/#executors-requires-version-21) are used to define the environment in which the steps of a job will be run, one for debug and one for release builds. Given only the release executor is used in testing the full matrix of RMW vendors, the docker images for the debug executor merely include the default RMW; sparing debug jobs the time in compiling unused RMW vendors in debug testing.

The executors also differ by a few environment variables. These include appropriate mixin flags for the workspace builds and an additional nonce string, unique to that executor, used in deriving the hash key for saving and restoring caches. This prevents cache key collisions or cross talk between parallel jobs using different executors in the same workflow.

## Commands

To reuse sequences of steps across multiple jobs, a number of [commands](https://circleci.com/docs/2.0/configuration-reference/#commands-requires-version-21) are defined. In addition to a few common commands for abstracting lower level tasks as a function, a number of higher level commands are also recursively composed of smaller formulaic steps.

When checking our source code from the repo, additional pre and post checkout steps are performed to prepare the file system. When setting up dependencies, the build is prepared much the same way as in the project Dockerfile, by first setting up the underlay and then the overlay workspace. Additional steps are included for restoring and resetting previous ccache files and statistics, including a few diagnostic steps that can be used for debugging the CI itself, e.g. investigating as to why the hit rate for ccache may be abnormally low. Once the overlay setup is finished, more ccache diagnostics and files are saved for inspection or later restoration. A restore build command is also defined for test jobs, operating under the condition that a workspace with the same checksum can be restored from a previous build job in the workflow. Additional commands are also defined for steps in testing overlay workspace and reporting coverage results.

## References

Per the YAML syntax, a reference must proceed any anchor that it links back to. As a formality, all references are merely prefaced and organized under the key titled `references`. 

### Common Environment

For environment variables common among the executors, workspace paths and specific config options are listed here. While the workspace paths are parameterized to avoid hardcoding paths in common command functions, they may also be hardcoded as fields among a few high level steps given a limitation of the config syntax. The additional config options help optimize our project build given CI resource limits:

* Capping ccache size/location given CI storage limits, speeding up cache restoration
* Limiting parallel make and linker jobs as to avoid exhausting all un-allocated RAM
* Further adjustments for changing robust test behavior and sequential test stdout.

### Common Commands

Common commands for low level, repeated, and formulaic tasks are defined for saving and restoring CI caches, as well as for installing, building, and testing workspaces.

#### Caching

Multiple caches may be saved and restored over the completion of a single workflow. To appropriately save and restore cache in a deterministic manner, these steps are abstracted as commands. For this project, caching is done with respect to a given workspace. As such, a specified string and checksum from the workspace are combined with workflow specifics to ensure the [restored cache](https://circleci.com/docs/2.0/configuration-reference/#restore_cache) is uniquely identifiable and won't collide with other concurrent PRs.

For saving a cache, the command is similar, aside from specifying the path to directory or file to be stored in the [saved cache](https://circleci.com/docs/2.0/configuration-reference/#save_cache). Given CI cache are conventionally read only, meaning a cache key can not be reused or updated to point to a newer cache, the current unix epoch is appended to the cache key to ensure key uniqueness. Because CircleCI key lookup behavior for cache restoration is performed via the most recent longest matching prefix, the latest matching cache is always restored.

These workspace checksums are uploaded as [stored artifacts](https://circleci.com/docs/2.0/configuration-reference/#store_artifacts) throughout other commands to help introspect to debug caching behavior when needed.

#### Building

For installing and building workspaces, the process resembles that within the project Dockerfile. Additional bookkeeping is performed to update the workspace checksum file by piping stdout that deterministically describes the state of the build environment. This is done by seeding from the checksum of the underlay workspace and then appending info about source code checked out into the overlay workspace, as well as the list of required dependencies installed. When setting up the workspace, this checksum will first be used to check if the workspace can be restored from a prior workflow build. If the source code or required dependencies change, resulting in a missed cache hit, the unfinished workspace is then built. If the workspace build is successful then it will be cached. Regardless however, the build logs are always uploaded as stored artifacts for review or debugging. The odd shuffling of symbolic directories is done as a workaround given a limitation of the S3 SDK: 

* [Failing to upload artifacts from symbolic link directories](https://discuss.circleci.com/t/failing-to-upload-artifacts-from-symbolic-link-directories/28000)

#### Testing

For testing workspaces, the list of packages within the workspace are [tested in parallel](https://circleci.com/docs/2.0/parallelism-faster-jobs/) across the number of replicated containers for the given test job as denoted by the `parallelism` option. Here packages are split by anticipated test timing; the heuristic derived from the reported duration and classname of prior recent test results. The logs and results from the tests are then always [reported](https://circleci.com/docs/2.0/configuration-reference/#store_test_results) and uploaded.

### Checkout

Checking out code consists of three stages, including pre and post checkout steps. To simplify the formulaic common commands above, the pre-checkout step replicates a synthetic workspace from the installed ROS distro directory by symbolically linking an install folder, and bootstrapping the checksum from the timestamp of an expected file in ROS docker images. This is a measure to ensure if the nightly docker image is changed/rebuilt, then all CI caches should also be busted. Ideally, the docker image sha/hash should be used for this instead, but as of writing there does not seem to be a reliable method for acquiring this image digest from within the same derived container:

* [Introspect image info from inside docker executor](https://discuss.circleci.com/t/introspect-image-info-from-inside-docker-executor/31620)

The overlay workspace is then cleaned prior to checking out the project. The post checkout step simply checks to see if the underlay has changed to determine whether it should also be cleaned, recloned, and thus rebuilt as well.

### Workspaces

The rest of the high level commands serve as references to repeatedly define workspace specific commands, such as install, building and testing either the underlay or overlay workspace. Some point of note however include: 

* The CI cache for ccache is intentionally linked with the underlay rather than the overlay workspace
  * so that consecutive commits to the same PR are more likely to retain a warm and recent ccache
* CCache Stats is intentionally used to zero stats before building a workspace
  * so the next consecutive run of the CCache Stats reflects only upon that given workspace
* Restore workspace command intentionally sets the `build` filed to `false`
  * to avoid unnecessary duplication of the same workspace cache and build logs

### Code Coverage

A custom script within the repo is reused to collect and post process the generated code coverage results from debug test jobs. This always runs regardless if the tests fail so that failed code coverage reports may still be reviewed. The final report is uploaded to CodeCov via the third party orb. Note that the use of a third party orb requires the enabling of a permission in CircleCI settings for the repo. So if you would like to upload code coverage results for your own fork, you'll have to explicitly allow it.

* [CodeCov Orb](https://circleci.com/orbs/registry/orb/codecov/codecov)
* [Orbs Introduction](https://circleci.com/docs/2.0/orb-intro)
