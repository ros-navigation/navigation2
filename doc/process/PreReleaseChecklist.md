# Pre Release Checklist

This documents the steps to be taken prior to making a new release of the
nav2 stack.

## Summary
1. `Ensure all dependencies are listed in the package.xml files` by doing a
build of all of ROS2, dependencies, and navigation 2 in one workspace.

2. `Ensure all dependencies are released.` by using rosdep to pull in dependencies instead of building them ourselves.

3. `Ensure the test suite passes`

## Detailed Steps

### Ensure all dependencies are listed in the package.xml files

We want to ensure that every package has a complete list of its dependencies
in the `package.xml` file. This can be done by not sourcing any ros `setup.bash` files. Instead we need to build everything as one big repo.

There is a docker file to do that, so run

```bash
sudo docker build -t nav2:full_ros_build --build-arg ROS2_BRANCH=dashing --build-arg http_proxy=http://myproxy.example.com:80  --build-arg https_proxy=http://myproxy.example.com:80 -f Dockerfile.full_ros_build ./
```

ROS2_BRANCH should be the release you are targeting or just `main` if you want
to compare against ROS2 main.

### Ensure all dependencies are released.

We want to ensure the correct version of all our dependencies have been released
to the branch we are targeting. To do that, we skip the
`underlay.repos` install step and rely solely on rosdep to install
everything.

There is a dockerfile to do that as well, so run

```bash
sudo docker build -t nav2:rosdep_only_build --build-arg ROS2_BRANCH=dashing --build-arg http_proxy=http://myproxy.example.com:80  --build-arg https_proxy=http://myproxy.example.com:80 -f Dockerfile.release_branch ./
```

As before, ROS2_BRANCH is the branch you are targeting. In this case, there is
no main option. We can only run this dockerfile against a set of released
packages.

### Ensure the test suite passes

Ensure the test suite passes in one of the docker images you just built.

#### Crystal

For the `crystal` release, run

```bash
sudo docker run nav2:crystal colcon test
```

#### Dashing and newer

For newer releases, run

```bash
sudo docker run nav2:crystal src/navigation2/tools/run_test_suite.bash
```
