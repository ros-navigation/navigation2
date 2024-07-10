# Dev Containers

This folder contains the necessary files to build and run development containers for the Navigation2 project. The containers are based on the ROS 2 Rolling distribution and include all necessary dependencies to build and run the Navigation2 stack.

## Quick Start

To get started, follow the instructions below.

### Prerequisites

First, ensure your using a recent enough version of Docker Engine that supports [BuildKit](https://docs.docker.com/build/buildkit/). If you plan on running robot simulations locally, Hardware Acceleration for sensor raytracing and 3D rendering is also recommended. While other compatible devcontainer tools may be used, Visual Studio Code is recommended for simplicity.

#### System Software
- [Docker Engine](https://docs.docker.com/engine/install/)
  - https://get.docker.com - simple universal install script
  - [Linux post-installation](https://docs.docker.com/engine/install/linux-postinstall/) - manage Docker as a non-root user
- [Git LFS](https://git-lfs.github.com/) - optional for managing large assets
  - Use for version controlling media such as figures
- [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) - optional for enabling Hardware Acceleration
  - [Installing the Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) - only necessary host running Docker Engine

#### Development Tools
- [Visual Studio Code](https://code.visualstudio.com/) - alternative to Dev Containers CLI
  - [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) - via SSH, Tunnels, Containers, WSL
    - [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) - specific to just Containers
  - [Docker extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) - for introspecting Docker daemon
  - [Using SSH keys](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials#_using-ssh-keys) - sharing Git credentials with container
- [Dev Container CLI](https://github.com/devcontainers/cli) - alternative to VSCode
  - [Installation via NPM](https://github.com/devcontainers/cli?tab=readme-ov-file#npm-install) - for custom install setup
  - [Installation via VSCode](https://code.visualstudio.com/docs/devcontainers/devcontainer-cli) - for simple install setup
    - Note: CLI installed via VSCode is warped but bugged, install via NPM is recommended for now
    - https://github.com/devcontainers/cli/issues/799
- [GitHub CLI](https://cli.github.com/) - optional for interacting with CI Workflows
    - [Installation](https://github.com/cli/cli#installation) - specifically [for Linux](https://github.com/cli/cli/blob/trunk/docs/install_linux.md)
    - [Configuration](https://cli.github.com/manual/) - login authentication and setup

### Environment Setup

Once you've setup your ssh keys for GitHub and account credentials for the CLI, you can add your private SSH key to the SSH agent if you'd like to use your local credentials for Git operations inside the dev container:

```shell
# Add your SSH key to the SSH agent
ssh-add ~/.ssh/<github-key>
```

You may also configure Docker to use those credentials when interacting with the GitHub Container Registry (GHCR). This setup is optional, but can be useful to avoid rate limiting issues when pulling images from a popular public IP address. This can be done by using docker login with the GitHub CLI:

```shell
# Login to GitHub Container Registry
gh auth token | docker login ghcr.io --username <github-username> --password-stdin
```

### Submodule Setup

Next, recursively clone this repository and included submodules.

```shell
# Clone the repository and submodules
git clone --recurse-submodules -j8 \
  git@github.com:ros-navigation/navigation2.git

# Change into the repository directory
cd navigation2

# Configure the local git include path
git config --local include.path ../.gitconfig
```

### Images Setup

To create the base image for the container, one can either build the image locally, or pull CI image layers cached remotely. If you need to modify the base image during the development process, such as adding or removing dependencies, you can build images locally before opening any PRs. If you want to evaluate modifications to the base image during the review process, you could then also pull pre-built CI images from an opened PR.

#### Building Locally

Building locally leverages various caching strategies to speedup subsequent docker rebuilds, such as caching apt package downloads, incremental build artifacts, and multi-stage optimizations for image layer reuse. You can either let the dev container's initialization lifecycle script build the image for you, or you can pre-build the image manually:

```shell
# Bake the dever image tag as a test
docker buildx bake dever

# Run container from image as a test
docker run -it --rm nav2:tooler bash
```

#### Pulling Remotely

Alternatively, you can pull the CI image layers from the GitHub Container Registry (GHCR) to bootstrap the dev container. While this method may be faster or slower depending on your local network connection, it remains particularly useful for downloading pre-built colcon workspaces built by CI and baked into the debugger stage, avoiding the need to build the image layers or re-compile a PR locally. Simply use the `DEV_FROM_STAGE` environment variable to shortcut the build process to use a given reference image, as commented from the initialization script:

```shell
REFERENCE_IMAGE=ghcr.io/ros-navigation/navigation2:main-debugger
docker pull $REFERENCE_IMAGE
export DEV_FROM_STAGE=$REFERENCE_IMAGE
```

> Note: you may want to clean or create new named volumes as defined in the dev container config, given cached volumes only initialize from their first attached container. I.e. to switch between different PRs, you'll want to avoid using a named volume that was seeded from a different/older docker image.

### Launching Development Containers

Note: using Dev Containers from a remote host is also possible:

-  [Open a folder on a remote SSH host in a container](https://code.visualstudio.com/docs/devcontainers/containers#_open-a-folder-on-a-remote-ssh-host-in-a-container)
-  [Open a folder on a remote Tunnel host in a container](https://code.visualstudio.com/docs/devcontainers/containers#_open-a-folder-on-a-remote-tunnel-host-in-a-container)

#### Visual Studio Code
Finally, open VSCode and use the Remote Containers extension:

```shell
code .
# Press Ctrl+Shift+P to open the Command Palette
# Type and select `Dev Containers: Reopen in Container`
```

#### Dev Containers CLI
Alternatively, use the CLI to bring up and exec into the Dev Container:

```shell
devcontainer up --workspace-folder .
devcontainer exec --workspace-folder . bash
```

### Verifying Development Containers

To verify the dev container is setup correctly, i.e. hardware acceleration and display forwarding is working as expected, you can run simulation examples to check:

```shell
# Included alias to source overlay workspace
sows
# Launch simulation example with GUIs enabled
ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False
```

## Further Reading and Concepts

Afterwards, you may want to further familiarize yourself more with the following topics:

- Git Submodules
  - https://git-scm.com/book/en/Git-Tools-Submodules
  - https://git-scm.com/docs/git-submodule
- Docker
  - Multi-stage
    - https://docs.docker.com/build/building/multi-stage/
  - BuildKit
    - https://docs.docker.com/build/buildkit/
  - Bake
    - https://docs.docker.com/build/bake/
- Development Containers
  - https://navigation.ros.org/development_guides/devcontainer_docs/index.html
  - https://containers.dev/
  - https://code.visualstudio.com/docs/devcontainers/containers
