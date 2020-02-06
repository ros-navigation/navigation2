.. _docker:

Navigation2 Docker
******************

There are 2 options for docker with Navigation2:
building a container and using the DockerHub container.

Building Docker Container
-------------------------

To build an image from the Dockerfile in the navigation2 folder:
First, clone the repo to your local system (or see Building the source above)

.. code:: bash

  sudo docker build -t nav2/latest .

If proxies are needed:

.. code:: bash

  sudo docker build -t nav2/latest --build-arg http_proxy=http://proxy.my.com:### --build-arg https_proxy=http://proxy.my.com:### .

Note: You may also need to configure your docker for DNS to work. See article here for details: https://development.robinwinslow.uk/2016/06/23/fix-docker-networking-dns/

Using DockerHub Container
-------------------------

We allow for you to pull the latest docker image from the master branch at any time. As new releases and tags are made, docker containers on docker hub will be versioned as well to chose from.

.. code:: bash

  sudo docker pull rosplanning/navigation2:latest
