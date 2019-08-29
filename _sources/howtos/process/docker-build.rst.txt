.. _docker_build:

Use Docker to build this Sphinx project
#######################################

.. warning::

   This is a work in progress and will only work if Docker is already
   installed.

.. note::

    Permission to share the C drive must be reset every time you change
    your windows password.

If you'd like to build the documentation but don't want to install all of the 
dependencies on your system, consider using a docker container to do it. This 
is very similar to what most automated build systems do. 

You'll need to be in the same directory as the included 
:file:`/scripts/docker/Dockerfile` to build the image. Once you're there,
simply run this command:

.. code-block:: bash

   sudo docker build -t sphinx-builder . #Linux Bash

   docker build -t sphinx-builder . #Windows Powershell

.. note::

   ``sphinx-builder`` is the name of the image that gets created. 

Once the image has successfully built (docker not being configured for proxy
is a common gotcha [#f1]_ [#f2]_), you can build the project. You'll need to be in the same
directory as the :file:`conf.py` to successfully build the documentation, so
do that first, then run the following command (remember sphinx-builder is the
name we gave the image in the previous command, so if you named it something
else use that instead):

.. code-block:: bash

   sudo docker  run --rm  -v `pwd`:/home/sphinx -w /home/sphinx -i -t  sphinx-builder make html #Linux Bash

   docker  run --rm  -v ${pwd}:/home/sphinx -w /home/sphinx -i -t  sphinx-builder make html #Windows Powershell

``pwd`` is shorthand for "present working directory," so the
command maps (-v) the ``pwd`` of the host environment to the :file:`/home/sphinx` directory in the container, then sets it to the working directory
(-w). And finally, we run the container by name and pass in the
:command:`make html` command to build the documentation. The ``--rm`` option
ensures that docker cleans up after it's finished by deleting the container.

.. rubric:: Footnotes

.. [#f1] Configuring proxy for Docker for Windows: https://docs.docker.com/docker-for-windows/#proxies
.. [#f2] Configuring proxy for DockerCE in Linux: https://docs.docker.com/config/daemon/systemd/#httphttps-proxy