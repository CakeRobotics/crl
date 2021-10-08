.. _bare_metal:

=======================
Bare-metal Installation
=======================

.. attention::
    This installation method is not recommended for general use. See :ref:`Usage <usage>` for better alternatives.

This page describes steps to use Cake Robot Library by
installing it as a Python library.
It also describes steps to install runtime requirements manually.

This installation method is only recommended for CRL core
developers. This page also serves as a starting point to
understand how bundler works.

Platforms
==============

The instructions are for both Ubuntu 20.04 and Archlinux.
The Ubuntu setup is similar to what bundler does,
and is considered more stable than that of Archlinux.

Fixed Installation Steps
==========================

These steps are fixed, in contrast to the steps described in the next section
that vary depending on your project.

1.  Install Cake Robot Library

    ::

        pip3 install crl

    .. note::

        To install from source, run::

            git clone https://github.com/cakerobotics/crl
            cd crl
            pip3 install -e .

    .. note::

        It is good practice to use a virtual environment, but watch for the following caveats:

        1.  You should create venv only after sourcing ROS base layer, i.e.::

               source /opt/ros/foxy/setup.bash
               python -m venv venv --system-site-packages

            Also note the ``--system-site-packages`` argument. This allows venv to
            access base ROS libraries like rclpy.

        2.  You need to manually add the ``source /opt/ros/foxy/setup.bash`` to
            the beginning of your ``venv/bin/activate`` script.

        3.  You may need extra configuration for your IDE to work correctly. For
            example, if you use VSCode with PyLint and PyRight, you probably want
            to add these lines to ``.vscode/settings.json``::

                "python.pythonPath": "venv/bin/python",
                "python.analysis.extraPaths": ["/opt/ros2/foxy/lib/python3.9/site-packages"],
                "python.terminal.activateEnvInCurrentTerminal": true,

            and this line to ``.env``::

                PYTHONPATH=/opt/ros2/foxy/lib/python3.9/site-packages

2.  Install ROS 2 (Rolling)

    .. tab:: Ubuntu

        Official instructions for Ubuntu are available `here <https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Binary.html>`_\ .

    .. tab:: Archlinux

        ::

            yay -S ros2-git
            printf "\nexport ROS_DOMAIN_ID=42\nsource /opt/ros2/foxy/setup.bash\n" >> ~/.bashrc

3.  Install extra ROS 2 packages

    .. tab:: Ubuntu

        ::

            apt-get install ?

    .. tab:: Archlinux

        ::

            yay -S ?

    .. tab:: Other

        Full list of these packages:

        - gw

Dynamic Installation Steps
============================

These steps depend on the requirements of the specific project you want to run.
Therefore, you may need to come back to this section as your project evolves.

1.  Install PyPi packages (e.g. numpy, ...) that you have used in your project code.

    ::

        pip3 install <packages>

    .. admonition:: Side-note

        Bundler detects the list of required packages by reading the key ``pip_requirements``
        from the props file.

Running the Robot Program
===============================

1.  Navigate to the directory of your project:

    ::

        cd my-first-cake-project


2.  Run the following command:

    ::

        sudo cake

    This will start the robot described in your ``main.py``.

    .. note::

        If just you want to run the program in a simulation environment:

        - Add ``--sim`` argument to ``cake`` command. This will tell cake to
          disable hardware operations, and communicate with a simulation
          container for actuator/sensor operations.

        - Before running, make sure that Gazebo is installed.

        - You can drop ``sudo`` as hardware operations are disabled.
