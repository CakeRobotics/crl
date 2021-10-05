.. _usage:

=======================
Usage
=======================

There are three options for using Cake Robots Library:

1.
    **The Cloud Service (Recommended)**

    This is the easiest option and you should use it if you're not a software expert.
    With this option, you don't have to install anything on your computer.
    You only need to create an account (`here <https://cloud.cakerobotics.com/auth/login>`_)
    and you'll get a web page in which you can write code, simulate it, and push it to actual robots.

    The cloud service has some other cool features too, e.g. device management, logging, cloud processing,
    shared knowledge, etc.
    Some of these perks are paid. By using them you'll also support our team.

    The downside with this approach is that some of our cloud services are not open-source.
    If you want a purely open-source option, use other options.

2.
    **Docker Bundler**

    With this option, you need to have a Linux machine with Docker installed.
    The first step is to write the robot program and save it on your computer.
    In order to simulate the program or push it to a robot,
    you'll use our tool called *Bundler*.
    This tool generates a Dockerfile from the code, which
    contains instructions to install CRL and requirements like ROS,
    ROS packages, Python packages, etc.
    This file can be used to build images and push them to robots or run
    them locally alongside a simulation container. Detailed instructions
    can be found in the `project page <https://github.com/cakerobotics/bundler>`_.

3.
    **Bare-metal**

    This option is only recommended if you are working on the Cake project itself.
    In this case, you'll use CRL as a Python library but you have to
    install all the requirements manually. See :ref:`Bare-metal Installation <bare_metal>`
    guide for more information.
