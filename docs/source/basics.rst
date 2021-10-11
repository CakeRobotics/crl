======
Basics
======

In this page we'll to cover the basics of programming robots with Cake Robots Library.
It's going to be fun!
No special prior knowledge required.

Project Structure
======================

The smallest project contains only two files:

1. ``main.py``
----------------

This file logically describes the tasks that the robot must do, and doesn't deal much with hardware stuff.

.. admonition:: Example

    *You've already seen this code in the introduction page.*

    .. code-block:: Python

        import cake
        import time

        class MyRobot:
            def init(self):
                pass

            def loop(self):
                cake.wheels.set_speed(2)
                time.sleep(5)
                cake.wheels.set_speed(-2)
                time.sleep(5)

2. ``props.yaml``
--------------------

This file describes project information, including but not limited to: name,
dependencies, and most importantly, hardware.

.. note::
    If you are using the cloud platform (i.e. the website), you don't have to deal with this file.
    In that case, the website generates this file behind the scenes, based on the information you enter in the project settings page.

.. admonition:: Example

    .. code-block:: yaml

        name: An Example Robot Program
        pip_requirements:
            - numpy
        hardware:
            camera-1:
                type: camera/rgb
                location: 10, 0, 5 cm
                orientation: front-facing
                fov: 60 deg
                connection: usb

            wheels-1:
                type: wheels/ackermann
                locations:
                    front-axis: 3, 0, 0 cm
                    back-axis: 14, 0, 0 cm
                actuated-wheels: back
                max-torque: 10 N.cm
                max-steering: 20 deg
                driver: dca-440

Programming
===================

So ``main.py`` contains the main program. What are the rules about it?

As you might have already learned from the example, the main file
should define a class with two methods: ``init`` and ``loop``.

You might have also guessed what are these methods supposed to do, especially if you've
programmed an Arduino before. The ``init`` method should inititate the program. It is called only once,
when the robot boots. After ``init`` finishes, the ``loop`` function is then called again and again indefinitely.

.. note::

    If you are an advanced programmer, you might prefer more sophisticated programming patterns
    such as multithreaded and async, over the init-loop structure described here.
    Fortunately, it is totally possible to program the robot in other fashions.
    However, we prefered not to cover it here in the basics section.

    We even provide some useful Python decorators that you can use to make a function trigger upon
    events like a command from the cloud UI, a sensor threshold pass, a timer, etc.

The code that goes inside these functions is just the old regular Python,
but now you can write ``import cake`` and use the functionalities that it provides.
For example, to start moving forward you can write:

::

    cake.wheels.set_speed(2)

If you want to turn left:

::

    cake.wheels.set_steering_angle(20)

You can also read from sensors:

::

    img = cake.camera.current_image()

And these are just low-level stuff. How about:

::

    target_location = cake.navigation.current_location() + (20, 4)
    cake.navigation.go_to(target_location)

In the above code, the robot will move to the specified location,
and it will *not* hit any obstacles in the way!

A set of the features are cloud-based. We may open-source the cloud services in the future,
but right now they can only be used if you have a Cake account. For example:

::

    cake.log.config(cloud=True, key="alice/my-blue-robot")
    cake.log("Hi!")

To see all the functionalities, libraries, and how to use them, refer to the next section: :ref:`Full Reference <full_reference>`\ .

Building and Deploying
======================

Okay, I've created the files. How to make the robot actually run them?

First, ``cd`` to the directory that contains ``main.py`` and ``props.yaml``. Then run the following command:

::

    cake-bundler .

This will generate a ``Dockerfile``.
Now you can create a Docker image from it:

::

    docker build --tag=my-cool-robot:v1 .

.. note::

    If you want to perform Docker build on another machine (potentially the target robot),
    make sure you also copy your project files along with the Dockerfile.

Now you just need to run this Docker image on your robot. In the robot, run:

::

    docker run --rm --privileged my-cool-robot:v1

That's it! The robot should be moving around already.

.. note::

    With the cloud platform, the program Docker image is transparently built and
    transfered to the specified target robot.
    The platform also allows remote monitoring and management of the fleet.

Simulation
======================

A simulation build differs from a normal build in that,
the docker container doesn't work with hardware.
Instead, it communicates with a separate simulation container.
The communication is achieved via DDS on UDP.

In practice, this means that before runnnig the code, you should run a
cake simulation container:

::

    docker run --rm -t -p 8080:8080 --name=sim cakerobotics/sim

After running this, you should be able to see the simulation environment
at http://localhost:8080/

Then you can run bundler:

::

    cake-bundler --sim .

Build the image:

::

    docker build . --tag=my-cool-robot:v1-sim

Run the container:

::

    docker run --rm --network=container:sim my-cool-robot:v1-sim

Now the robot should be wandering in the simulation world!

.. note::

    You guessed it! The cloud service does that too, with a single click.
