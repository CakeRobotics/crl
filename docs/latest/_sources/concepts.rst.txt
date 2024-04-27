=====================
Engineering Concepts
=====================

This section describes the internals of Cake Robotics Library.
You don't need this knowledge if you just want to *use*
Cake---the same way you don't need to know how a piston engine works in order to
ride a car.

Mission Statement
======================
Cake Robotics, as a set of projects and services, aims to make it easy and
efficient to program robots in the lower end of production scale spectrum.

The observation that clarifies this mission statement is that in small-scale
robotics projects, *development costs* are the economic bottleneck---in
contrast to manufacturing costs in large-scale projects.

CRL is the core framework that our users interact with in their codes.
Therefore, the role of CRL is to provide the developer experience required
to fulfill the mission of Cake Robotics project as a whole.

Design Philosophy
======================
Ease of Use
-------------
It is decided that in CRL we put ease-of-use over almost everything else.
This means:

* Only minimal software engineering knowledge should be required to use the library.
* Only minimal configuration should be required to use the library.

Naturaly, this philosophy tends to produce opinioated software.
To counter this, we try to keep the opinioated design in the outer layers of the
library.
Therefore, CRL would still be usable by
the power users---especially from the open-source community---who prefer
to use software in more sophisticated configurations.

Architecture
======================
In the basic form, every Cake project consists of a main.py file and a props.yaml (or
.json) file. CRL comes with a command ``cake run <path_to_project_directory>``
that can be used to run the robot program on the robot.

In other words, Cake project is typically runs in a *managed* fashion: User code
goes to a file like ``main.py`` but the file has no ``main()`` function. It contains
``init()`` and ``loop()`` functions (much like Arduino), and the manager script,
i.e. ``cake run`` reads this file and executes it.

This allows the host script to scan the props file and make the required
configurations before running the user code. On the flip side, this restricts
user code to a predefined structure. That's why there's an alternative, meant
for advanced users, in which the the main function is written by user but they
should call ``robot.init_from_props(props)`` function before running any other cake code. This
method is what ``cake run`` does internally. Therefore, no matter if the user
uses ``cake run`` or manually calls init in a self-contained script, behavior of
cake modules is the same.

This is a good example of being easy to use for general users, without limiting
advanced users. It is notable that for sake simplicity and consistency, the
latter way (i.e. calling init manually) is not emphasized in the user
documentation and tutorials.

With either usage, the user imports cake functionalities using ``from cake
import Robot`` or ``robot`` command in
their script. Then, commands like ``robot.wheels_1.set_velocity(2)`` will work
without any extra configurations. This is because, upon initialization, cake
reads the props object and makes the required configurations.

For example, if in the initialization step Cake detects a ``wheels_1`` key with
``type: wheels`` in the props, ``robot.wheels`` will be created by
instantiating an internal ``Wheels`` object. The object itself may run a managed
ROS node, used for tasks like broadcast user commands, e.g.
``robot.wheels.set_velociry``, to a ROS topic. The object may also run other
ROS nodes, for example, drivers that convert these messages to hardware signals.
This is why every object has a ``.get_dependencies()`` member function, used in
automatic bundling process, which returns a list of ROS packages, etc. that
should be available before the object can work under the specified
configuration.

This design allows for hierarchy too. For example, a ``Wheels`` object may
implement a factory pattern in which different drived classes will be
instantiated based upon the props file. Composition is possible too. For
example, dependencies of a class can be determined by merging the dependencies
of the objects within that class.

It's also notable that ``robot.wheels`` will fail if ``wheels_1`` is not defined
in the props file. Also, if more that one hardware with ``wheels`` type are
defined, the call will cause all the wheels to perform the action, unless
specified otherwise using an optional argument. So generally, ``robot`` object
serves as a set of instantiated objects (or lazily instantiated objects).
The end user does not have to deal with object oriented paradigms. Also, some of
the functionalities are provided as lazy objects.

Codebase Structure
=======================

TODO
