.. _full_reference:

================
Full Reference
================

Props File
================

Props can be provided in three ways:

1. **Using the cloud service UI:** In this case, you don't need to read
   this section because the user interface is self-explaining. The file will be
   generated from your settings, and will be passed to the appropriate services
   transparently.

2. **As a static YAML or JSON file:** This is the recommended way if you're not
   using the cloud service. In this method, a file named ``props.yaml`` or
   ``props.json`` is used to specify project props. In both cases, a props
   object is described. The schema for the props object is specified in the
   following lines. YAML is generally considered better than JSON for the files
   that are to be manually maintained.

3. **As a python Dict object**: This method is more flexible, as the dict is
dynamically generated. However, you probably shouldn't use it as it makes the
project more complicated. Also, this variant can't be used with *bundler*.

Props file contains keys and values that are described below:

- ``name``: Name of the project
- ``author``: Author of the project *(optional)*
- ``pip_requirements``: PyPI dependencies *(required if using bundler)*
- ``hardware``: Hardware of the robot. See the section below.

Hardware Props
==============

Hardware props go under ``hardware`` key of the props file. Each set of hardware
(e.g. wheels, camera, lidar)
is denoted by a unique **key** and a **value object**. The latter defines
properties of robot hardware, and is passed to the corresponding hardware module.
Therefore, the schema of **value object** depends on the type of hardware that is
to be defined. In fact, the only key that is fixed in the **value object** is
``type`` itself, which is used to determine the parser class of the hardware
part.

To understand the schema for each type of hardware, see *Hardware Modules*
documentation. TODO: add hyperlink.

.. admonition:: Example props file

    .. code-block:: yaml

        name: An Example Robot Program
        pip_requirements:
            - numpy
        hardware:
            camera-1:
                type: camera
                variant: rgb
                location: 10, 0, 5 cm
                orientation: front-facing
                fov: 60 deg
                connection: usb

            wheels-1:
                type: wheels
                steering: ackermann
                locations:
                    front-axis: 3, 0, 0 cm
                    back-axis: 14, 0, 0 cm
                actuated-wheels: back
                max-torque: 10 N.cm
                max-steering: 20 deg
                driver: dca-440

Robot object
================

robot.wheels
----------------

This member controls the wheels. The member functions of this object are
described below:

.. autoclass:: cake.modules.hardware.wheels.Wheels.Wheels
    :members:
