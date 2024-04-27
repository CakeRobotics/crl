=============
Introduction
=============

What is Cake Robot Library?
===========================

Cake Robot Library (CRL) is a Python package that provides
an easy and efficient framework for robot programming.
It is also the core of `Cake Robotics <https://cakerobotics.com>`_ cloud platform.

CRL wraps ROS functionalities and integrates them with opt-in Cake cloud services. From the outside, CRL is
just a set of declarative classes and functions that can be used to create
maintainable robot programs.

An Example
==========

This simple example can illustrate how CRL helps you write robot code.
Can you guess what happens if we could run this code in a robot?

.. code-block:: Python

    import cake
    import time

    robot = cake.Robot()
    
    while True:
        robot.wheels.set_speed(2)
        time.sleep(5)
        robot.wheels.set_speed(-2)
        time.sleep(5)

You're right. The robot will simply move back and forth indefinitely.

In the real world, you can't just write a code like that and push
it to a robot. It is true that the code suffices to define the *task*
but you also need a lot of code to manage *hardware*.

But do you?

With CRL, you don't. In fact, that code is a real program and you can run it
as it is, using CRL.

Of course, you still have to briefly *define* your hardware for CRL (we'll catch on that later),
but you no longer have to *manage* hardware in your code.
Your code will only describe the task, and it will just work.
Piece of *cake*!
