# Assisted Teleoperation with Augmented Reality

JUST STARTED WRITING THE README. LOTS TO BE WRITTEN SOON!

This repository contains the source code for the ATAR package. This ROS package can be used to design interactive augmented reality (or virtual reality) tasks using a stereo camera and display, master and slave manipulators. I have developed and tested this on a da Vinci Research Kit (DVRK) that comprises 5 manipulators (2 masters, 2 slaves and a camera arm), a stereo endoscope and stereo vision console. Tha package includes the code for the calibrations involved (Camera intrinsics, extrinsics, and arm to world) and a qt graphical user interface. THe simulated world includes rigid and soft dynamics simulation based on the Bullet Physics library and graphics are generated using the Visualization toolkit (VTK) and OpenGL.

## Requirements
Apart from ROS (Tested with Kinetic), this package needs the following libraries:
* [Opencv](https://github.com/opencv) - For image conversions and calibrations
* [VTK](http://www.vtk.org/download/) - For the graphics
* [Bullet](http://bulletphysics.org/wordpress/) - For the dynamics

and the following packages from my github page:
* custom_conversions
* custom_msgs
* ActiveConstraints

## To compile
First clone the following packages in your ros workspace and compile them:
```bash
git clone https://github.com/neemoh/custom_msgs.git
git clone https://github.com/neemoh/custom_conversions.git
git clone https://github.com/neemoh/active_constraints.git
catkin build
```

Next download VTK compile and install it. [TODO] add details about shadows etc
Then download, compile and install the Bullet physics library. Make sure that you compile the Extra VHACD module and that its headers are in your system paths.
