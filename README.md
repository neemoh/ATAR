# Assisted Teleoperation with Augmented Reality

JUST STARTED WRITING THE README. LOTS TO BE WRITTEN SOON!

This repository contains the source code for the ATAR package. This ROS package can be used to design interactive augmented reality (or virtual reality) tasks using a stereo camera and display, master and slave manipulators. I have developed and tested this on a da Vinci Research Kit (DVRK) that comprises 5 manipulators (2 masters, 2 slaves and a camera arm), a stereo endoscope and stereo vision console. Tha package includes the code for the calibrations involved (Camera intrinsics, extrinsics, and arm to world) and a qt graphical user interface. THe simulated world includes rigid and soft dynamics simulation based on the Bullet Physics library and graphics are generated using the Visualization toolkit (VTK) and OpenGL.

![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_for_readme.png)

## Prerequisites
Apart from ROS (Tested with Kinetic), this package needs the following libraries:
* [Opencv](https://github.com/opencv) - For image conversions and calibrations
* [VTK](http://www.vtk.org/download/) - For the graphics
* [Bullet](http://bulletphysics.org/wordpress/) - For the dynamics

and the following packages from my github page:
* custom_conversions
* custom_msgs
* active_constraints

## To compile
First clone the following packages in your ros workspace and compile them:
```bash
git clone https://github.com/neemoh/custom_msgs.git
git clone https://github.com/neemoh/custom_conversions.git
git clone https://github.com/neemoh/active_constraints.git
catkin build
```
You should also be able to use catkin_make, but I suggest to use catkin tools. 

Next download VTK compile and install it. 
TODO: add details about shadows etc
Then download, compile and install the Bullet physics library. Make sure that you compile the Extra VHACD module and that its headers are in your system paths.


Download vtk: https://www.vtk.org/download/

modify the CMakeLists.txt in bullet3/Extras directory
 SUBDIRS( obj2sdf Serialize ConvexDecomposition HACD VHACD GIMPACTUtils )

cmake -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_EXTRAS=ON DINSTALL_EXTRA_LIBS=ON ..


Creating mesh objects with Blender:
You can use blender to easily create msh objects. When the object is ready follow these steps to make sure you have the correct units even if your mesh's dimensions are as small as a few millimiters:
convert it to mesh (alt-c)
go to the Scene tab and in the Units section selec Centimeters
Go to File-> Export and select Wavefront(.obj)
In the options set the scale as 0.01 and save.

