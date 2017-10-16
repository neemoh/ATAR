# Assisted Teleoperation with Augmented Reality

JUST STARTED WRITING THE README. LOTS TO BE WRITTEN SOON!

This repository contains the source code for the ATAR package. This ROS package can be used to design interactive augmented reality (or virtual reality) tasks using a stereo camera and display, master and slave manipulators. I have developed and tested this on a da Vinci Research Kit (DVRK) that comprises 5 manipulators (2 masters, 2 slaves and a camera arm), a stereo endoscope and stereo vision console. Tha package includes the code for the calibrations involved (Camera intrinsics, extrinsics, and arm to world) and a qt graphical user interface. THe simulated world includes rigid and soft dynamics simulation based on the Bullet Physics library and graphics are generated using the Visualization toolkit (VTK) and OpenGL.

![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_for_readme.png)

## Prerequisites
Apart from ROS (Tested with Kinetic), this package needs the following libraries:
* [VTK](http://www.vtk.org/download/) - For the graphics
* [Bullet](http://bulletphysics.org/wordpress/) - For the dynamics

and the following packages from my github page:
* custom_conversions
* custom_msgs
* active_constraints

Opencv and qt libraries are needed too but these should come with ROS.

## To build
You should be able to use catkin_make, but I suggest to use catkin tools:

``` 
sudo apt-get install python-catkin-tools
```
Clone the following packages in your catkin workspace src directory and 
compile them. 


```bash
git clone https://github.com/neemoh/custom_msgs.git
git clone https://github.com/neemoh/custom_conversions.git
git clone https://github.com/neemoh/active_constraints.git
catkin build
```

Next download VTK from https://www.vtk.org/download/ , compile it with 
VTK_RENDERING_BACKEND=OpenGL and install it. In case you are not familiar 
with building libraries:
copy and extract the downloaded library somewhere (I personally put it in /opt).
then:

```bash
mkdir build  && cd build
cmake  -DVTK_RENDERING_BACKEND=OpenGL
make -j8
sudo make install
```

Next we download and compile bullet physics:
```bash
cd /opt
sudo mkdir bullet_physics
sudo chown <YOUR USERNAME> bullet_physics
cd bullet_physics
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
mkdir build && cd build
cmake -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_EXTRAS=ON DINSTALL_EXTRA_LIBS=ON ..
make -j8
sudo make install
```

Assuming everything went well, you can finally download the atar repository 
in your catkin workspace and compile it:
```bash
git clone https://github.com/neemoh/ATAR.git
catkin build
```

## Creating tasks

## Creating mesh objects with Blender:
You can use blender to easily create msh objects. When the object is ready follow these steps to make sure you have the correct units even if your mesh's dimensions are as small as a few millimiters:
convert it to mesh (alt-c)
go to the Scene tab and in the Units section selec Centimeters
Go to File-> Export and select Wavefront(.obj)
In the options set the scale as 0.01 and save.

