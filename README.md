# Assisted Teleoperation with Augmented Reality

This repository contains the source code for the ATAR package. This ROS package can be used to design interactive augmented reality (or virtual reality) tasks using a stereo camera and display, master and slave manipulators. I have developed and tested this on a da Vinci Research Kit (DVRK) that comprises 5 manipulators (2 masters, 2 slaves and a camera arm), a stereo endoscope and stereo vision console. Tha package includes the code for the calibrations involved (Camera intrinsics, extrinsics, and arm to world) and a qt graphical user interface. THe simulated world includes rigid and soft dynamics simulation based on the Bullet Physics library and graphics are generated using the Visualization toolkit (VTK) and OpenGL.

![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_for_readme.png)

## Prerequisites
Apart from ROS (Tested with Kinetic), this package needs the following libraries:
* [VTK](http://www.vtk.org/download/) - For the graphics
* [Bullet](http://bulletphysics.org/wordpress/) - For the physics

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
then inside the extracted vtk folder:

```bash
mkdir build  && cd build
cmake  -DVTK_RENDERING_BACKEND=OpenGL ..
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
cmake -DUSE_DOUBLE_PRECISION=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBUILD_EXTRAS=ON DINSTALL_EXTRA_LIBS=ON ..
make -j8
sudo make install
```

Assuming everything went well, you can finally download the atar repository 
in your catkin workspace and compile it:
```bash
git clone https://github.com/neemoh/ATAR.git
catkin build
```
To test it, first make a folder named camera_info in the .ros folder in your 
home directory and copy the default_intrinsics.yaml file from the 
/ATAR/resources folder in camera_info. Then run the vr_test.launch file and click on 
task 8. Wait for the compound convex mesh to be generated and the task should
 run after that. You should see some spheres and cubes falling on a floor, 
 which would mean you have successfully built the libs. Congrats! 
 In case you have an nvidia card and its driver has benn installed, you can 
 have faster graphics and also shadows. To test the shadows set the 
 with_shadows flag to true in the vr_test.lanuch file.

## TaskHandler
Details will be updated.
![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_for_readme_ATAR_Arch.png)

### Creating tasks
Details will be updated.

### Creating simulated objects
You can define objects using the SimObject class which has a graphic actor 
(actor_) and physics body (rigid_body_). A simObject can be dynamic (i.e. 
its pose will be update by physics simulation), static (i.e constant pose), 
kinematic (i.e. pose is assigned externally, e.g. from a mster device). There
 are some primitive shapes available (Cube, sphere, cylinder, cone and plane)
 , but more usefully the shape can be from a .obj mesh file. To learn more 
 about SimObjects refer to SimObjects.h.

#### Mesh objects
Meshes are decomposed into approximated compound meshes using the VHACD 
method. Generating the approximated compound mesh can take up to a few 
minutes. So we do this only once and save the compound mesh in a separate 
file that has the same name of the original mesh file with an added _hacd. 
Next time the application is executed we search for the file with _hacd and 
if found, it is used and compound mesh generation is not repeated.

##### Creating mesh objects with Blender:
You can use blender to create mesh objects. When the object is ready, follow 
these steps to make sure you have the correct units even if your mesh's 
dimensions are as small as a few millimiters: 
* Convert it to mesh (alt-c)
* Go to the Scene tab and in the Units section selec Centimeters
* Go to File-> Export and select Wavefront(.obj). In the options set the scale
 as 0.01 and save.

### Manipulator Class
In order to interact with the virtual environment you would need to have an 
input device of some sort. The Manipulator class helps you to read the 
cartesian pose and twist of that device (assuming some other node is 
publishing them) and transform them to the world reference frame. This
local to world transformation is found differently in VR and AR cases. The
common element in both cases is that we need to know the pose of the
camera with respect to the world (camera_to_world_frame_tr). This has to
be set from outside of the class by passing the pointer of this Manipulator 
object to a Rendering object through the SetManipulatorInterestedInCamPose 
method (check the DemoTask). After doing that the camera pose will be 
communicated to the manipulator every time it changes. Now about the rest of 
the kinematics chain:

* VR: The manipulator is interfacing with a master device. Here the 
local_to_world_frame_tr is calculated as:
local_to_world_frame_tr.M = camera_to_world_frame_tr.M *local_to_image_frame_rot;
where local_to_image_frame_rot is the tr from the base of the master
device to the image frame (i.e. the image you see in the display, i.e. the
camera!). THe image frame is opencv style: X axis is left to right, y is
top to bottom and so z is perpendicular into the image. Note that we are
only interested in the rotation from the master base to the image. This tr
is set by setting a parameter "/calibrations"+arm_ns+"_frame_to_image_frame"
with 4 elements representing the quaternion rotation. Check the
params_ar_calibrations_polimi.yaml file to see examples of this.

* AR: In the augmented reality case we are interfacing with a slave arm
that is seen in the camera images. Here we need to find the transformation
from the slave to the world frame by performing a calibration procedure. 
TO BE COMPLETED

## Augmented Reality
Things to explain: camera. Calibrations (intrinsic, extrinsic, arm to world)

### AR Camera

* "/"+cam_name+ "/image_raw"
* intrinsic calibration file
* "/calibrations/world_frame_to_"+cam_name+"_frame"
* "/"+cam_name+ "/world_to_camera_transform"

Place the intrinsic calibration file of each camera in ~/.ros/camera_info/ 
named as <cam_name>_intrinsics.yaml