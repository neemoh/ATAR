# Assisted Teleoperation with Augmented Reality

This repository contains the source code for the ATAR package. This ROS 
package can be used to design interactive augmented reality or virtual 
reality tasks using a stereo camera and display, master and slave 
manipulators. I have developed and tested this on a da Vinci Research Kit 
(DVRK) that comprises 5 manipulators (2 masters, 2 slaves and a camera arm), 
a stereo endoscope and stereo vision console. I also have tested it with a 
single display and a Sigma master device interacting with a VR task. I have 
tried to make the package as modular and generic as possible, so that it can be 
used for different purposes such as creating a VR or AR interactive simulation 
with embedded physics or simply overlaying 3D graphics on top of a single 
camera images. The camera images and the manipulator cartesian 
measurements are read through ros topics, so if you want to create a 
simulation that includes one of these you should have a node that publishes 
them. The package includes the code for the calibrations involved (Camera 
intrinsics,  extrinsics using charuco boards and manipulator base to camera 
base) and a qt graphical user interface. Bullet Physics library is used for the 
rigid and soft body dynamics simulation and graphics are generated using the 
Visualization toolkit (VTK).

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
To test it, run the test_vr.launch file and click on task 1. Wait for the 
compound convex mesh to be generated (you can see the progress in the terminal)
and the task should run after that. You should see some spheres and mesh 
objects falling on a floor, while the camera rotates, which would mean you 
have successfully built the libs. Congrats! 
 In case you have an nvidia card and its driver has benn installed, you can 
 have faster graphics and also shadows. To test the shadows set the 
 with_shadows flag to true in the lanuch file.

## TaskHandler
This is the class that subscribes to a control events topics (published by the 
GUI) and loads and unloads tasks. When you want to add a new task, you can 
make a class with a name that starts with Task<something> (because all the 
files starting with Task will be automatically added to the CMakeLists) and 
include your task's header in the TaskHandler and add its allocation to the 
StartTask method. The task class must be a child of SimTask class (Check 
TaskDemo1). 
* Note that you need to reload your CMakeLists project for the automatic
addition of your Task file to the cmake project. If you don't know how to do
that, just modify a line in the CMakeLists.txt file like add a space or 
something before compiling the project!
 
![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_for_readme_ATAR_Arch.png)

## Creating tasks
In my first version of the code the Task class used to have only the 
simulated objects in it. Later I decided to take everything (i.e. rendering 
and communication with the manipulators etc) into the task class so that we 
can have tasks with different rendering configs (ar or vr, different number 
of rendering windows, resolutions...) and different manipulators. This of 
course adds some overhead cost (and some memory leak that I will hopefully 
fix soon!).
The block diagram above shows the main elements of a task class. You can have
a look at the TaskDemo1 and TaskDemo2 for minimal examples. The main 
elements are the Rendering, SimObjects and manipulators that are explained in 
the following. 

The usual flow is that you create SimObjects (or vtk actors or
 Bullet bodies) and add them to the world in the constructor. Then in one 
 of the following two periodically called functions you write task logic 
 and other things that need to be done on runtime. There are two methods of 
 the Task class that are called periodically:
* TaskLoop: This is called from the main thread at a refresh rate of about 30
 Hz which makes it the choice for updating graphics and task related logic.
 * HapticsThread: This method is called from a separate thread and you can 
 set its refresh rate directly in the method (creating a loop and so on). 
 This thread exists for haptics related matters where a high refresh rate is 
 needed to provide a stable feedback. Have a look at the TaskSteadyHand 
 (haptics thread running at 500Hz or 1KHz) for an example of how to use this 
 thread.   
 
### 1. Rendering Class
As its name suggests this is where the graphics are produced. I have moved 
the explanations regarding the augmented reality case to the bottom to 
simplify the description here. The constructor of the Rendering class let's 
you specify the configuration of the windows that you want. It is easier to 
explain this with an example: In our DVRK setup we had two displays in the 
surgeon console (one for each eye) and we wanted to show what is happening 
in the console in a third display to viewers. These 3 displays were connected
to the same graphics card in a horizontal layout is you can see in the image
bellow:
![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_for_readme_Render_window.png)

We can have one window with three views in it defined as:

```
    bool ar_mode = false;
    int n_views = 3;
    bool one_window_per_view = false;
    bool borders_off  = true;
    std::vector<int> view_resolution = {640, 480};
    std::vector<int> window_positions = {1280, 0};

    graphics = new Rendering(view_resolution, ar_mode, n_views,
                             one_window_per_view, borders_off,
                             window_positions);
 ```
Having the borders off allows for full screen view in the dvrk displays. 
You can have multiple windows (up to 3) by setting the one_window_per_view 
flag as true. However, VTK fails to generate the shadows when there are more
than one rendering windows. If you have 2 windows, the 
window_positions must contain 4 elements(x,y,x,y) and 6 elements if you have 
3 windows.
 
### 2. SimObjects
You can define objects using the SimObject class which has a graphic actor 
(actor_) and physics body (rigid_body_). A SimObject can be dynamic (i.e. 
its pose will be update by physics simulation), static (i.e constant pose), 
kinematic (i.e. pose is assigned externally, e.g. from a master device). There
 are some primitive shapes available (Cube, sphere, cylinder, cone and plane)
 , but more usefully the shape can be from a .obj mesh file. After 
 constructing a SimObject call the AddSimObjectToTask method of the Task to 
 add your object to the simulation.
 * Note: You can still use all the functionalities of VTK for creating 
 graphics. When you create a vtk actor, call graphics->AddActorsToScene
 (my_actor).
 
 For Sphere and Plane shapes you can have a texture image. PNG and JPG 
 formats are supported. To learn more about SimObjects refer to SimObjects.h.

#### Mesh objects
Meshes are decomposed into approximated compound meshes using the VHACD 
method. Generating the approximated compound mesh can take up to a few 
minutes. So we do this only once and save the compound mesh in a separate 
file that has the same name of the original mesh file with an added _hacd. 
Next time the application is executed we search for the file with _hacd and 
if found, it is used and compound mesh generation is not repeated.
* Note: The generated compound meshes are approximate and sometimes the 
approximation deviates considerably from the original mesh. To check how the 
generated compound object looks like, you can either open the generated 
<filename>_hacd.obj in blender or set the show_compound_mesh boolean to 
true in the constructor of SimObject.

##### Creating mesh objects with Blender:
You can use blender to create mesh objects. When the object is ready, follow 
these steps to make sure you have the correct units even if your mesh's 
dimensions are as small as a few millimiters: 
* Convert it to mesh (alt-c)
* Go to the Scene tab and in the Units section select Centimeters
* Go to File-> Export and select Wavefront(.obj). In the options set the scale
 as 0.01 and save.

### 3. Interact with objects using a haptic device
In order to interact with the virtual environment you would need to have an 
input device of some sort. The Manipulator class helps you to read the 
cartesian pose and twist of that device (assuming some other node is 
publishing them) and transform them to the virtual world reference frame. 
Check TaskDemo2 for an example of this. You can use the SimMechanism class to
make a virtual tool that follows the pose of the real manipulator. I have 
already created one called:
  * SimForceps. It consists of 3 links. first is a small box representing the
   base. The other two are jaws (mesh objects) that are connected with an 
   elastic link to the base cube. The angle of the jaws is set with a 
   bulletConstraint that is actuated. All these complications is because 
   Kinematic objects interact very aggressively with the dynamic objects, so 
   if the forceps were 3 kinematics links, it would have been impossible to 
   grab something with them. That's why the jaws sometimes get distorted! 
   since are actually dynamic objects and when they hit something their 
   elastic constraint let's them move too far from the base...

#### Grasping: 
Grasping is a challenging interaction to simulate. The 
multi-lateral physical interaction between the surface of the tool and the 
object creates a large amount of energy which leads to the instability. I did
not find the time to dig more into this and tried to stabilize the grasp by 
adding more friction and contact damping.


#### Hand-Eye Calibrations
The local to world transformation is found differently in VR and AR cases. The
common element in both cases is that we need to know the pose of the
camera with respect to the world (camera_to_world_frame_tr). This has to
be set from outside of the class by passing the pointer of this Manipulator 
object to a Rendering object through the SetManipulatorInterestedInCamPose 
method (check the Demo2Task). After doing that the camera pose will be 
communicated to the manipulator every time it changes. Now about the rest of 
the kinematics chain:

##### VR or MASTER MODE: 
The manipulator is interfacing with a master device. Here the 
local_to_world_frame_tr is calculated as:

local_to_world_frame_tr.M = camera_to_world_frame_tr.M *local_to_image_frame_rot;

where local_to_image_frame_rot is the tr from the base of the master
device to the image frame (i.e. the image you see in the display, i.e. the
camera!). THe image frame is opencv style: X axis is left to right, y is
top to bottom and so z is perpendicular into the image. Note that we are
only interested in the rotation from the master base to the image. This tr
is set by setting a parameter "/calibrations"+arm_name+"_frame_to_image_frame"
with 4 elements representing the quaternion rotation. Check the
params_calibrations_test_vr.yaml file to see examples of this.

##### AR or SLAVE MODE: 
In the augmented reality case we are interfacing with a slave arm that is 
seen in the camera images. Here we need to find the transformation from the 
slave to the world frame by performing a calibration procedure. This 
calibration is done by pointing at 6 known points on a charuco board and is 
implemented as a separate temporary thread when the 
DoArmToWorldFrameCalibration method is called. Here we directly calculate the
local_to_world_frame_tr and therefore we don't need to SetWorldToCamTrfrom 
outside like in the VR case. After the calibration a ros parameter is set 
called: "/calibrations/world_frame_to_"+arm_name+"_frame"
you can set this parameter in the params_calibrations_ar.yaml so that you 
don't have to repeat the calibration as long as the base of the robot does 
not move with respect to the world (board) coordinate. 

![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_references_frames.png)

### AR Camera
This class used to interface with a camera through ros is similar to what the 
Manipulator class is for a robot arm. It reads images, intrinsic and 
extrinsic calibrations and in case nothing is found on topics for the latter 
two, it calculates them. For each view we create one ARCamera and for each 
ARCamera you need to provide a cam_name as a ros parameter cam_<NUMBER>_name 
where <Number> is from 0 to 2 (e.g: cam_0_name). Check out test_ar.launch.
Here are some details about each of these 3 tasks:

#### 1- reading images
This simply uses image transport from ros to read images that are published 
on a topic. We are of course assuming that an external node is publishing 
the images. For example for a simple usb camera you can use the uvc_camera 
package of ros. The topic we listen to is:
* "/"+cam_name+ "/image_raw"
You can use remap in the lunch file if your camera node publishes the images 
on a different topic:

        <remap from="/ar_core/left/image_raw" to="/dvrk/left/image_raw"/>
 
#### 2- Intrinsic calibration file
Intrinsic camera parameters are needed for a correct mapping of the 3D world 
to the camera images. We can get the parameters in two way:
* A yaml file. We search in your home directory under the .ros/camera_info 
directory for a file named as <camera_name>_intrinsics.yaml
* Subscribe to camera_info topic. This is the ros way of accessing intrinsic 
parameters that is: the camera node that publishes the images publishes also 
a topic called camera_info that contains needed parameters about the camera.
* If ARCamera doesn't find the parameters from any of the above two, it 
starts the intrinsic calibration procedure with a charuco board (explained 
bellow) in which you need to take at least 15 different poses of the board in
front of the camera (instructions are shown on the image). If the calibration
is successful we create a yaml file and put it in the camera_info directory. 
unfortunately this file has a different format than that used by ros and 
that's why we name it <camera_name>_intrinsics.yaml although ros camera_files
are supposed to be called <camera_name>.yaml.

#### 3- Extrinsic calibration
Extrinsic calibration refers basically to the pose of the camera with 
respect to a world reference frame. In ATAR we need this pose to correctly 
map the virtual 3D world to the real world of the images. Like intrinsic 
calibration here we use a charuco board again. To make things simpler we 
assume that the world_reference frame is on a corner of the charuco board. 
This means that if both intrinsic and extrinsic calibrations are correct, 
when you position a small virtual sphere at [0, 0, 0] it should appear on 
a corner of the charuco board. The extrinsic (i.e. camera pose) is found in 
two ways in ARCamera
* Read as a PoseStamped message from one of the following topics:
"/calibrations/world_frame_to_"+cam_name+"_frame"
"/"+cam_name+ "/world_to_camera_transform"
* Calculated at each iteration using the aruco library and the charuco board.


###### Charuco board
Both for intrinsic and extrinsic calibrations we rely on a charuco board. 
This is a checker board that has some fiducial markers (called aruco) instead
of its white squares. We use the aruco library to detect those markers in the
camera image and perform intrinsic or extrinsic calibration based on the 
markers' spatial information. You need to print a charuco board (for 
example charuco_d0_h4_w6_sl200_m10_ml150.png that is in the resources folder) on a 
paper and make it solid by attaching it to a cardboard. Then you'll need to 
set the parameters of the board in your lunch file so that atar knows what to
look for. In the test_ar.launch file you can see what are the elements 
of board_params. 
If you want to make a different board run
```
rosrun atar create_charuco_board <YOUR PARAMS>
 
```


## TODO:
* After Manipulator to world calibration, the calculated transformation is 
used and set as a param, but not saved in the params_calibrations.YAML file. 
So if the ros master is restarted the transformation is lost.
* Add the option of directly reading usb camera in the ARCamera class.


## License
This software is released under a BSD license:

Copyright 2017 Nima Enayati.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:
Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
POSSIBILITY OF SUCH DAMAGE.
