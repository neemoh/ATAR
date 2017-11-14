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
the following. There are two methods of the Task class that are called 
periodically:
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

#### Hand-Eye Calibrations
The local to world transformation is found differently in VR and AR cases. The
common element in both cases is that we need to know the pose of the
camera with respect to the world (camera_to_world_frame_tr). This has to
be set from outside of the class by passing the pointer of this Manipulator 
object to a Rendering object through the SetManipulatorInterestedInCamPose 
method (check the Demo2Task). After doing that the camera pose will be 
communicated to the manipulator every time it changes. Now about the rest of 
the kinematics chain:

##### VR: 
The manipulator is interfacing with a master device. Here the 
local_to_world_frame_tr is calculated as:

local_to_world_frame_tr.M = camera_to_world_frame_tr.M *local_to_image_frame_rot;

where local_to_image_frame_rot is the tr from the base of the master
device to the image frame (i.e. the image you see in the display, i.e. the
camera!). THe image frame is opencv style: X axis is left to right, y is
top to bottom and so z is perpendicular into the image. Note that we are
only interested in the rotation from the master base to the image. This tr
is set by setting a parameter "/calibrations"+arm_ns+"_frame_to_image_frame"
with 4 elements representing the quaternion rotation. Check the
params_calibrations_test_vr.yaml file to see examples of this.

##### AR: 
In the augmented reality case we are interfacing with a slave arm
that is seen in the camera images. Here we need to find the transformation
from the slave to the world frame by performing a calibration procedure. 
TO BE COMPLETED
![example_screenshots](https://github.com/neemoh/ATAR/blob/master/resources/Screenshot_references_frames.png)

## TODO: Augmented Reality 
Things to explain: camera. Calibrations (intrinsic, extrinsic, arm to world)

intrinsic calib files: make a folder named camera_info in the .ros folder in your 
                       home directory and copy the default_intrinsics.yaml file from the 
                       /ATAR/resources folder in camera_info.
### AR Camera

* "/"+cam_name+ "/image_raw"
* intrinsic calibration file
* "/calibrations/world_frame_to_"+cam_name+"_frame"
* "/"+cam_name+ "/world_to_camera_transform"

Place the intrinsic calibration file of each camera in ~/.ros/camera_info/ 
named as <cam_name>_intrinsics.yaml

## Miscellaneous

* Grasping:  