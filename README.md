# EDG Robot Control ROS Package (Node/Python)

## Objective
The objective of this package is to regroup in one place every modules useful for the control of EDG's UR-10 robotic arm with RTDE controller. This package also expose a launch file that is useful to quickstart all programs related to the control of the arm. THe main functions of this package is to move the UR robot to desired pose and recording data (robot pose, ATI F/T, ect.).

## How to use the launch file?
The included launch file takes care of loading the appropriate robot description, the parameters and the configuration of the UR-10 arm through the usage of the [universal_robot](https://github.com/ros-industrial/universal_robot) ROS package. Furthermore, this launch file spawn an instance of [MoveIt](https://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html), again using the files coming with the [universal_robot](https://github.com/ros-industrial/universal_robot) package. Finally, this launch file also spawn an instance of the [rviz](http://wiki.ros.org/rviz/UserGuide) program which let's the user see a virtual robot with its environment as well as allowing to interact with the robot.

Ethernet connection for an experiment. The RealTek Ethernet Connected must connect to UR10 Robot.

In order to use this package, the following two launches are needed to execute (use two separate terminals).

The first launch file takes a single argument which is the IP address of the robot. By default, a direct connection is assumed and the IP address of 10.0.0.1 is assumed to be the one of the robot. Also, this launch file excutes rviz to see the current and desired robot pose. As all launch files, the usage is pretty simple:
```bash
roslaunch edg_ur10 ur_control.launch
```

The Second launch file is mainly for data-logging. It launches data logger and visual rqt for real-time force torque readings.
```bash
roslaunch edg_ur10 ur_experiment.launch
```

## ✨ How does it work?
The principal script uses several different [Python modules](https://docs.python.org/2/tutorial/modules.html) and functions in heloperFunction in order to work. Each of these modules have a specific purpose.

### The robotStatePublisher module
This module keep publishing TCP pose as a topic (/endEffectorPose) with 30 Hz. If you want to change the publish rate, you need to edit the followng line:
```python
rate = rospy.Rate(30)
```
TCP pose published here is from TCP offset set in the UR pendent (not in the code).

This module is excuted from the second launch file ('ur_experiment.launch').


### The data_logger module
The objective of this module to grab topics and save them as .csv files. This module uses 'service' so that user can request and stop data logging process.

The topics in TopicList.txt (inside /config) are only recorded from this data logger. By default, /endEffectorPose and /netft_data are recorded. If you want to log your own defined topics, please add lines in TopicList.txt (see the example code 'simple_data_log.py').

This module is excuted from the second launch file ('ur_experiment.launch').

### Helper Functions
These modules' primary purpose are to make the main code simple by integrating all helper functions related to robot control and data logging

#### The rtde_helper module
This module has all things about robot control. It uses Real-Time Data Exchange (RTDE) protocol designed for Universal Robots (see details: https://www.universal-robots.com/products/ur-developer-suite/communication-protocol/rtde/). It includes basic motion of the UR robot using moveL and servoL as well as getting a TCP pose from the robot, getActualTCPPose. If needed, you can add other control methods (see details: https://sdurobotics.gitlab.io/ur_rtde/).

#### The FT_callback_helper module
This module is for subscribing data (/netft_data) from the ATI F/T sensor and performing 7-points moving average. Also, it helps us remove offset of data. In that case, need to use average_NoOffset variables.


#### The trasnformation_matrix module
This module includes functions for transformation matrixes and other form of them, including PoseStamped.

#### The utils module
This module has many util functions for mathematical calculation of robotic application, such as hat operator.

#### The fileSaveHelper module
This module saves data in the form of .mat file from logged cvs files. It saves mat file into EDG_Experiment folder. If you want to change the name of the parent folder, please change savingFolderName when you call this object.

```python
def __init__(self, savingFolderName = 'EDG_Experiment'):
```

## 🚀 Usage (Example codes)
Before run following example codes, the launch files are needed to excecute first.

### Simple robot control (simple_robot_control.py)
This script is to demonstrate a simple robot move from point A to B with a goToPose function. The speed and acceleration of the UR robot can be changed when rtde_helper 

```bash
rosrun edg_ur10 simple_robot_control.py
```


The objective of this module is to send commands to the robot such that it moves as intended. To do so, three main functions can be used: goRelOrientation, goRelPosition and goPose. The first is used to move the wrist of the robot by a number of degrees. The second is used to translate the robot end effector in space relative to its current position and following the world reference frame. The third function is used to command the robot to go to a specific precomputed pose.

Although it would be entirely possible to calculate the destination pose relative to the world reference frame and to use exclusively the goPose function, it would also make things more complicated than what is required for most common tasks. An easier workflow would be:
1) Use the teach pendant to reach the desired position.
2) Run the PrintCurrentPose.py script in order to generate the lines of code needed to reach that position/orientation.
3) In your program, use the previously generated lines of code to instruct the robot to reach that chosen pose.
4) Then translate the end effector using the goRelPosition function.
5) Then, if needed, change the orientation of the end effector using the goRelOrientation function.
6) Rinse and repeat the last two steps as needed.

### simple data log (simple_data_log.py)
Topic list
```bash
rosrun edg_ur10 simple_data_log.py
```

### simple experiment (simple_experiment.py)


## Author
Please contact the author to ask any question and need any funcion

👤 **Jungpyo Lee**
- e-mail: jungpyolee@berkeley.edu
- Github: [@Jungpyo-L](https://github.com/Jungpyo-L)
