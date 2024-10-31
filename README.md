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
Before writing your own code, please run and understand how the following examples work. Note: Before running the examples below, ensure that two required launch files are executed first.

### Simple robot control (`simple_robot_control.py`)
The objective of this module is to send commands to the robot so that it moves as intended. This is achieved using the `goToPose` function. 

```bash
rosrun edg_ur10 simple_robot_control.py
```

The speed and acceleration of the UR robot can be specified when creating an instance of the `rtdeHelp` class:

```python
rtde_help = rtdeHelp(125, speed=0.1 , acc= 0.1)
```

Note: Replace `125` (control rate) with the appropriate value for your setup.

The final pose object is constructed using two pieces of information: position and orientation.

```python
position = [0.520, -0.200, 0.40]
orientation = tf.transformations.quaternion_from_euler(np.pi,0,-np.pi/2,'sxyz') #static (s) rotating (r)
```

- __Position__: A list `[x, y, z]` representing the coordinates with respect to the world frame. You can verify these values using the robot's pendant by setting the coordinate system to 'Base'.
- __Orientation__: Euler angles are used. In this example, the 'sxyz' order specifies static frame rotations about the x, y, and z axes in that sequence. The rotation is applied as `R_x(θ_x)`, then `R_y(θ_y)`, followed by `R_z(θ_z)`.

The pose object is obtained with:
```python
rtde_help.goToPose(pose)
```

To command the robot to move to a defined pose, use the `goToPose` function:

```python
rtde_help.goToPose(pose)
```

This script demonstrates a simple robot movement from point A (`poseA`) to point B (`poseB`), then rotates with respect to the Tool Center Point (TCP) to reach pose C (`poseC`).


### simple data log (`simple_data_log.py`)
This script, `simple_data_log.py`, is designed for basic data logging using a UR10e robot and ATI sensors. It records ATI data and robot positions for a specified duration (10 seconds in this example).

```bash
rosrun edg_ur10 simple_data_log.py
```

The `data_logger.py` module records all topics specified in `TopicsList.txt` located in the `config` folder. By default, `/endEffectorPose` and `/netft_data` are listed.

`TopicsList.txt`:
```text
/endEffectorPose
/netft_data
```

If you define your own message types or use conventional message types and you want to record them, you should add the topic names to `TopicsList.txt`.

For example, if you want to record the `/sync` topic, you can add it to the list:

```text
/endEffectorPose
/netft_data
/sync
```

You need to enable and disable the data logger using a service so that you can specify the time window during which you want to record data.

First, you need to define the service that you want to call:

```python
print("Wait for the data_logger to be enabled")
rospy.wait_for_service('data_logging')
dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
dataLoggerEnable(False) # reset Data Logger just in case
rospy.sleep(1)
file_help.clearTmpFolder()  # clear the temporary folder
datadir = file_help.ResultSavingDirectory
```

Then, send an enable command to the service to start data logging:

```python
# Start data logging
dataLoggerEnable(True)
```

After you are done, you need to stop recording so that raw data is saved properly:

```python
# stop data logging
dataLoggerEnable(False)
rospy.sleep(0.2)
```

Finally, all topics and arguments you specified in the script will be saved as a `.mat` file. By using `file_help.saveDataParams`, you can append text to the file name:

```python
# save data and clear the temporary folder
file_help.saveDataParams(args, appendTxt='Simple_data_log_'+'argument(int)_'+ str(args.int)+'_argument(code)_'+ str(args.currentTime))
file_help.clearTmpFolder()
```
The `.mat` file will be saved in a designated folder. By default, the files will be saved into `EDG_Experiment`. If you want to change the parent folder, you need to specify the folder name when you instantiate the `file_help` class:

```python
 file_help = fileSaveHelp(savingFolderName = 'EDG_Experiment')
```



### simple experiment (simple_experiment.py)


## Author
Please contact the author to ask any question and need any funcion

👤 **Jungpyo Lee**
- e-mail: jungpyolee@berkeley.edu
- Github: [@Jungpyo-L](https://github.com/Jungpyo-L)
