# 808X Week10 Submission
## Overview
The pub/sub assignment where user can provide a command line String argument which will be read from argv and published but the 'talker' ros node and 'listener' ros node subscribes to the message and echoes it.

A launch file is provided to start both talker and listener nodes with the option of providing arguments to set a custom string displayed and a rate at which the string is set.  If no argument is provided, default values will be used.

The talker nodes provides a service call /updateStr that enables the user to update the string to be used via the rosservice utility. 

The talker publishes a frame with non-zero static rotation and trasnlation.

A record.bag file for the talker was recorded and posted on the /results directory

## Standard install clonning from git

On a catkin workspace src folder:
```
git clone --recursive https://github.com/stinta/beginner_tutorials.git
```
On a catkin workspace
```
catkin_make
```

## Running and Starting ROS nodes manually
```
cd <path to repository>
source devel/setup.sh
roscore

Open a couple of terminals and for each run 
cd <path to repository>
source devel/setup.sh
Terminal1: rosrun beginn_tutorials talker "CUSTOM STRING"
Terminal2: rosrun beginn_tutorials listener
```
## Running and Starting ROS node via the launch file
Ensure that the setup.sh script has been sourced
Note that the arguments to be provided are a string followed by a double.  If no value or invalid values are provided'
the program will sure default values
```
roslaunch beginner_tutorials Week10_HW.launch myArgs:="SENCONDTRY 0.5"
```

## Updating the string via de service
While the nodes are running
```
rosservice call /updateStr "UPDATED VIA  SERVICE CALL"

```

## TF Frames
The talker module publishes a non-zero non-time variant transformation.  The set-up of the frame can be found under the results folder.

To view the tf frame information published by talker:
```
rosrun tf tf_echo /world /talk

```
## Record rosbag for Talker via launch file
To record a rosbag with the Week10_HW.launch file provided a parameter enableRosBag must be set as true as shown.  If the parameter is not provided or set to false the rosbag is not recorded.
Once the cmd below is executed the recorded file can be found in <project_folder>/results/record.bag
```
roslaunch beginner_tutorials Week10_HW.launch enableRosBag:=true

```
## Inspecting the rosbag file
To view highlevel details
```
rosbag info record.bag
```

To replay the bag file
```
rosbag play record.bag
```
## Running Test
Test cases were added to the test folder
Running the test after compiling the code
```
rostest beginner_tutorials allTalkerTests.test 
```

## Code Checks

cppcheck
```
cppcheck --std=c++11 $(find . -name \*.cpp -or -name \*.srv | grep -vE -e "^./
build/" -e "^./results/")
```
cpplint
```
 cpplint $(find . -name \*.cpp | grep -vE -e "^./build/" -e "^./results")
```
## Author
Sandra P. Tinta

## License
[BSD-3](https://opensource.org/licenses/BSD-3-Clause)
