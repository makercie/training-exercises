# Makercie Training Lab: Creating a Publisher Node
##### by the Makercie Training & Tech Support Department

## Task 1
1. Create a folder called *training* in Ubuntu Noetic.
2. Create a workspace using *catkin_make* called *random_rover_driver*. (Use roscpp and std_msgs as prerequisites).
***Note:** This will create a CMakeLists.txt and package.xml inside the random_driver directory; this is also where you will store all your source code for your packages. The roscpp and std_msgs  dependencies were added into the CMakeList.txt and package.xml.*


## Task 2
As mentioned in our previous tutorials, a publisher publishes messages to a particular topic. For this tutorial, we will be publishing random commands to the /rover/cmd_vel topic to make your rover visualization drive itself. Start by creating a file in your
`~/training/src/random_rover_driver/src directory called random_driver.cpp`
and copy the following code:
```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "random_rover_commands");
     ros::NodeHandle nh;

     // Complete Q3 here
     // ros::Publisher 

     //Sets up the random number generator
     srand(time(0));

     //Set the loop to publish at a rate of 10Hz
     

       while(ros::ok()) {
          // Declare and setup your message here         
          
          pub.publish(msg);
          //Delays until it is time to send another message
          rate.sleep();

         }
}
```
3. Initialise the publisher, and tell it to publish to the rover/cmd_vel topic, with a queue size of 100
4. Establish a way in such that the while loop publishes at a *Rate* of *10Hz*
5. Declare a message variable, that is from *geometry_msgs*, which utilises *Twist* and name it *msg*.
6. Have it so the robot is sending a msg that it will move on it's x and z axis. (You must utilize the random number generator with an appropriate range).

## Task 3
1\. Review the CMakeLists.txt and package.xml files in the random_rover_driver package directory (~/ros101/src/random_rover_driver). Analyze how package dependencies are specified.
     1a. In the same package directory, declare a new executable named random_driver in the CMakeLists.txt file. Specify its source files and required libraries. Then, navigate to the workspace directory (~/training) and build the workspace using the catkin_make command. Ensure that all packages, including random_rover_driver, are successfully compiled.


2. Running and Testing
Launch the rover visualization with the roslaunch command. Verify that the rover visualization launches without errors.

3. In a new terminal instance, source the setup.bash file located in your workspace (~/ros101/devel/setup.bash). Remember that this step should be done in every new terminal instance.

4. Start the random_driver node using the rosrun command. Ensure you have a running instance of roscore in a separate terminal. Observe if the rover responds to the node's commands as expected.

5. In a new terminal, use the rostopic echo command to monitor messages on the /rover/cmd_vel topic. Confirm that the node is publishing messages to this topic as expected.