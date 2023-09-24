# Makercie Lab Solutions - Publisher Node Creation
##### by the Makercie Training & Tech Support Department
## Task 1: Creating a Workspace & Package
Before we begin writing a node, we need to create a workspace and a package. Workspaces are simply directories to store all of your packages. First we will need to create a new directory.

`mkdir ~/training`

This created a directory in your home folder which we will use as a workspace directory. We now need to create a subdirectory in your workspace directory to store all your source code for your packages

`mkdir ~/training/src`
The last step to creating the workspace will be to initialize the workspace with catkin_make.

`cd ~/training/src`
`catkin_make`
## Task 2
Now that our workspace has been created, we will create a package in the src directory we just created. This can be done by navigating to the ~/training/src directory, which you should have already done in the last step, and using the catkin_create_pkg command  followed by what we want the package to be named, and then followed by what other packages our package will be dependent on; this command creates another directory for your new package, and two new configuration files inside that directory with some default settings.

`catkin_create_pkg random_rover_driver roscpp std_msgs`
You can see that this created CMakeLists.txt and package.xml inside the random_driver directory; this is also where you will store all your source code for your packages. The roscpp and std_msgs  dependencies were added into the CMakeLIst.txt and package.xml.

```#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "random_rover_commands");
     ros::NodeHandle nh;

     //Ceates the publisher, and tells it to publish
     //to the rover/cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("rover/cmd_vel", 100);

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

       while(ros::ok()) {
            //Declares the message to be sent
            geometry_msgs::Twist msg;
           //Random x value between -2 and 2
           msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
           //Random y value between -3 and 3
           msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
           //Publish the message
           pub.publish(msg);

          //Delays untill it is time to send another message
          rate.sleep();
         }
}
```
Let’s break down this code line by line,

```
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
```

These lines includes the headers that we are going to need. The *<ros/ros.h> *header is required for ROS functionality and  the *<geometry_msgs/Twist.h>* is added so that we can create a message of that type.

```
ros::init(argc, argv, "random_rover_commands");
ros::NodeHandle nh;
```

The first line, *ros::init*,  is used to initialize the ROS node, and name it *“random_rover_commands”*, while ros:NodeHandle starts the node.
```
ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("rover/cmd_vel", 100);
```
Publishing a message is done using *ros:Publisher pub=nh.advertise*, followed by the message type that we are going to be sending, in this case it is a *geometry_msga::Twist*, and the topic that we are going to be sending it too, which for us is rover/cmd_vel.

The 100 is the message queue size, that is, if you are publishing message faster then what roscpp can send, 100 messages will be saved in the queue to be sent. The larger the queue, the more delay in robot movement in case of buffering.

Therefore in a real life example, you will want to have a smaller queue in the case of robot movement, where delay in movement commands are undesirable and even dangerous, but dropped messages are acceptable. In the case of sensors, it is recommended to use a larger queue, since delay is acceptable to ensure no data is lost.
```
ros::Rate rate(10)
rate.sleep()
```

ROS is able to control the loop frequency  using ros:Rate to dictate how rapidly the loop will run in Hz. rate.sleep will delay a variable amount of time such that your loop cycles at the desired frequency. This accounts for time consumed by other parts of the loop. All Clearpath robots require a minimum loop rate of 10Hz.
```
while(ros::ok())
```
The ros::ok function will return true unless it recevis a command to shut down, either by using the rosnode kill command, or by the user pusing Ctrl-C in a terminal.
```
geometry_msgs::Twist msg;
```
This creates the message we are going to send, msg, of the type 
```
geometry_msgs:Twist
msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
```

![](../images/Pasted%20image%2020230924182216.png)

These lines calculate the random linear x and angular z values that will be sent to rover.
```
pub.publish(msg)
```
We are finally ready to publish the message! The pub.publish adds msg to the publisher queue to be sent.

## Task 3
Compilation in ROS in handled by the catkin build system. The first step would usually be to set up our package dependencies in the CMakeLists.txt and package.xml. However this has already been done for us when we created the package and specified our dependencies. The next step is then to declare our new node as a executable, this is done by adding the following two lines to the CMakeLists.txt files in ~/ros101/src/random_rover_driver

add_executable(random_driver src/random_driver.cpp)
target_link_libraries(random_driver ${catkin_LIBRARIES})
The first line creates the executable called random_driver, and directs ROS to it’s source files. The second lines specifies what libraries will be used. Now we need to build our workspace using the catkin_make command in the workspace directory

cd ~/ros101
catkin_make
Let’s bring up the rover visualization as we did in a previous blog post.

roslaunch rover_gazebo rover_empty_world.launch
The final step is to source your setup.bash file in the workspace you have created. This script allows ROS to find the packages that are contained in your workspace. Don’t forget this process will have to be done on every new terminal instance!

source ~/ros101/devel/setup.bash
It’s now time to test it out! Make sure you have a instance of roscore running in a separate terminal, then start the node.

rosrun random_rover_driver random_driver
You should now see rover drive around! In a new terminal window, we can make sure that our node is publishing to the /rover/cmd_vel topic by echoing all messages on this topic

rostopic echo /rover/cmd_vel

