# Robotic Operating System (ROS)

## What is ROS?
ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. 

ROS is supported on Ubuntu, Debian, and Fedora.

## What is the ROS Community?
The ROS Community is a group of developers, researchers, and companies working together to advance the state of the art in robotics using ROS. The ROS Community is a non-profit organization that is open to anyone who is interested in ROS. The ROS Community is governed by a board of directors, and is funded by a membership program and donations.

## How does ROS work?
ROS is a framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. It is a distributed framework of processes that enables executables to be individually designed and loosely coupled. ROS provides conventions and tools so that your processes can discover and use each other's services without needing to know anything about the network. ROS also provides tools for configuring, launching, and monitoring the processes that make up your system.

## Who uses ROS?
ROS is used by researchers, hobbyists, and companies around the world. ROS is used in academia, research labs, and commercial environments. ROS is used by small teams and large organizations. ROS is used by hobbyists and professionals. ROS is used on small robots and large robots. ROS is used in warehouses and on the moon. ROS is used for education and research. ROS is used for fun and profit.

## Installation(Noetic)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt install ros-noetic-desktop-full
```

### Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Environment setup

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Getting rosinstall

```bash
sudo apt-get install python-rosinstall
```
## Create project
1. Create working directory: `mkdir -p workspace/src`
2. Get into working directory: `cd workspace`
3. Create catkin workspace: `catkin_make`
4. Source the workspace: `source devel/setup.bash`
5. Get into the source directory: `cd src`
6. Create a package: `catkin_create_pkg simple_pkg rospy`
7. Get into the package directory: `cd simple_pkg`
8. Create a script directory: `mkdir scripts`
9. Create node file: `touch scripts/simple_node.py`
## Simple ROS Node Example
- `simple_node.py`:
``` python
#!/usr/bin/env python3
# Import the rospy package: this almost always needs to be done
# Be ware that almost any thing that will be imported need to be added to the package.xml file first
import rospy

if __name__ == '__main__':
    # Initialize the node: this is lke a constructor for an object
    # TO initialize you need to provide a name for the node 
    # Such name could be different from the package name and the file name
    rospy.init_node("Simple_Node")

    # This is just to limit the rate of the node per second
    # This not necessary to do but it is a good practice for cpu usage
    rate = rospy.Rate(50)

    # Now the node will start to run and continue running until it is shutdown (ctrl + c)
    while not rospy.is_shutdown():

        # This is just to print a message to the terminal
        rospy.loginfo("Hello")

        # Put node to sleep for a certain time that is defined by the rate
        rate.sleep()

```
- We need to make the file executable To do that, open the terminal and type the following command:

``` bash
chmod +x {pkg path}/{file name}
chmod +x ~/catkin_ws/src/simple_pkg/scripts/simple_node.py
```
- To run the node, First start master node by typing the following command:
``` bash
roscore
```
- Now you can run the node by typing the following command:
``` bash
rosrun <pkg name> <node name>
```

## Simple ROS Publisher Example
- `simple_publisher.py`:
``` python
#!/usr/bin/env python3

# Import the rospy package: this almost always needs to be done
# Be ware that almost any thing that will be imported need to be added to the package.xml file first
import rospy

# Import the message type that you want to publish
# Note that the message type should be added to the package.xml file first
# In this case, the message type is std_msgs/String
# If you want your own custom message type, you need to create it first
from std_msgs.msg import String

# What is a publisher?
# A publisher is a node that will publish messages to a topic (explaned later)

if __name__ == '__main__':
    # Initialize the node: this is lke a constructor for an object
    # TO intialize you need to provide a name for the node 
    # Such name could be different from the package name and the file name
    rospy.init_node("Simple_Publisher")

    # This is just to print a message to the terminal
    rospy.loginfo("Simple_Publisher node has been initialized")

    # This is just to limit the rate of the node per second
    # This not neccesary to do but it is a good practice for cpu usage
    rate = rospy.Rate(10)

    # Create a publisher object that will publish to the topic "simple_topic"
    # A topic: is a channel that is used to publish and subscribe to messages (like message queues in other languages)
    # We don't need to create the topic first, the publisher will create it automatically
    # The second argument is the message type that will be published
    # Message type that will be published is String
    # Hopefully this much is sufficient for our case
    pubObj = rospy.Publisher("simple_topic", String)

    # Now the node will start to run and continue running until it is shutdown (ctrl + c)
    while not rospy.is_shutdown():

        # Create a message object
        msg = String()

        # Set the data of the message       #Note that in our case data is the only member of String class
        msg.data = "Hello"                  #But in other cases there could be more than one member
                                            #Like for example in the case of the Pose message type (You will find it in odomerty topic)
                                            #There are 3 members: position, orientation and covariance
        # This is just to print a message to the terminal
        rospy.loginfo("Start publishing")

        # Publish the message through the publisher object created before
        pubObj.publish(msg)

        # Put node to sleep for a certain time that is defined by the rate
        rate.sleep()
```
- We need to make the file executable To do that, open the terminal and type the following command:

``` bash
chmod +x {pkg path}/{file name}
chmod +x ~/catkin_ws/src/simple_pkg/scripts/simple_publisher.py
```
- To run the node, First start master node by typing the following command:
``` bash
roscore
```
- Now you can run the node by typing the following command:
``` bash
rosrun <pkg name> <node name>
```

## Simple ROS Subscriber Example
- `simple_subscriber.py`:
``` python
#!/usr/bin/env python3

# Import the rospy package: this almost always needs to be done
# Be ware that almost any thing that will be imported need to be added to the package.xml file first
import rospy

# Import the message type that you want to publish
# Note that the message type should be added to the package.xml file first
# Message type imported should match the message type that the publisher is publishing
# In this case, the message type is std_msgs/String
# You can identify message type also form the topic info
from std_msgs.msg import String

# What is a subscriber?
# A subscriber is a node that will subscribe to a topic and will get messages from it

# This is the callback function that will be called when the subscriber receives a message
# msg: the message that is received
def simple_callback(msg):
    # This is just to print a message to the terminal
    rospy.loginfo("Message received: " + msg.data)

if __name__ == '__main__':
    # Initialize the node: this is lke a constructor for an object
    # TO intialize you need to provide a name for the node 
    # Such name could be different from the package name and the file name
    rospy.init_node("Simple_Subscriber")

    # This is just to print a message to the terminal
    rospy.loginfo("Simple_Subscriber node has been initialized")

    # This is just to limit the rate of the node per second
    # It is not neccessary to be equal to publisher rate
    # This not neccesary to do but it is a good practice for cpu usage
    rate = rospy.Rate(5)

    # Create a subscriber object that will subscribe to the topic "simple_topic"
    # A topic: is a channel that is used to publish and subscribe to messages (like message queues in other languages)
    # We don't need to create the topic first, the publisher will create it automatically
    # The second argument is the message type that will be received
    # Message type that will be received is String
    # The third argument is the callback function that will be called when a message is received
    # Hopefully this much is sufficient for our case
    subObj = rospy.Subscriber("simple_topic", String, callback = simple_callback)

    # This is like an infinite loop that keeps node running
    # Why different from publisher?
    # Publisher needs to keep running to perform its logic then publish results
    # On the other hand, subscriber only needs to run occasunaly to check if there is a new message
    # If so it will receive it and call the callback function to perfrom any required logic
    # Then it will go back to sleep until the next time it needs to check for new messages
    # Note: Not literally sleep, but it will not run the logic until the next time
    # For this spin is better than while loop
    rospy.spin()
```
- We need to make the file executable To do that, open the terminal and type the following command:

``` bash
chmod +x {pkg path}/{file name}
chmod +x ~/catkin_ws/src/simple_pkg/scripts/simple_subscriber.py
```
- To run the node, First start master node by typing the following command:
``` bash
roscore
```
- Now you can run the node by typing the following command:
``` bash
rosrun <pkg name> <node name>
```

## Useful ROS Commands
- To list all the topics:
```bash
rostopic list
rostopic {topic name} info
rostopic echo {topic name}

rosmsg list
rosmsg info {message type}
```
## Credit:
This guide is heavily based on the work of [Ziad Atef](https://github.com/ziad-atef/ziad-atef).