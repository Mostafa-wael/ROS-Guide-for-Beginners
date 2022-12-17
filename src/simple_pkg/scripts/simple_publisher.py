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

# Finally, we need to make the file executable
# To do that, open the terminal and type the following command:
#
# chmod +x {pkg path}/{file name}
# For example:
# chmod +x ~/catkin_ws/src/simple_pkg/scripts/simple_publisher.py

# To run the node, First start master node by typing the following command:
# roscore

# Now you can run the node by typing the following command:
# rosrun {pkg name} {node name}