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