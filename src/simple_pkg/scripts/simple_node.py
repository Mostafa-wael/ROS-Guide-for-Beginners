#!/usr/bin/env python3

# Import the rospy package: this almost always needs to be done
# Be ware that almost any thing that will be imported need to be added to the package.xml file first
import rospy

if __name__ == '__main__':
    # Initialize the node: this is lke a constructor for an object
    # TO intialize you need to provide a name for the node 
    # Such name could be different from the package name and the file name
    rospy.init_node("Simple_Node")

    # This is just to limit the rate of the node per second
    # This not neccesary to do but it is a good practice for cpu usage
    rate = rospy.Rate(50)

    # Now the node will start to run and continue running until it is shutdown (ctrl + c)
    while not rospy.is_shutdown():

        # This is just to print a message to the terminal
        rospy.loginfo("Hello")

        # Put node to sleep for a certain time that is defined by the rate
        rate.sleep()

# Finally, we need to make the file executable
# To do that, open the terminal and type the following command:
#
# chmod +x {pkg path}/{file name}
# For example:
# chmod +x ~/catkin_ws/src/simple_pkg/scripts/simple_node.py

# To run the node, First start master node by typing the following command:
# roscore

# Now you can run the node by typing the following command:
# rosrun {pkg name} {node name}