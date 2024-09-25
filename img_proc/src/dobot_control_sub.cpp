#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "mg400_bringup/MovL.h"

ros::Publisher pub;  // Declare a global publisher
ros::ServiceClient dobot_move_client_;  // Declare a global service client

// Initialize a global variable to keep track of the published number
int publish_counter = 1;

// Function to send Vec4f data to the robot
bool dobotMoveCommandMsg(float x, float y, float z, float r)
{
    mg400_bringup::MovL commandMsg;

    commandMsg.request.x = x;
	commandMsg.request.y = y;
	commandMsg.request.z = z;
	commandMsg.request.r = r;

	// Execute the service call
    if (dobot_move_client_.call(commandMsg)) {
        ROS_INFO("Command sent successfully.");
        return true;
    } else {
        ROS_ERROR("Failed to call service dobot_move_command");
        return false;
    }
}

// Callback function for the subscriber
void vec4fCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) 
{
    // Create a cv::Vec4f from the message data
    if (msg->data.size() == 4)  // Need exactly 4 elements
    {
        cv::Vec4f vec(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);

        // Display the Vec4f data
        std::cout << "Received Vec4f data: [" 
                  << vec[0] << ", " << vec[1] << ", " 
                  << vec[2] << ", " << vec[3] << "]" << std::endl;

        // Send the Vec4f data to the robot
        if (dobotMoveCommandMsg(vec[0], vec[1], vec[2], vec[3])) {
            // If the service call was successful, publish the incremented number
            std_msgs::Int32 int_msg;
            int_msg.data = publish_counter;  // Publish the current counter value
            pub.publish(int_msg);  // Publish the number
            publish_counter++;  // Increment the counter for the next callback
        }
    }
    else
    {
        ROS_WARN("Invalid data size received. Expected 4 elements.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vec4f_subscriber_node");
    ros::NodeHandle nh;

    // Initialize the service client (service name needs to match the actual service)
    dobot_move_client_ = nh.serviceClient<mg400_bringup::MovL>("/mg400_bringup/srv/MovL");

    // Create a subscriber
    ros::Subscriber sub = nh.subscribe("vec4f_data", 10, vec4fCallback);

    // Create a publisher to publish integers
    pub = nh.advertise<std_msgs::Int32>("int_data", 10);

    ros::spin();  // Keep the node running and processing callbacks

    return 0;
}
