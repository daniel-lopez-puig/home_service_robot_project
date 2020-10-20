// process image and when it is localized, calls drive_bot service to move the robot
// subscribe image topic
// run service client to move robot

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv_request;

    srv_request.request.linear_x = lin_x;
    srv_request.request.angular_z = ang_z;

    client.call(srv_request);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    std::vector<int> white_pixel = {255,255,255};
    bool ball_detected = false;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i = 0; i < (img.height * img.step); i=i+3)
    {  
        if (img.data[i] == white_pixel[0] && img.data[i+1] == white_pixel[1] && img.data[i+2] == white_pixel[2])
        {
            // Get x coordinat
            int white_pixel_x_coordinate = i%img.step;
            ROS_INFO_STREAM("Ball detected - first_left_pixel_x_coordinate: "<<std::to_string(white_pixel_x_coordinate));

            // Orders depending on x region
            if (white_pixel_x_coordinate < img.step/2)
            {
                // Turn left
                drive_robot(0.2, 0.4);
            } else if (white_pixel_x_coordinate >= img.step/2 && white_pixel_x_coordinate < img.step*2/3)
            {   
                // Go straight
                drive_robot(0.2, 0.0);
            } else if (white_pixel_x_coordinate >= img.step*2/3)
            {   
                // Turn right
                drive_robot(0.2, -0.4);
            }
            ball_detected = true;
            break;
        }
        
    }
    // If ball not detected stop the robot and ros_info
    if (!ball_detected)
    {   
        drive_robot(0.0, 0.0);
        ROS_INFO_STREAM("Ball not detected detected");
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}