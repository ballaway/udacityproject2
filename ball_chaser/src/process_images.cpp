#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // Request direction
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if (!client.call(srv))
  ROS_ERROR("Failed to call service ball_chaser");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{


  bool white_ball_found = false;
  float white_pixel_location = -1;
  float rowLength = img.step;
  float linear = 0.0;
  float angular = 0.0;
  int white_pixel = 255;
  int left_edge = img.width / 3;
  int center_edge = left_edge * 2;
  int currentRow = 1;
  int endOfRow = img.step;

  // Loop through each pixel in the image and check if there's a bright white one
  for (int i = 0; i < img.height * img.step; i = i + 3) {

    if (img.data[i] == 255 && img.data[i + 1] == 255 && img.data[i + 2] == 255) {
      white_ball_found = true;
      white_pixel_location = ((img.step * currentRow) - i) / rowLength;
      // ROS_INFO_STREAM("i is " << i << " current row is " << currentRow << "white pixel location is " << white_pixel_location);

      // right
      if(white_pixel_location <= 0.475){
        linear = 0.1;
        angular = -0.3;
      }
      // center
      else if(white_pixel_location <= 0.575){
        linear = 0.4;
        angular = 0.0;
      }
      // left
      else if(white_pixel_location <= 1.0){
        linear = 0.1;
        angular = 0.3;
      }
      break;
    }

    // update current row and end of row at the end of each row
    if(i == endOfRow){
      currentRow++;
      endOfRow += img.step;
    }
  }

  if(white_ball_found){
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    drive_robot(linear, angular);
  }
  else{
    // Request a stop when there's no white ball seen by the camera
    //ROS_ERROR("Failed to locate any white pixels");
    drive_robot(0.0, 0.0);
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
