#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <exception>

#include <armadillo>

#include <ros/ros.h>
#include <ros/package.h>

#include <io_lib/io_lib.h>
#include <ur10_robot/ur10_robot.h>


int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "test_logging_in_freedrive_mode");

  std::string data_filename = ros::package::getPath("ur10_test")+ "/data/logged_data";
  bool data_logging = true;
  bool binary = true;
  int precision = 7;

  // ===========  Create robot  ==================
  std::shared_ptr<ur10_::Robot> robot;
  robot.reset(new ur10_::Robot());

  // Set the robot in freedrive mode
  robot->setMode(ur10_::Mode::FREEDRIVE_MODE);
  std::cout << io_::bold << io_::green << "The robot is in freedrive mode.\n" << io_::reset;

  std::cout << io_::blue << "Press enter to start data logging...\n" << io_::reset;
  std::string tmp_str;
  std::getline(std::cin, tmp_str);

  robot->startLogging();

  // ===========  Launch thread for printing  ==================
  // optional...
  // robot->launch_printRobotStateThread(1); // print with frequency 1 Hz

  std::cout << io_::blue << "Press ctrl+C to stop data logging...\n" << io_::reset;
  while (ros::ok())
  {
    robot->waitNextCycle(); // wait for the robot to update
  }

  robot->stopLogging();
  std::cout << io_::blue << "Stopped data logging...\n" << io_::reset;

  std::cout << io_::blue << "Saving logged data to file: \"" << data_filename << "\"\n";
  robot->saveLoggedData(data_filename, binary, precision);

  // robot->stop_printRobotStateThread();

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
