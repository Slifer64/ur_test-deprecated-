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
#include <ur_robot/ur_robot.h>

using namespace as64_;

int main(int argc, char** argv)
{
  // ===========  Initialize the ROS node  ==================
  ros::init(argc, argv, "test_delay");

  // ===========  Create robot  ==================
  std::shared_ptr<ur_::Robot> robot;
  robot.reset(new ur_::Robot());

  arma::vec q;
  arma::vec q_start = {4.2,  -1.73,  -2.2,  -0.81,   1.6,  -0.03};
  arma::vec q_end = {3.2,  -1.43,  -1.8,  -0.81,   1.6,  -0.03};

  std::cerr << "q_start = " << q_start.t() << "\n";
  std::cerr << "q_end = " << q_end.t() << "\n";

  // Set the robot in position control mode
  robot->setMode(ur_::Mode::POSITION_CONTROL);
  std::cout << io_::bold << io_::green << "=== The robot is in position control ===\n" << io_::reset;

  if (!ros::ok()) exit(-1);

  robot->waitNextCycle();
  q = robot->getJointsPosition();
  double duration = std::max(arma::max(arma::abs(q_start-q))*14.0/3.14159, 6.5);
  std::cout << io_::bold << io_::cyan << "The robot will move to its initial pose in " << duration << " sec.\n" << io_::reset;

  robot->setJointsTrajectory(q_start, duration);
  std::cout << io_::bold << io_::cyan << "Initial pose reached!\n" << io_::reset;
  if (!ros::ok()) exit(-1);
  
  robot->waitNextCycle();
  q = robot->getJointsPosition();
  duration = std::max(arma::max(arma::abs(q_end-q))*14.0/3.14159, 6.5);
  std::cout << io_::bold << io_::cyan << "The robot will move to its final pose in " << duration << " sec.\n" << io_::reset;
  robot->startLogging();
  robot->setJointsTrajectory(q_end, duration);
  robot->stopLogging();
  std::cout << io_::bold << io_::cyan << "*** Final pose reached! ***\n" << io_::reset;

  robot->saveLoggedData(ros::package::getPath("ur_test") + "/data/logged_data.bin");
  // ===========  Shutdown ROS node  ==================
  // ros::shutdown();

  return 0;
}
