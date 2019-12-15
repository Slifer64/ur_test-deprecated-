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
  ros::init(argc, argv, "simple_control");

  // ===========  Create robot  ==================
  std::shared_ptr<ur10_::Robot> robot;
  robot.reset(new ur10_::Robot());

  // Set the robot in freedrive mode
  robot->setMode(ur10_::Mode::FREEDRIVE_MODE);
  std::cout << io_::bold << io_::green << "=== The robot is in freedrive mode ===\n" << io_::reset;

  std::cout << io_::blue << "Move the robot wherever you want and press enter to register the starting pose...\n" << io_::reset;
  std::string tmp_str;
  std::getline(std::cin, tmp_str);
  robot->waitNextCycle();
  arma::vec q_start = robot->getJointPosition();
  std::cout << io_::cyan << "Registered start pose\n" << io_::reset;

  std::cout << io_::blue << "Move the robot wherever you want and press enter to register the final pose...\n" << io_::reset;
  std::getline(std::cin, tmp_str);
  robot->waitNextCycle();
  arma::vec q_end = robot->getJointPosition();
  std::cout << io_::cyan << "Registered final pose\n" << io_::reset;

  std::cout << io_::blue << "Press enter to enter position control and make the robot move to the start pose...\n" << io_::reset;
  std::getline(std::cin, tmp_str);

  // Set the robot in position control mode
  robot->setMode(ur10_::Mode::POSITION_CONTROL);
  std::cout << io_::bold << io_::green << "=== The robot is in position control ===\n" << io_::reset;

  robot->waitNextCycle();
  std::cout << io_::bold << io_::cyan << "The robot will move to its initial pose in 8 sec.\n" << io_::reset;
  robot->setJointsTrajectory(q_start,8);
  std::cout << io_::bold << io_::cyan << "Initial pose reached!\n" << io_::reset;

  robot->waitNextCycle();
  // Set the robot in velocity control mode
  robot->setMode(ur10_::Mode::VELOCITY_CONTROL);
  std::cout << io_::bold << io_::green << "=== The robot is in velocity control ===\n" << io_::reset;

  std::cout << io_::blue << "Moving the robot with velocity control to the final pose...\n" << io_::reset;
  double a = 0.001;
  double s = 0.0;
  double s_end = 1.0;
  while (ros::ok())
  {
    robot->waitNextCycle(); // wait for the robot to update its state

    // read robot state
    arma::vec q = robot->getJointPosition();
    // optionally...
    // arma::vec wrench = robot->getTaskWrench();
    // arma::vec p_tool = robot->getTaskPosition();
    // arma:vec Q_tool = robot->getTaskOrientation();
    // arma::vec Pose_tool = robot->getTaskPose();
    // arma::vec twist = robot->getTaskVelocity();

    // check if final pose has been reached
    if (arma::norm(q-q_end) < 0.02) break;

    // to make the velocity increase smoothly
    arma::vec dq = s*(q_end-q);
    // apply threshold for safety
    arma::uvec ind = arma::find(arma::abs(dq)>0.3);
    dq.elem(ind) = arma::sign(dq.elem(ind))*0.3;

    robot->setJointVelocity(dq);

    s = (1-a)*s + a*s_end;
  }

  std::cout << io_::bold << io_::green << "*** Final pose reached! ***\n" << io_::reset;

  // ===========  Shutdown ROS node  ==================
  ros::shutdown();

  return 0;
}
