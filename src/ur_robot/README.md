The ur_robot folder contains 3 main folders:<br/>
==> universal_robot: Contains the drivers and controllers for the robot.<br/>
==> ur10_robot: Contains a c++ API for controlling the robot using URscript like commands or<br/>
                general high level commands like reading joint position, velocities, wrenches etc,<br/>
                and sending position and velocity commands.<br/>
==> utils_lib: Contains some general purpose IO utilities-functions.<br/>
<br/>
*** ur10_robot *** <br/>
- ur10_robot/include/ur10_robot/ur10_robot.h: the c++ API (with documentation) for controlling the robot. <br/>
- ur10_robot/src/ur10_robot.cpp: contains the c++ API implementation. <br/>
- ur10_robot/config: contains the configuration files for the ur10 robot and optoforce. <br/>
  You can change the name of the topics for the ur10 robot or the callibration parameters of <br/>
  the optoforce sensor. <br/>
- ur10_robot/launch: contains launch files for launching the optoforce (optoforce.launch), <br/>
  the ur10 controller (ur10_controller.launch), the ur10's urdf model in rviz (ur10_urdf.launch), <br/>
  or launch all of them together (ur10_optoforce_rviz.launch). <br/>
