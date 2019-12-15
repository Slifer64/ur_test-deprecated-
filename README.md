# ur_ws
Universal Robots API in c++ for controlling the ur10 robot.

## General
Provides an API in c++ for controlling the ur10 robot in a synchronous way. Provides also
URscript like commands. Specifically, it contains:<br/>
==> a package for ur10 robot with a c++ API for controlling the robot.<br/>
==> a package for optoforce sensor.<br/>
==> a package for some useful IO utilities-functions.<br/>
==> a package with test nodes for the ur10 robot API.<br/>

## Install dependencies
To install any required dependencies run:
```
cd ur_ws/
./install.sh
```

## Build
To build all packages in the workspace run:
```
cd ur_ws/
./build.sh
```

## Running the tests
Start the ur simulator.<br/>
To launch a test node (e.g. test_logging):
```
cd ur_ws/
source devel/setup.bash
roslaunch ur10_test test_logging.launch
```
To plot the logged data open matlab set the workspace to ur_ws/test/ur10_test/matlab/ and run:
```
load_logged_data.m
plot_logged_data.m
```
See also the explanations in [README.md](src/test/ur10_test/README.md) in ur10_test folder.

## Authors

* **Antonis Sidiropoulos** - [Slifer64](https://github.com/Slifer64)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
