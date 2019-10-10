# rrt_turtlebot
The turtlebot RRT package 


This is a project on known environment exploration using a TurtleBot.
The algorithm used is RRT (rapidly exploring random tree), a popular algorithm
for autonomous robotic planning. The proposed project will have many applications – the robot
can be used for domestic household work, used as a butler and can also be extended to pick
and place objects by adding arms to the turtle bot and use RRT to control the arms.

## License

This program is under MIT License. A copy of the license can be obtained from [here](https://github.com/PranavDhulipala/rrt_turtlebot/LICENSE) 

Copyright (c) 2019 Pranav Dhulipala
```bash
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```


## Dependencies

To run this program you need to have the following installed on your system:

    Ubuntu (Wily/Xenial) or Debian (Jessie)
    ROS Kinetic
    Gazebo 7.x (part of ros-kinetic-desktop-full package)
    Turtlebot simulation stack
    Turtlebot interactions




## Build Instructions

```
mkdir -p ~/catkin_ws/
cd catkin_ws/
git clone https://github.com/PranavDhulipala/rrt_turtlebot.git src
catkin_make
source devel/setup.bash

```

## Running the demo

```
cd src/scripts/
chmod +x test.sh
./test.sh
```

## Running the tests

```
cd ~/catkin_ws/
source devel/setup.bash
cd build/
make run_tests
```

