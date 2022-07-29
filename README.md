# ros2cv
Practical implementation of a publisher and subscriber with OpenCV.



# Install ROS2

Follow the instructions on the official doc: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


# Create a Workspace

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

>> cd ~
>> mkdir -p ~/dev_ws/src


Navigate to the workspace src subdirectory, and clone

Hint: Ensure you’re in the dev_ws/src subdirectory before you clone.

>> cd ~/dev_ws/src
>> git clone https://github.com/emmbiz/ros2cv.git -b main


Resolve package dependencies with rosdep (because of possible missing dependencies)

Hint: Ensure you’re in the root workspace directory (~/dev_ws) before you run rosdep. 

>> rosdep install --from-paths src --rosdistro <distro> -y
e.g.
>> cd ~/dev_ws
>> rosdep install -i --from-path src --rosdistro humble -y

# Source ROS 2 Workspace

>> source /opt/ros/<distro>/setup.bash
e.g.
>> source /opt/ros/humble/setup.bash


# Build

5) Build the workspace with Colcon

Hint: Make sure you are in the root directory of your workspace (~/dev_ws).

>> cd ~/dev_ws
>> colcon build --packages-select cv_node


i) In a new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it.

>> source /opt/ros/humble/setup.bash

ii) Go into the root of your workspace, and then source your overlay (the local or secondary workspace)

>> cd ~/dev_ws
>> . install/local_setup.bash

- Run the Publisher

>> ros2 run cv_node talker


iii) Open a new terminal again, and source your main ROS 2 environment, and your local workspace.

>> source /opt/ros/humble/setup.bash
>> cd ~/dev_ws
>> . install/local_setup.bash

- Run the Subscriber

>> ros2 run cv_node listener

