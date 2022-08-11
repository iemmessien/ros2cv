# ros2cv
Practical implementation of a publisher and subscriber with ROS2 and OpenCV.


# Install ROS 2

Follow the instructions on the official doc to [install ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).


# Create a Local Workspace

```
>> cd ~
>> mkdir -p ~/dev_ws/src
```

Navigate to the workspace src subdirectory, and clone. (Hint: Ensure you are in the dev_ws/src subdirectory before you clone.)

```
>> cd ~/dev_ws/src
>> git clone https://github.com/emmbiz/ros2cv.git -b main
```

Resolve package dependencies with rosdep because of possible missing dependencies. In the root workspace directory, run rosdep. 

```
>> cd ~/dev_ws
>> rosdep install -i --from-path src --rosdistro humble -y
```

# Build the Package

Source the ROS 2 environment.

```
>> source /opt/ros/humble/setup.bash
```

Build the workspace with colcon. (Hint: cd to the root directory of your workspace, and then build.)

```
>> cd ~/dev_ws
>> colcon build --packages-select cv_node
```

# Run
i) In a new terminal, source the ROS 2 environment; the underlay.

```
>> source /opt/ros/humble/setup.bash
```

ii) Go into the root of your local workspace, and source the overlay.

```
>> cd ~/dev_ws
>> . install/local_setup.bash
```

iii) Run the Publisher.

```
>> ros2 run cv_node talker
```

iv) Open a new terminal again, and source your ROS 2 environment, and your local workspace as above.

```
>> source /opt/ros/humble/setup.bash
>> cd ~/dev_ws
>> . install/local_setup.bash
```


v) Run the Subscriber.

```
>> ros2 run cv_node listener
```


# Result
![screenshot](https://user-images.githubusercontent.com/87937713/184142670-4a2813a2-91e2-425d-a650-09046885b408.png)

