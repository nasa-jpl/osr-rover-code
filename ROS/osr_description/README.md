# osr_description

This package contains files relating to the URDF of the OSR robot which can be used for visualization in RViz or Gazebo, collision checking, navigation, extrinsic calibration, and so on.

## Generating the URDF

[onshape-to-robot](https://onshape-to-robot.readthedocs.io/) is used to convert the OnShape model to URDF format. You'll need to give this package [access to the OnShape model through the OnShape API](https://onshape-to-robot.readthedocs.io/en/latest/installation.html#setting-up-your-api-key).

In this folder, run:

```bash
onshape-to-robot urdf
```

## Visualizing the URDF

This is intended to be run from a separate computer with access to a screen so RViz can be opened.

### if you already have ROS 2 installed

In this case you won't need to build the docker container. In a (new) workspace:

```
ros2 launch osr_description osr_rviz.launch.py
```

RViz should launch showing the robot model.

### if you want to use docker with graphics forwarding

```bash
bash build.sh
bash run.sh
cd ../..
colcon build --symlink-install
ros2 launch osr_description osr_rviz.launch.py
```

RViz should launch showing the robot model.
