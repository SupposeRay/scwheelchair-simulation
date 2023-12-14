# Shared Control Wheelchair Simulation Model

This folder includes a simple wheelchair model and a sensor library for simulation purpose of RRIS Shared Control Wheelchair.

Maintained by:

- Kuan Yuee kuanyuee.tan@ntu.edu.sg

---

## Model

The `model_scwheelchair` package contains a simple differential drive wheelchair model.

![](img/base_model.png?raw=true "model_scwheelchair" )

The base model description, model properties, and gazebo plugins are located in `/model_scwheelchair/urdf/base_model_scwheelchair/` with trailing underscores `xxx_.xacro`. Modifying these files are **NOT RECOMMENDED** as the design and properties are measured and usually fixed.

Instead, refer to section [Usage](#Usage) below for usage and modification guide.



## Sensors

This package includes a collection of sensors for gazebo simulation. The usage of the sensors has been organized into macros in `/sensors/sensor_macro.xacro` for the ease of implementation.

Currently available sensors:

- HC-SR04 ultrasonic sensor

- RealSense D435 depth camera

- RPLIDAR A3 2D lidar

- Velodyne VLP-16 3D lidar

Please refer to section [Usage](#Usage) for details of including the sensors in urdf file.

It is **STRONGLY DISCOURAGED** to modify the parameters of the sensors as they are carefully tweaked to match the real properties of the physical sensors. 

Adjustable parameters are already made available in the sensor macros, see section [Usage](#Usage) for detailed steps.

## Usage

This section describes a recommended usage and modification of the models for better housekeeping.

In package `model_scwheelchair/launch` folder, the `gazebo.launch` file was created as a template file to spawn robot into gazebo environment.

```
roslaunch model_scwheelchair gazebo.launch
```

To launch the base model with example of sensors usage

```
roslaunch model_scwheelchair gazebo.launch xacro_file:=model_scwheelchair_sensors.xacro
```

### Modification

#### URDF

The URDF `/model_scwheelchair/model_scwheelchair.xacro` file was created and meant to be the template of further addition on the wheelchair. This file already included the wheelchair base model and sensor macros.

It is recommended to create a new xacro file and simply include the minimal base model `model_scwheelchair.xacro` when making different setups as shown below.

```
<?xml version="1.0" ?>

<robot name="model_scwheelchair" xmlns:xacro="https://www.ros.org/wiki/xacro">

    <xacro:include filename="$(find model_scwheelchair)/urdf/model_scwheelchair.xacro"/>

    <link name="new_link">
        <visual>...</visual>
        <collision>...</collision>
        <inertial>...</inertial>
    </link>

    <joint name="new_joint"  type="fixed">
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="new_link"/>
    </joint>

</robot>
```

#### Adding Sensors

In order to add sensors that are available in the sensors package, simply include the minimal base model `model_scwheelchair.xacro` and use the macro tag to include the sensor. Below is an example of adding a RPLidar to the wheelchair.

A joint to the sensor will be auto created in the macro, thus only need to specify the pose and orientation to the parent link as parameters. 

Adjustable parameters are also made available in the macro by providing correspoding values to it, otherwise default values will be used.

The file `model_scwheelchair_sensors.xacro` is an example of wheelchair base model with multiple sensors addition.

```
<robot name="model_scwheelchair" xmlns:xacro="https://www.ros.org/wiki/xacro">

    <xacro:include filename="$(find model_scwheelchair)/urdf/model_scwheelchair.xacro"/>

    <xacro:sensor_rplidarA3 name="lidar1" 
                            parent="base_link" 
                            x="0.3" y="0" z="0.15" 
                            roll="0" pitch="0" yaw="0"
                            topic_name="scan"
                            min_angle="-2.3562" max_angle="2.3562"  
                            visualize="false"
    />

</robot>
```

#### Launch Files

Once the new modified xacro file is created, it is recommended to use `gazebo.launch` file as a template to spawn robot into gazebo simulation.

- model_name : robot model name

- xacro_file : the full xacro file name in `model_scwheelchair/urdf` folder with extension

- world_file : gazebo .world file full path, default will be empty world if this arg is left blank

- xyzrpy : initial spawn position of the robot

To spawn robot in using other launch files, include `gazebo.launch` in other launch files.

```
<launch>

    <include file="$(find model_scwheelchair)/launch/gazebo.launch">
        <arg name="xacro_file"  value="your_xacro_file.xacro"/>
        <arg name="world_file>  value="your_world_file.world"/>
    </include>

    <!-- other nodes and launch files -->

</launch>
```

### Note for Navigation Usage

- The robot dimension and mass properties are specified in `model_scwheelchair/urdf/base_model_scwheelchair/model_properties.xacro`. By default the robot is very light in weight.

- The robot differential drive plugin is specified in `model_scwheelchair/urdf/base_model_scwheelchair/model_scwheelchair_.gazebo`.

- Command velocity topic is `/cmd_vel`.

- Odometry frame is `odom`.

- The robot base frame is `base_footprint` which is located at the middle of left and right front wheel on the floor.

    ![](img/base_footprint.png?raw=true "base_footprint" )

- TF tree

    ![](img/tf_tree.png?raw=true "tf_tree")

## Disclaimer

This wheelchair model is just a very simple design without proper physics properties setting. 

Suggestion for different kind of sensors are appreciated in order to increase the number of sensors collection in sensors library.

All feedback and recommendations are welcomed to improve the robot behaviour in simulation.

