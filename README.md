# edge-ai-ros-inventory-demo

## Getting started

This guide assumes that you have a ROS1 docker container with all of the necessary demo code installed.
If that is not the case, please follow _TODO: this guide._

To log in to the robot, set up the wifi router and log in over wifi:

```
# no password - if this doesn't work you have the wrong address!
ssh root@192.168.50.200
```
or log in via the serial terminal

```
# if you have no other usb devices, it's likely USB2
screen /dev/ttyUSBX 115200
login: root
```

### Calibrating the scene:

(TODO - READ ME AND DETERMINE MY ACCURACY)

First the AprilTag must be placed above the ground, level and parallel to the floor, face down.
Then, measure the distance (in meters) from the face of the AprilTag to the floor.

This value must be placed in the file `/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/launch/robot_tf.launch`

open the file and edit the line below

`<node pkg="tf" type="static_transform_publisher" name="map_to_apriltag21" args="0 0 <distance> 0 0 3.14159 map apriltag21 100" />`

Replace `<distance>` with your value (in meters), ideally up to two decimals in precision.


### Setting the waypoints

First - you must launch the docker container:

```
# This command takes a long time, please be patient!
/home/root/j7ros_home/tda4_docker_run.sh
```

Then run the calibration code:

`roslaunch d3_apriltag point_calib.launch`

It will ask you what you would like to do.

Place the robot at a desired waypoint and enter `savepose`.  It will then ask you what file to save to.
Enter the suggested value of `points.json`. It will then ask you what you would like to name the point. 
Currently, the demo objectives are configured for the following points, in order:

* `point_a`
* `point_b`
* `point_c`
* `point_d`
* `crowd`

Use these point names if you don't want to edit the objectives file. Continue placing the robot at the
desired locations and saving the pose until you've captured all of the ones you need for your demo.


Note: you may re-calibrate any individual point as long as it successfully saves the file.

### Running the demo

If you are not already in the docker container you must launch it:
`/home/root/j7ros_home/tda4_docker_run.sh`

You'll want to run a roscore so when the demo _invariably crashes_, your visualizer doesn't:
`roscore &`

Assuming you've calibrated the device - run the demo:
`roslaunch d3_inventory_demo demo_full.launch`

###### WARNING:

`d3_inventory_demo demo_full.launch` has issues being run a second time.  This has to do with the front facing camera - I don't fully understand it.

If you're just testing the demo and you don't care about the front-facing camera, run this launch file: (TODO)

If obstacle avoidance is not working, run this launch file: `d3_inventory_demo demo.launch`


### Visualizing the demo

To run the visualizer, do the following:

1. Turn on the Visualizer PC and connect to the network
2. Open a Terminal (Ctrl+Alt+T)
3. Run the command `~/j7ros_home/pc_docker_run.sh`
4. Wait for Docker to Start
5. Run the command `source devel/setup.bash`
6. Once the `roscore` has been launched on the Robot, run `roslaunch d3_inv_viz inventory_viz.launch` on the Visualizer PC
7. The Visualizer should appear. If it does not, check the console for errors. The error `unable to contact ROS master` means that the Robot was unreachable. Check that you can ping the robot at `192.168.50.200` and relaunch the Visualizer. Other error messages may occur related to the window manager, they can be ignored and you can relaunch the Visualizer

## Advanced configuration

Below is configuration that is not necessary to get the demo online, but if you want to change
how the demo works at all, this section may help you.

### Changing demo objectives

The demo is configurable with JSON files in the directory `/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/`
The demo works by having "objectives" that it accomplishes at particular waypoints. By default, it will read the `objectives.json` file
and the `points.json` file, then figure out at what points the objectives need to happen. Below is an example of the configuration file:

```
{
  "Shelf A": {
    "sequence": 10,
    "point_name": "point_a",
    "action": "SCAN",
    "action_info": {
      "number_targets": 1
    }
  },
  "Shelf B": {
    "sequence": 20,
    "point_name": "point_b",
    "action": "SCAN",
    "action_info": {
      "number_targets": 1
    }
  },
  "Shelf C": {
    "sequence": 30,
    "point_name": "point_c",
    "action": "SCAN",
    "precise_goal": true,
    "action_info": {
      "number_targets": 1
    }
  },
  "Shelf D": {
    "sequence": 40,
    "point_name": "point_d",
    "action": "SCAN",
    "precise_goal": true,
    "action_info": {
      "number_targets": 1
    }
  },
  "the Audience": {
    "sequence": 50,
    "point_name": "crowd",
    "action": "TRACK",
    "action_info": {
      "duration": 30
    },
    "precise_goal": false
  }
}
```

Each objective starts with its display name (this is what is displayed on the Visualizer)
in this file there are Five: "Shelf A", "Shelf B", "Shelf C", "Shelf D", and "the Audience"

The rest of the fields mean the following:

* `sequence`: Used to order objectives.  The lowest sequence number will be run first, the last sequence number will be run last
* `point_name`: The name of the point that will be used as a waypoint.  If left empty - it will default it will use the name of the objective.
* `action`: what to do when the robot has reached the waypoint.  Below are the following options:
  * `NOOP`: Do nothing when you get there
  * `SCAN`: Scan the datamatrix tags from the left camera
  * `TRACK`: Run the tracker & object identifier
  * `DONE`: Stop when you reach this point
* `precise_goal`: When set to "true", the robot will try to get to the target point with very little error before performing its' action.
  When set to false, it will perform the action immediately after reaching the destination
* `action_info`: Data array that has action specific information based on what action you're performing.  Must be set with the appropriate data
  * `SCAN`: Requires field `number_targets` - how many targets the visualizer should expect to see - if it sees less than this, it will take another image and run again with increased processing time. For a more fluid demo, set to `1`
  * `TRACK`: Requires field `duration` - how long (in seconds) it will sit there before moving on to the next objective
  * `NOOP`: No fields are used
  * `DONE`: No fields are used


WARNING: each objective MUST HAVE A CORRESPONDING POINT - if it doesn't then the demo will crash and raise the following error:
`"WARNING: Objective '" + obj.name + "' has no corresponding point named '" + obj.point_name + "' - removing..."`
This means you need to either: add a point to points.json via calibration, fix the corresponding point name in the objective, or remove the objective

