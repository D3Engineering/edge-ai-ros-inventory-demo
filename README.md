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

First the apriltag must be placed above the ground, parallel to the floor - face down.
Then, measure the distance (in meters) from the face of the april tag to the floor.

This value must be placed in the file `` (TODO: Add one file where april tag transforms exist)

open the file and edit the line below

`<node pkg="tf" type="static_transform_publisher" name="map_to_apriltag21" args="0 0 <distance> 0 0 3.14159 map apriltag21 100" />`

Replace "<distance>" with your value (in meters) - ideally up to the hundredths place in precision.


### Setting the waypoints

First - you must launch the docker container:

```
# This command takes a long time, please be patient!
/home/root/j7ros_home/tda4_docker_run.sh
```

Then run the calibration code:

`roslaunch d3_apriltag <something>`

It will ask you what you would like to do.

Place the robot at a desired waypoint and enter `savepose`.  It will then ask
you what you would like to name the point. Currently, the demo objectives are configured
for the following points, in order:

* point_a
* point_b
* point_c
* crowd
* home

Use these point names if you don't want to edit the objectives file. Continue placing the robot at the
desired locations and saving the pose until you've captured all of the ones you need for your demo.


Note: you may re-calibrate any individual point as long as it successfully saves the file.

(something about what file to save to)

### Running the demo

If you are not already in the docker container you must launch it:
`/home/root/j7ros_home/tda4_docker_run.sh`

You'll want to run a roscore so when the demo _invariably crashes_, your visualizer doesn't:
`roscore &`

Assuming you've calibrated the device - run the demo:
`roslaunch d3_inventory_demo demo_full.launch`

###### WARNING:

demo_full.launch has issues being run a second time.  This has to do with the front facing camera - I don't fully understand it.

If you're just testing the demo and you don't care about the front-facing camera, run this launch file: (TODO)

If obstacle avoidance is not working, run this launch file: (TODO)


### Visualizing the demo

(Notes from seth on how to run the visualizer)

## Advanced configuration

Below is configuration that is not necessary to get the demo online, but if you want to change
how the demo works at all, this section may help you.

### Changing demo objectives

The demo works by having "objectives" that it accomplishes at particular waypoints. By default, it will read the objectives.json file
and the points.json file, then figure out at what points the objectives need to happen. Below is an example of the configuration file:

```
{
  "home": {
    "sequence": 4,
    "action": "DONE",
    "precise_goal": false,
    "action_info": {}
  },
  "point_a": {
    "sequence": 0,
    "precise_goal": false
    "action": "NOOP",
    "action_info": {
      "number_targets": 1
    }
  "point_b": {
    "sequence": 2,
    "action": "SCAN",
    "precise_goal": true,
    "action_info": {
      "number_targets": 1
    }
  },
  "crowd": {
    "sequence": 3,
    "action": "TRACK",
    "action_info": {
      "duration": 3
    },
    "precise_goal": false
  }
}
```

Each objective starts with its' - in this file there are four: "point\_a", "point\_b", "crowd", and "home"

The rest of the fields mean the following:

* Sequence: Used to order objectives.  The lowest sequence number will be run first, the last sequence number will be run last
* Action: what to do when the robot has reached the waypoint.  Below are the following options:
  * NOOP: Do nothing when you get there
  * SCAN: Scan the datamatrix tags from the left camera
  * TRACK: Run the tracker & object identifier
  * DONE: Stop when you reach this point
* precise_goal: When set to "true", the robot will try to get to the target point with very little error before performing its' action.
  When set to false, it will perform the action immediately after reaching the destination
* action_info: Data array that has action specific information based on what action you're performing.  Must be set with the appropriate data
  * SCAN: Requires field "number_targets" - how many targets the visualizer should expect to see
  * TRACK: Requires field "duration" - how long it will sit there before moving on to the next objective
  * NOOP: No fields are used
  * DONE: No fields are used

