## Hardware Requirements

To use this package you need a Care-O-bot light modul connected via USB.

## ROS API

The cob_light package provides a node that
connects to the light hardware board and offers a message interface over
[topics](Topics). The current state of the light is published as a
[visualization marker](https://index.ros.org/p/rviz).

### cob_light.py

The `cob_light.py` node takes in
[std_msgs/msg/ColorRGBA](https://docs.ros.org/en/rolling/p/std_msgs/msg/ColorRGBA.html)
messages and send this directly to the hardware. If no hardware is
present, the node runs in a simulated mode only publishing
[visualization markers](https://index.ros.org/p/rviz).

#### Subscribed Topics

- `command`
  ([std_msgs/msg/ColorRGBA](https://docs.ros.org/en/rolling/p/std_msgs/msg/ColorRGBA.html)):
  Receives color commands in RGBA format.

#### Published Topics

- `marker`
  ([visualization_msgs/msg/Marker](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/Marker.html)):
  Publishes the current state of the light.

#### Parameters

- `~devicestring` (`string`, default: `Required`): Device identifier for
  hand, e.g. /dev/ttyUSB3
- `~baudrate` (`int`, default: `Required`): Baudrate of serial
  interface, e.g. 230400

## Usage/Examples

The package can be used via a launch file which loads all parameters and
starts the light node. For starting the light use


```
roslaunch cob_bringup light.launch
```

 For including the light in your overall launch
file use 

```
<include file="$(find cob_bringup)/components/light.launch" />
```

 A sample parameter file could look
like this 

```
devicestring: /dev/ttyUSB3
baudrate: 230400
```



For testing the light you can start a test_script which changes the LEDs
to different colors. 

```
rosrun cob_light test.py
```
