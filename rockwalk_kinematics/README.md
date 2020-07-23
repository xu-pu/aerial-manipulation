# Kinematics

Based on the model for a falling rolling disk.

 ![](media/rockwalk_rviz.gif)

**INPUT**
Subscribes to topics from an IMU
1. Orientation @
2. Angular Velocity @

**OUTPUT**
Publish following topics
1. Euler angles to appropriate for rolling analysis `@ euler_ginsberg`
2. `@ twist_ginsberg`
3. `@ object_pose`
4. Coordinate of ground contact `@ ground_contact_coordinates`

*TODO: Add video/gif for output*

## Pre-requisites

### Hardware:
* 9-axis motion shield affixed to Arduino Mega. But of course any other IMU is ok.
We only subscribe to orientation of the object (quaternion) and the angular velocity (in degrees).


### Software:
* `rosserial`


## Installation
Upload the code in `/arduino` to Arduino board equipped with a 9-axis motion shield.

Simply run the launch file

```
roslaunch rockwalk_kinematics rw_kinematics.launch
```
