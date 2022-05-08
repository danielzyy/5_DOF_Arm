# 5_DOF_Arm
Control and simulation code for a 5 degree of freedom robotic arm.

# Arduino Prototype
* 4 servos controlled with PCA9685 servo expander capable of speed adjustments
* Three 180° servo motors for planar joints
* One continous servo with optical rotary disk encoder to simulate base rotation
* Real-time motor control using IMU
<img src="https://user-images.githubusercontent.com/15254803/167279164-ddecfc67-f772-4ac4-95c0-79078480c0e9.gif" height="400" />

## 2D Python Simulation
Simulation created using pygame to simulate the 2d Inverse Kinematics (IK) and Forward Kinematics (FK) of the three joints (pivots), with fixed link lengths matching the actual link lengths.
The angle of the end effector (measured CCW from horizontal) can be calculated automatically in General IK mode, or set to a fixed angle in the Fixed Angle IK mode, and the arm becomes greyed out when the mouse is placed in an unreachable position in the current configuration.
The modes may be changed by pressing they keys: 1 = General IK Mode, and 2 = Fixed Angle IK Mode.

### General IK Mode
Capable of determining the valid end effector angle to reach the desired mouse (x, y) location.

<img src="https://user-images.githubusercontent.com/15254803/166200032-c8abb8ea-b909-4e58-ba0a-b415672ab630.gif" height="400" />

### Fixed Angle IK Mode
Capable of setting a fixed end effector angle to maintain throughout all of the robot's motions. Angle can be changed using up and down arrow keys.

<img src="https://user-images.githubusercontent.com/15254803/166200056-cecf7c42-24cf-4a46-9a56-09dffa06ebbf.gif" height="400" />
