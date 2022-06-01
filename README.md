# StretchSenseROS

This package is used to connect to a StretchSense peripheral via Bluetooth and publish its sensor data to the ROS network.

## Installation

### 1. Inside the `src` folder of your [ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace) clone with:
```
$ git clone git@github.com:alvee1994/stretchSenseROS.git stretchsense
```

### 2. Install dependencies
```
$ pip install -r requirements.txt
```

### 3. Optional: In order to use `bluepy` without `sudo`

Find the module directory by creating and running the following Python script:
```python
import bluepy
print(bluepy.__file__)
```
Go to the directory and run
```
$ sudo setcap 'cap_net_raw,cap_net_admin+eip' bluepy-helper
```

## Usage

### 1. Start roscore
```
$ roscore
```

### 2. Launch the desired visualiser
```
$ roslaunch stretchsense smartGloveModelLeft.launch
```
OR
```
$ roslaunch stretchsense smartGloveModelRight.launch
```
### 2a. If needed, change **Fixed Frame** in the left sidebar to "<left/right>_metacarpal"

### 3. Launch the application 
```
$ roslaunch stretchsense smartGloveApplication.launch
```
