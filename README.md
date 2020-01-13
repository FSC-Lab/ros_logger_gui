# ROS-logger-GUI
Simple graphical interface to control Rosbag recording.

Current only compatible with ROS kinetic as it is dependent on Qt-Ros and Qt-Create. 

## Launching the GUI
Ensure the package is built and its `devel/setup.bash` has been sourced. Then launch the interface by the following command
```
rosrun ros_logger_gui
```

If a default list of topic is configured, launch the interface by the following command
```
roslaunch ros_logger_gui ros_logger_gui.launch
```

## Default Topic List
Edit `config/topics.yaml` and list desired topics as a yaml list of strings. Examples below show acceptable formats

**Single line array**
```
topics : ["/mavlink/from", "/mavros/battery", "/diagnostics"]
```

**List**
```
topics : 
  - "/mavlink/from" 
  - "/mavros/battery"
  - "/diagnostics"
```

Even if one or more listed topics are non-existent, or if data is not published, recording of other published data will not be affected.
