# Experiment: reconstruct signal based on fake simulated signal by exploring enviroment.



## 1) Run fake simualted signal:

In first RVIZ, subcribe to PointCloud2 topic /radio_antenna_signal. Style: flat squares. Size (m):0.05

```
python3 get_simulated_signal_color.py
```

```
python3 radio-signal-publisher.py
```

## 2) Run signal mapper abd subsignal mapper to create semantic map:

```
ros2 run era_5g_network_signal_mapper_ros2 signal_mapper
```

```
ros2 run era_5g_network_signal_mapper_ros2 sub_signal_mapper
```


## 3) Publish estimated signal after exploring sufficient terrain:

In second RVIZ, subcribe to PointCloud2 topic /estimated_radio_antenna_signal . Style: flat squares. Size (m):0.05

```
python3 find-center-antenna.py
```

```
estimated_signal_map_generator.py
```

**Expected result:**

Center RVIZ: semantic quality map of visited space by robot.

Left RVIZ: estimated reconstructed signal.

Right RVIZ: simulated signal.

![Alt text](<images/Screenshot from 2024-05-17 12-42-32.png>)