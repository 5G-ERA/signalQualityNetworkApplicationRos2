# signal_quality_network_application_ros2
Code developed  under ROS2 Foxy


Running instructions.
## Run using docker compose


### Download the docke compose files
```
git clone https://github.com/5G-ERA/signal_quality_network_application_ros2.git
cd signal_quality_network_application_ros2
```
### Adjust environment variables in the both files "docker-compose.yaml" from folder create_costmap also


### Run container for signal-mapper and signal-strength-publisher (InfluxDB)
```
docker compose up
```
### After map is created combine costmap with signal point-clouds


```
cd create_costmap
docker compose up
```

## Save map using mapsaver