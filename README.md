# signal_quality_network_application_ros2
Code developed  under ROS2 Foxy


Running instructions.
## Run using docker compose


### Download the docke compose files
```
git clone https://github.com/appmdev/test_git_hub.git
cd test_git_hub
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