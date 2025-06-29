## How to build and run

```shell
cd orpheus_project
# This command builds the image and copies all the necessary files
docker build -f ./Docker/FusedData.dockerfile -t fused_data .
# Run image -> container
docker run -it --rm  --name fused_data fused_data /bin/bash 

--------
# Once container is open run :
ros2 launch sensor_fusion_pkg data_fusion.launch.py 
# or below for more verbose output
ros2 run sensor_fusion_pkg data_fusion --ros-args --log-level DEBUG
# echo topic
ros2 topic echo /vertical_velocity 

# There is also a simulation script in the ros2_package that is used to verify the fused_data publisher is working. Run it with:
# run in a seperate terminal -> docker exec -it fused_data bash
ros2 run sensor_fusion_pkg data_fusion_sim 
-----------
# To run tests separately
colcon build --packages-select sensor_fusion_pkg --cmake-clean-cache
colcon test --packages-select sensor_fusion_pkg
colcon test-result --verbose


---------------
# If You want the sensor fusion package to run in your workspace
# Copy the package into your workspace and run a colcon build.

```

### Output sample
![image](?raw=true)
![image](?raw=true)

### Project details
- Project deliverable consists of 1 publishers and two subscribers.
- Sensor fusion approach is using a **complementary filter** to fuse the velocities from the two separate sensors.
- Velocities for each individual sensor are calculated in their respective classes.
- QOS settings are mostly set to default.



