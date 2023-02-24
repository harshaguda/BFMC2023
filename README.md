# BFMC2023

This is for controlling car using Raspberry Pi. 

If you are using fresh install of RPI image, do the setting up steps from [here](RPISetup.md)

Install ROS using the instructions [here](https://github.com/ECC-BFMC/Brain_ROS)

Usage 

`git clone https://github.com/harshaguda/BFMC2023.git`

`cd BFMC2023/catkin_ws`

`catkin_make`

or

`catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`

`source devel/setup.bash`

`roslaunch utils run_automobile_remote.launch`

Not a best way, but to just make things work! (Better way is to add the below packages in the ROS workspace)
 
```shell
cd src
git clone https://github.com/ros/common_msgs.git
git clone https://github.com/ros/actionlib.git
git clone https://github.com/ros/geometry.git
git clone https://github.com/ros/geometry2.git
sudo apt install liborocos-bfl-dev liborocos-kdl-dev libbullet-dev ros-noetic-angles
```
Make the following changes to the file src/geometry2/tf2/CMakeLists.txt

```diff
diff --git a/tf2/CMakeLists.txt b/tf2/CMakeLists.txt
index a43e14b..7bdbbe9 100644
--- a/tf2/CMakeLists.txt
+++ b/tf2/CMakeLists.txt
@@ -50,8 +50,8 @@ target_link_libraries(speed_test tf2  ${console_bridge_LIBRARIES})
 add_dependencies(tests speed_test)
 add_dependencies(tests ${catkin_EXPORTED_TARGETS})
 
-catkin_add_gtest(test_transform_datatypes test/test_transform_datatypes.cpp)
-target_link_libraries(test_transform_datatypes tf2  ${console_bridge_LIBRARIES})
-add_dependencies(test_transform_datatypes ${catkin_EXPORTED_TARGETS})
+catkin_add_gtest(test_transform2_datatypes test/test_transform_datatypes.cpp)
+target_link_libraries(test_transform2_datatypes tf2  ${console_bridge_LIBRARIES})
+add_dependencies(test_transform2_datatypes ${catkin_EXPORTED_TARGETS})
```
