# Stimulator G

## Links

* ахуительный local_planner http://wiki.ros.org/teb_local_planner
* http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide#The_Local_Planner
* http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide#The_Local_Planner
* https://github.com/turtlebot/turtlebot_simulator
* https://github.com/turtlebot/turtlebot_simulator/blob/melodic/turtlebot_gazebo/launch/gmapping_demo.launch
* http://library.isr.ist.utl.pt/docs/roswiki/move_base.html
* отслеживание данных и нод в ROS https://answers.ros.org/question/232458/problem-moving-the-robot-using-navigation-stack/
* https://answers.ros.org/question/294784/robot-doesn-t-start-moving/
* https://github.com/gpldecha/turtlebot-navigation/blob/master/turtlebot_nav/launch/includes/velocity_smoother.launch.xml
* http://wiki.ros.org/Remapping%20Arguments
* http://wiki.ros.org/costmap_converter использование costmap_convertor на практике позволяет снизить нагрузку на процессор и с большей выгодой подобрать параметры teb_local_planner
* https://answers.ros.org/question/226236/openni2driver-no-matching-device-found/
* http://learn.turtlebot.com/2015/02/03/11/
* http://learn.turtlebot.com/2015/02/01/11/
* https://github.com/ros/geometry2.git

* https://blog.zhaw.ch/icclab/configuring-the-ros-navigation-stack-on-a-new-robot/
* https://answers.ros.org/question/35844/help-with-navigation-error-transform-from-base_link-to-map/
* https://github.com/STAR-Center/fastGPOM/blob/master/gp_occ_mapping/src/scripts/gpmaps.py
* https://github.com/Skydes/Turtlebot-Navigation-Stack/blob/master/doc/presentation/presentation.pdf
* http://wustl.probablydavid.com/publications/IROS2014.pdf
* https://github.com/goldarte/arucow/blob/master/aruco_destination_pose.py
* https://github.com/amusi/awesome-lane-detection

* https://github.com/Sr4l/virtual_obstacles
* https://wiki.ros.org/occupancy_grid_utils

* https://github.com/rst-tu-dortmund
* http://wiki.ros.org/teb_local_planner
* https://github.com/rst-tu-dortmund/teb_local_planner
* https://github.com/rst-tu-dortmund/teb_local_planner_tutorials
* https://github.com/ros-planning/navigation
* https://github.com/ros-planning/navigation/tree/melodic-devel/map_server
* http://wiki.ros.org/map_server

* http://wiki.ros.org/costmap_2d
* http://wiki.ros.org/costmap_2d/Tutorials
* http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer
* http://wiki.ros.org/costmap_2d/Tutorials/Configuring%20Layered%20Costmaps
* http://wiki.ros.org/costmap_2d/flat
* http://wiki.ros.org/costmap_2d/hydro/staticmap
* http://wiki.ros.org/costmap_2d/hydro/obstacles
* http://wiki.ros.org/costmap_2d/layered

* https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020
* https://github.com/ROBOTIS-GIT/turtlebot3_autorace
* https://github.com/ROBOTIS-GIT/turtlebot3_autorace/blob/master/turtlebot3_autorace_core/nodes/core_node_mission
* https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020/tree/master/turtlebot3_autorace_parking
* https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020/blob/master/turtlebot3_autorace_traffic_light/* turtlebot3_autorace_traffic_light_detect/nodes/detect_traffic_light
* http://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#ros-1-autonomous-driving

* http://wiki.ros.org/costmap_converter
* https://github.com/rst-tu-dortmund/costmap_converter
* http://wiki.ros.org/costmap_2d/hydro/obstacles
* https://stackoverflow.com/questions/44194541/obstacle-inflation-in-costmap-2d
* https://en.wikipedia.org/wiki/DBSCAN
* http://wiki.ros.org/teb_local_planner/Tutorials
* http://wiki.ros.org/teb_local_planner/Tutorials/Setup%20and%20test%20Optimization
* http://wiki.ros.org/teb_local_planner/Tutorials/Costmap%20conversion
* http://wiki.ros.org/costmap_converter
* http://wiki.ros.org/robot_navigation
* http://wiki.ros.org/navigation/Tutorials/RobotSetup
* https://blog.zhaw.ch/icclab/configuring-the-ros-navigation-stack-on-a-new-robot/
* https://github.com/ROBOTIS-GIT/turtlebot3_autorace/blob/master/turtlebot3_autorace_core/nodes/core_node_controller
* https://github.com.cnpmjs.org/stonier/cost_map
* https://roboticsknowledgebase.com/wiki/state-estimation/ros-cost-maps/

## Navigation

Один из скриптов навигации:

```bash
#! /bin/bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.674, y: 0.119, z: 0.0}, orientation: {w: 1.0}}}'
```
