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

## Navigation

Один из скриптов навигации:

```bash
#! /bin/bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.674, y: 0.119, z: 0.0}, orientation: {w: 1.0}}}'
```
