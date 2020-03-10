#! /bin/bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 0.674, y: 0.119, z: 0.0}, orientation: {w: 1.0}}}'

