#!/bin/bash
ros2 topic pub /marker_topic visualization_msgs/msg/MarkerArray "markers:
- header:
    frame_id: 'base_scan'
  type: 2
  action: 0
  pose:
    position:
      x: 0.5
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  scale:
    x: 0.3
    y: 0.3
    z: 0.3
  color:
    r: 1.0
    g: 0.0
    b: 0.0
    a: 1.0
  lifetime:
    sec: 5
    nanosec: 1
  frame_locked: false" -1
