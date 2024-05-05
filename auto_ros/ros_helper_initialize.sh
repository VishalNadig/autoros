#! /usr/bin/bash

function ros_helper_initialize() {
    git clone https://github.com/VishalNadig/RoboticsLog.git
    cd RoboticsLog/ros_helpers
    python ros_helper.py
}