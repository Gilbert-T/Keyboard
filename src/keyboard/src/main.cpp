#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"

int main(int argc, char ** argv) {
    ros::init(argc,argv,"keyboard");
    ros::NodeHandle nh("~");
    keyboard::KEYBOARD keyboard_ctrl(nh);
    keyboard_ctrl.print_promptings();
    keyboard_ctrl.config_tty();
    keyboard_ctrl.proc_input();
    keyboard_ctrl.reset_tty();

    return 0;
}