//
// Created by zhihui on 2/25/20.
//

#ifndef KEYBOARD_KEYBOARD_H
#define KEYBOARD_KEYBOARD_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <termios.h>
#define min(a,b) (a > b ? b : a)
#define max(a,b) (a > b ? a : b)

typedef struct velocity{
    double max_linear;
    double min_linear;
    double max_angular;
    double min_angular;
    double linear;
    double angular;
    double linear_inc;
    double angular_inc;

}velocity_t;
namespace keyboard{
    class KEYBOARD{
    public:
        KEYBOARD(ros::NodeHandle& nh);
        ~KEYBOARD();


        int proc_input(void);
        int config_tty(void);
        int reset_tty(void);
        void print_promptings(void);
    private:
        void publish_tar_vel(void);

        bool flag = false;
        velocity_t velocity_;
        ros::Publisher target_velocity_pub_;
        termios term_,saved_term_;
        char ch_;
        geometry_msgs::TwistStamped tv_;




    };
}

#endif //KEYBOARD_KEYBOARD_H
