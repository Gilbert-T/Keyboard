//
// Created by zhihui on 2/25/20.
//
#include "keyboard.h"
namespace keyboard{
    KEYBOARD::KEYBOARD(ros::NodeHandle &nh) {
        target_velocity_pub_ = nh.advertise<geometry_msgs::TwistStamped>("twist_raw",1);
        velocity_.max_linear    =  5.0;
        velocity_.min_linear    = -5.0;
        velocity_.max_angular   =  3.0;
        velocity_.min_angular   = -3.0;
        velocity_.linear        =  0.0;
        velocity_.angular       =  0.0;
        velocity_.linear_inc    =  0.1;
        velocity_.angular_inc   =  0.1;
    }
    KEYBOARD::~KEYBOARD() {

    }
    void KEYBOARD::publish_tar_vel() {

        tv_.twist.linear.x   = velocity_.linear;
        tv_.twist.angular.z  = velocity_.angular;
        target_velocity_pub_.publish(tv_);
    }
    int KEYBOARD::proc_input() {
        while(flag == 0){
            if(read(STDIN_FILENO,&ch_,1) < 1)
                break;
            switch(ch_){
                case 'w':
                case 'W':
                    velocity_.linear = max(velocity_.min_linear, min(velocity_.linear + velocity_.linear_inc, velocity_.max_linear));
                    break;
                case 's':
                case 'S':
                    velocity_.linear = max(velocity_.min_linear, min(velocity_.linear - velocity_.linear_inc, velocity_.max_linear));
                    break;
                case 'a':
                case 'A':
                    velocity_.angular = max(velocity_.min_angular, min(velocity_.angular + velocity_.angular_inc, velocity_.max_angular));
                    break;
                case 'd':
                case 'D':
                    velocity_.angular = max(velocity_.min_angular, min(velocity_.angular - velocity_.angular_inc, velocity_.max_angular));
                    break;
                case char(0x20):
                    velocity_.angular = 0;
                    break;
                case 27:
                    flag = 1;
                    break;
                default:
                    velocity_.angular = velocity_.linear = 0;
                    break;
            }
            if(flag == 0){
                printf("\rlin_vel: %+5.3lf,\tang_vel: %+5.3lf", velocity_.linear, velocity_.angular);
                fflush(stdout);
                publish_tar_vel();
            }
            else{
                printf("\nEsc pressed, terminating...\n");
            }

        }
        return 0;
    }
    int KEYBOARD::config_tty() {
        std::string name;
        if(!isatty(STDIN_FILENO)){
            name = "not a tty";
            return -1;
        }
        else{
            name = ttyname(STDIN_FILENO);
            if(name.empty())
                name = "undefined";
        }
        std::cout << "name: " << name << std::endl;

        if(tcgetattr(STDIN_FILENO,&saved_term_) < 0){
            printf("get tty attr failed.\n");
            return -1;
        }

        term_ = saved_term_;
        term_.c_lflag &= ~(ECHO | ICANON);
        term_.c_cc[VMIN] = 1;
        term_.c_cc[VTIME] = 0;
        if(tcsetattr(STDIN_FILENO,TCSANOW,&term_) < 0){
            printf("set tty attr failed.\n");
            return -1;
        }
        if(tcgetattr(STDIN_FILENO,&term_) < 0){
            printf("check tty attr,get attr failed.\n");
            tcsetattr(STDIN_FILENO,TCSAFLUSH,&saved_term_);
        }
        if(term_.c_lflag & (ECHO | ICANON) || term_.c_cc[VMIN] != 1 || term_.c_cc[VTIME] != 0){
            printf("config not effective.\n");
            printf("errno: %d\n",errno);
            tcsetattr(STDIN_FILENO,TCSAFLUSH,&saved_term_);
            return -1;
        }
        return 0;
        
    }
    int KEYBOARD::reset_tty(void) {
        if(tcsetattr(STDIN_FILENO,TCSAFLUSH,&saved_term_) < 0){
            printf("recover tty attr failed.\n");
            return -1;
        }
        return 0;
    }
    void KEYBOARD::print_promptings() {
        printf("max lin_vel: %5.3lf,min lin_vel: %5.3lf,max ang_vel: %5.3lf,min ang_vel: %5.3lf\n",velocity_.max_linear,velocity_.min_linear,
                                                                                                   velocity_.max_angular,velocity_.min_angular);
        printf("w(W) lin_vel += 0.1, s(S) lin_vel -= 0.1\n");
        printf("a(A) ang_vel += 0.1, d(D) ang_vel -= 0.1\n");
        printf("Space ang_vel = 0\n");
        printf("Esc to exit\n");
        printf("any other keys to reset the lin_vel and ang_vel to 0\n");
    }
}
