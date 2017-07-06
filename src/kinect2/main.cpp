#include "Protonect.h"

int main(int argc, char *argv[])
{
    ros::Time::init();
    ros::init(argc, argv, "kinect2_driver");

    Protonect pro;
    pro.cicle();

    return 0;
}
