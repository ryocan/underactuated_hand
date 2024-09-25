#include "img_proc/fb_control.hpp"

int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "deformable_obj_grasp");
    
    // define imgFbController Class as ifc
    FbControl FC;

    // 


    // get parameter from yaml file
    if (!FC.getYamlParam())
    {
        ROS_WARN("Failed to get parameter from yaml file");
        return false;
    }

    // run the whole program: exec()
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if(!FC.exec())
        {
            ROS_WARN("Program stopped");
            break;
        }

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}