//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "img_proc/fb_control.hpp"

/*************************************************************
 * @Function
 *    dynamixel control using service communication 
**************************************************************/
bool FbControl::dynamixelCommandMsg(std::string command, uint8_t id, std::string addr_name, int32_t value)
{
    dynamixel_workbench_msgs::DynamixelCommand commandMsg;

    commandMsg.request.command = command;
    commandMsg.request.id = id;
    commandMsg.request.addr_name = addr_name;
    commandMsg.request.value = value;

	// request commmand
    dynamixel_client_.call(commandMsg);
    return true;
}

/*************************************************************
 * @Function
 *    dobot payload
**************************************************************/
bool FbControl::dobotPayloadCommandMsg(float weight, float inertia)
{
    mg400_bringup::PayLoad commandMsg;

    commandMsg.request.weight = weight;
	commandMsg.request.inertia = inertia;

	// request commmand
    dobot_payload_client_.call(commandMsg);
    return true;
}

/*************************************************************
 * @Function
 *    dobot enable
**************************************************************/
bool FbControl::dobotEnableCommandMsg()
{
    mg400_bringup::EnableRobot set_enable;
    int response;
  
    response = dobot_enable_client_.call(set_enable);
    ROS_INFO("mg_enable:response -> %d", response);

    return true;
}

/*************************************************************
 * @Function
 *    dobot control using service communication 
**************************************************************/
bool FbControl::dobotMoveCommandMsg(float x, float y, float z, float r)
{
    mg400_bringup::MovL commandMsg;

    commandMsg.request.x = x;
	commandMsg.request.y = y;
	commandMsg.request.z = z;
	commandMsg.request.r = r;

	// request commmand
    dobot_move_client_.call(commandMsg);
    return true;
}

/*************************************************************
 * @Function
 *    speed control
**************************************************************/
bool FbControl::dobotSpeedCommandMsg(int r)
{
    mg400_bringup::SpeedL commandMsg;

	commandMsg.request.r = r;

	// request commmand
    dobot_speed_client_.call(commandMsg);
    return true;
}

/*************************************************************
 * @Function
 *    dobot disable
**************************************************************/
bool FbControl::dobotDisableCommandMsg()
{
    mg400_bringup::DisableRobot set_disable;
  
    dobot_disable_client_.call(set_disable);
    return true;
}

/*************************************************************
 * @Function
 *    dobot pub
**************************************************************/
void FbControl::publishVec4fData(cv::Vec4f vec) 
{
    std_msgs::Float32MultiArray msg;

    // std::vector<cv::Vec4f>をFloat32MultiArrayに変換
    msg.data.push_back(vec[0]);  // x
    msg.data.push_back(vec[1]);  // y
    msg.data.push_back(vec[2]);  // z
    msg.data.push_back(vec[3]);  // w

    pub_dobot_pos_.publish(msg);
}