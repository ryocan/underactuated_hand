//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "img_proc/fb_control.hpp"

/*************************************************************
 * Function:     subscribe 2D color image
 * Comment:
**************************************************************/
void FbControl::intCb(const std_msgs::Int32::ConstPtr& msg)
{
	control_fin_ = msg->data;
    std::cout << "Received Int32 data: " << control_fin_ << std::endl;
}

/*************************************************************
 * @Function
 *    get parameter from param.yaml
**************************************************************/
bool FbControl::getYamlParam()
{
	// dynamixel servo motor
    nh.getParam("fb_control_launch/DYNAMIXEL_GOAL_VELOCITY_", DYNAMIXEL_GOAL_VELOCITY_);

	// hsv param: hand
    nh.getParam("fb_control_launch/H_MIN_HAND_", H_MIN_HAND_);
    nh.getParam("fb_control_launch/H_MAX_HAND_", H_MAX_HAND_);
    nh.getParam("fb_control_launch/S_MIN_HAND_", S_MIN_HAND_);
    nh.getParam("fb_control_launch/S_MAX_HAND_", S_MAX_HAND_);
    nh.getParam("fb_control_launch/V_MIN_HAND_", V_MIN_HAND_);
    nh.getParam("fb_control_launch/V_MAX_HAND_", V_MAX_HAND_);

	// dobot current position
	nh.getParam("fb_control_launch/DOBOT_X_", DOBOT_X_);
	nh.getParam("fb_control_launch/DOBOT_Y_", DOBOT_Y_);
	nh.getParam("fb_control_launch/DOBOT_Z_", DOBOT_Z_);
	nh.getParam("fb_control_launch/DOBOT_R_", DOBOT_R_);

	// thlesholds
	nh.getParam("fb_control_launch/DEFORMATION_TH_", DEFORMATION_TH_);
	nh.getParam("fb_control_launch/OCCLUSION_TH_", OCCLUSION_TH_);


	return true;
}

/*************************************************************
 * @Function
 *    execute whole detection program
**************************************************************/
bool FbControl::exec()
{
	if(video_count == 0)
	{
		video_writer.open("/home/umelab-pgi5g/Downloads/drop_detection_output.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 280.0, cv::Size(640, 480), true); 
		video_count = 1;
	}

	if(!img_src_color_.empty())
	{
		switch (flag_entire_process_)
        {
			case 0:
				// dobot bringup
				dobotEnableCommandMsg();
				sleep(1);

				dobotPayloadCommandMsg(500, 200);
				dobotSpeedCommandMsg(10); //10
				dobotMoveCommandMsg(DOBOT_X_, DOBOT_Y_, DOBOT_Z_, DOBOT_R_);
				sleep(1);

				// get image for object detection only once
				img_bg_ = img_src_color_.clone();

				flag_entire_process_ = 1;
				break;

			case 1:
				// object detection: object_detection.cpp
				if(objectDetection())
				{
					ROS_INFO("MOTOR ACTIVATION");
					dynamixelCommandMsg("", 1, "Torque_Enable", 1);
					dynamixelCommandMsg("", 2, "Torque_Enable", 1);

					// set goal velocity
					dynamixelCommandMsg("", 1, "Goal_Velocity", DYNAMIXEL_GOAL_VELOCITY_);
					dynamixelCommandMsg("", 2, "Goal_Velocity", DYNAMIXEL_GOAL_VELOCITY_);

					flag_entire_process_ = 2;
				}
				break;
			
			case 2:
				// contact detection: contact_detection.cpp
				if(contactDetection())
				{
					ROS_INFO("Contact Detected!!");
					flag_entire_process_ = 3;
				}
					
				break;
			
			case 3:
				// object state determination
				if(objectStateDetection())
					flag_entire_process_ = 4;
				break;
			
			case 4:
				// stop motor
				dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
            	dynamixelCommandMsg("", 2, "Goal_Velocity", 0);

				// arm trajectory
				arm_traj_point_.clear();
				for(int i = 0; i < 5; i++)
				{
					arm_traj_point_.push_back(cv::Vec4f(DOBOT_X_, DOBOT_Y_, DOBOT_Z_ + 15.0, DOBOT_R_));
					arm_traj_point_.push_back(cv::Vec4f(DOBOT_X_ - 30.0, DOBOT_Y_, DOBOT_Z_ + 15.0, DOBOT_R_));
					arm_traj_point_.push_back(cv::Vec4f(DOBOT_X_ + 30.0, DOBOT_Y_, DOBOT_Z_ + 15.0, DOBOT_R_));
					arm_traj_point_.push_back(cv::Vec4f(DOBOT_X_, DOBOT_Y_, DOBOT_Z_ + 15.0, DOBOT_R_));
					arm_traj_point_.push_back(cv::Vec4f(DOBOT_X_, DOBOT_Y_, DOBOT_Z_ + 30.0, DOBOT_R_));
				}
				arm_traj_point_.push_back(cv::Vec4f(DOBOT_X_, DOBOT_Y_, DOBOT_Z_, DOBOT_R_));
				ROS_INFO("Dobot Position Published");
				flag_entire_process_ = 5;
				break;
			
			case 5:
				// move robot
				publishVec4fData(arm_traj_point_[pos_num_]);
				pos_num_++;
				flag_entire_process_ = 6;
				break;


			case 6:
				// drop detection
				dropDetection();
				break;

			case 7:
				ROS_INFO("MOTOR WILL STOP...");
				dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
            	dynamixelCommandMsg("", 2, "Goal_Velocity", 0);
                dynamixelCommandMsg("", 1, "Torque_Enable", 0);
                dynamixelCommandMsg("", 2, "Torque_Enable", 0);

				ROS_INFO("DOBOT WILL STOP...");
				dobotDisableCommandMsg();
				return false;
				break;
			
			default:
				return false;
				break;
		}


		cv::imshow("img_src_color_", img_src_color_);
		video_writer.write(img_src_color_);
		
		if (cv::waitKey(1) == 'q') 
		{
			ROS_INFO("MOTOR WILL STOP...");
			dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
			dynamixelCommandMsg("", 2, "Goal_Velocity", 0);
			dynamixelCommandMsg("", 1, "Torque_Enable", 0);
			dynamixelCommandMsg("", 2, "Torque_Enable", 0);

			ROS_INFO("MOTOR WILL STOP...");
			dobotDisableCommandMsg();

			return false;
		}

	}
	return true;
}