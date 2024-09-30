//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// for image processing
#include "img_proc/common_proc.hpp"

// about dynamixel
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

// Dobot MG400
#include "mg400_bringup/PayLoad.h"
#include "mg400_bringup/EnableRobot.h"
#include "mg400_bringup/DisableRobot.h"
#include "mg400_bringup/SpeedL.h"
#include "mg400_bringup/MovL.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <thread>
#include <future>
//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class FbControl : public CommonProc
{
	private:
		/*--- ros setup ---*/
		ros::NodeHandle nh;
		ros::ServiceClient dynamixel_client_;

		ros::ServiceClient dobot_payload_client_;
		ros::ServiceClient dobot_enable_client_; 
		ros::ServiceClient dobot_move_client_;
		ros::ServiceClient dobot_disable_client_;
		ros::ServiceClient dobot_speed_client_;

		ros::Publisher pub_dobot_pos_;
		ros::Subscriber sub_dobot_control_fin_;

		/*--- important variables --- */
		int flag_entire_process_ = 0;

		// dynamixel: service communication 
    	bool dynamixelCommandMsg(std::string command, uint8_t id, std::string addr_name, int32_t value);
		double DYNAMIXEL_GOAL_VELOCITY_;

		// dobot mg400: service communication 
		bool dobotPayloadCommandMsg(float weight, float inertia);
		bool dobotEnableCommandMsg();
		bool dobotMoveCommandMsg(float x, float y, float z, float r);
		bool dobotDisableCommandMsg();
		bool dobotSpeedCommandMsg(int r);	// if the speed is too big, robotic arm will be going crazy

		// dobot mg pub
		void publishVec4fData(cv::Vec4f vecs);
		void intCb(const std_msgs::Int32::ConstPtr& msg);
		int control_fin_ = 0;
		double DOBOT_X_ = 0.0;
		double DOBOT_Y_ = 0.0;
		double DOBOT_Z_ = 0.0;
		double DOBOT_R_ = 0.0;

		/*--- detect object by using background subtraction ---*/
		bool objectDetection();
		bool getBgImage();
		void backgroundSubtraction();
		void objectColorExtraction();
		cv::Mat img_bg_, img_fg_, img_obj_th_;
		int H_MIN_OBJ_, S_MIN_OBJ_, V_MIN_OBJ_;
		int H_MAX_OBJ_, S_MAX_OBJ_, V_MAX_OBJ_;

		/*--- contact detection ---*/
		bool contactDetection();
        int H_MIN_HAND_ =  90;
        int H_MAX_HAND_ = 150;
        int S_MIN_HAND_ =  50;
        int S_MAX_HAND_ = 255;
        int V_MIN_HAND_ =  50;
        int V_MAX_HAND_ = 255; 

		/*--- object state detection ---*/
		bool objectStateDetection();
		int count_frame_ = 0;

		// IfG1
		bool handMoveDetection();
		std::vector<cv::Point> centroid_hand_prev_;

		// IfG2
		bool objMoveDetection();
		std::vector<cv::Point2f> centroid_obj_prev_;

		// IfG3
		bool deformationDetection();
		void generateRandomPointsInContours(cv::Mat img_mask);
		cv::Point3f calcPoint3d(cv::Point2f point_input);
		cv::Mat img_obj_gray_prev_;
		std::vector<cv::Point2f> point_optflow_2d_prev_;
		std::vector<cv::Point3f> point_optflow_3d_prev_;
		double deformation_total_3d_ = 0.0;
		int DEFORMATION_TH_;

		// IfG4
		bool handOcclusionDetection();
		double hand_area_init_ = 0.0;
		int OCCLUSION_TH_;

		/*--- drop detection ---*/
		bool dropDetection();
		int frame_current_ = 0;
		int frame_drop_ = 0;
		double distance_centroid_std_ = 0.0;

		// define control point
    	std::vector<cv::Vec4f> arm_traj_point_;
		int count_ = 0;
		int pos_num_ = 0;

		cv::VideoWriter video_writer;
		int video_count = 0;

	public:
		FbControl()
		{
			dynamixel_client_ = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
			
			dobot_payload_client_ = nh.serviceClient<mg400_bringup::PayLoad>("/mg400_bringup/srv/PayLoad");
			dobot_enable_client_ = nh.serviceClient<mg400_bringup::EnableRobot>("/mg400_bringup/srv/EnableRobot");
			dobot_move_client_ = nh.serviceClient<mg400_bringup::MovL>("/mg400_bringup/srv/MovL");
			dobot_speed_client_ = nh.serviceClient<mg400_bringup::SpeedL>("/mg400_bringup/srv/SpeedL");
			dobot_disable_client_ = nh.serviceClient<mg400_bringup::DisableRobot>("/mg400_bringup/srv/DisableRobot");

			pub_dobot_pos_ = nh.advertise<std_msgs::Float32MultiArray>("vec4f_data", 1);
			sub_dobot_control_fin_ = nh.subscribe("int_data", 1, &FbControl::intCb, this);

		}

		~FbControl()
		{
			cv::destroyAllWindows();
		}

		// get parameter from yaml fole
		bool getYamlParam();
	
		// execute whole detection program
    	bool exec();
};