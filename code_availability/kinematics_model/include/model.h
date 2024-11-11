#ifndef model_visualization_H
#define model_visualization_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "std_msgs/String.h"
 

using namespace std;

//////////////////////////////////////////////////
struct task_space
{
  // translation
  float x = 0;
  float y = 0;
  float z = 0;

  // rotation
  float m11 = 0; float m12 = 0; float m13 = 0;
  float m21 = 0; float m22 = 0; float m23 = 0;
  float m31 = 0; float m32 = 0; float m33 = 0;
};
 
struct Pcc_config
{
  float alpha = 0;
  float beta  = 0;
};

struct arm_configuration
{
  float sec1_alpha = 0;
  float sec1_beta = 0;
  float sec2_alpha = 0;
  float sec2_beta = 0;
  float sec3_alpha = 0;
  float sec3_beta = 0;
};

struct IMU_eulerAngle
{
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
};
extern struct IMU_eulerAngle IMU;
extern struct IMU_eulerAngle IMU1;
extern struct IMU_eulerAngle IMU2;
extern struct IMU_eulerAngle IMU3;

struct actuaction_Torque
{
  int Torque_1 = 0;
  int Torque_2 = 0;
  int Torque_3 = 0;
  int Torque_4 = 0;
};

extern struct actuaction_Torque act_Torque;

struct actuaction_Encoder
{
  int Encoder_1 = 0;
  int Encoder_2 = 0;
  int Encoder_3 = 0;
  int Encoder_4 = 0;
};

extern struct actuaction_Encoder act_Encoder;

struct actuaction_Length
{
  float Length_1 = 0;
  float Length_2 = 0;
  float Length_3 = 0;
  float Length_4 = 0;
  float Length_5 = 0;
  float Length_6 = 0;
};

extern struct actuaction_Length act_Length;

struct to_MCU
{
  int section_1_theta  = 0;
  int section_1_phi = 0;
  int section_2_theta = 0;
  int section_2_phi = 0;
  int section_3_theta = 0;
  int section_3_phi = 0;
};

extern struct to_MCU CMD_to_MCU;

struct imu_accel
{
  int x = 0;
  int y = 0;
  int z = 0;
};

extern struct imu_accel accel;

struct imu_gyro 
{
  int x = 0;
  int y = 0;
  int z = 0;
};

extern struct imu_gyro gyro;

struct imu_mag 
{
  int x = 0;
  int y = 0;
  int z = 0;
};

extern struct imu_mag mag;

struct Arm_cons
{
  float seg_n   = 0;
  float r       = 0;
  float arc_len = 0;
  float seg_len = 0;
};

extern struct Arm_cons arm_cons;

 

struct config_space
{
  float sec1_alpha = 0;
  float sec1_beta  = 0;
  float sec2_alpha = 0;
  float sec2_beta  = 0;
  float sec3_alpha = 0;
  float sec3_beta  = 0;
};
extern struct config_space IMUs_config;
extern struct config_space LENs_config;

extern struct Pcc_config pcc_config;
extern struct Pcc_config section_1;
extern struct Pcc_config section_2;
extern struct Pcc_config section_3;

struct EE_POS
{
  float EE_X = 0;
  float EE_Y = 0;
  float EE_Z = 0;
  float EE_roll = 0;
  float EE_pitch = 0;
};

extern struct EE_POS EE_pos;
extern struct EE_POS Gripper_pos;

extern struct RobotMsgFromMCU msg_frommcu;
extern struct RobotMsgToMCU_write Ctr_command;

struct Secx_normal
{
  float ex = 0;
  float ey = 0;
  float ez = 0;
};

extern struct Secx_normal sec1_normal;
extern struct Secx_normal sec2_normal;
extern struct Secx_normal sec3_normal;

struct euler_angle_space
{
  double sec1_theta = 0;
  double sec1_phi   = 0;
  double sec1_psi   = 0;

  double sec2_theta = 0;
  double sec2_phi   = 0;
  double sec2_psi   = 0;

  double sec3_theta = 0;
  double sec3_phi   = 0;
  double sec3_psi   = 0;
};

extern struct euler_angle_space des_euler_angle_space;

struct optimal_error_space
{
  bool  error_init_flag = true;
  float sec1_phi_init_error = 0;
  float sec1_theta_init_error = 0;
  float sec1_phi_realtime_error = 0;
  float sec1_theta_realtime_error = 0;
  float sec1_phi_err_percentage = 0;
  float sec1_theta_err_percentage = 0;

  float sec2_phi_init_error = 0;
  float sec2_theta_init_error = 0;
  float sec2_phi_realtime_error = 0;
  float sec2_theta_realtime_error = 0;
  float sec2_phi_err_percentage = 0;
  float sec2_theta_err_percentage = 0;

  float sec3_phi_init_error = 0;
  float sec3_theta_init_error = 0;
  float sec3_phi_realtime_error = 0;
  float sec3_theta_realtime_error = 0;
  float sec3_phi_err_percentage = 0;
  float sec3_theta_err_percentage = 0;

};



////////////////////////////////////////////////

extern geometry_msgs::PoseStamped object1_pose;
extern geometry_msgs::PoseStamped object2_pose;
extern geometry_msgs::PoseStamped object_grab_pose;

extern geometry_msgs::PoseStamped mcu_control_cmd;

extern geometry_msgs::PoseStamped EE_Cur_pose;
extern geometry_msgs::PoseStamped MCU_info;

extern geometry_msgs::PoseStamped sec1_eePos_info;
extern geometry_msgs::PoseStamped sec2_eePos_info;
extern geometry_msgs::PoseStamped sec3_eePos_info;

extern geometry_msgs::PoseStamped vic_basePos_info;
extern geometry_msgs::PoseStamped vic_sec1Pos_info;
extern geometry_msgs::PoseStamped vic_sec2Pos_info;
extern geometry_msgs::PoseStamped vic_sec3Pos_info;
extern geometry_msgs::PoseStamped vic_gtPos;

// ////////////////////////////////


extern bool rec_flag ;

extern int  manipulator_initial_flag ;


extern float object_grab_roll   ;
extern float object_grab_pitch  ;

extern float EE_vicon_roll   ;
extern float EE_vicon_pitch  ;


extern struct Pcc_config sec1;
extern struct Pcc_config sec2;







#endif
