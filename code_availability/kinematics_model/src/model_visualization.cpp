/******************** (C) COPYRIGHT 2024 Arclab ********************************
 * Author      ： Rui Peng
 * File        ： model_visualization.cpp
 * Description ： AET model visualization 
 * Website     ： https://arclab.hku.hk/
 * Github      ： https://github.com/arclab-hku/AET/
**********************************************************************************/

#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include "model.h"

void robot_worldFrame(ros::Publisher robotFrame_pub1, ros::Publisher robotFrame_pub2, ros::Publisher robotFrame_pub3);
Eigen::Vector3f config_to_translationMatrix(float alpha, float beta, float arcLength);
Eigen::Matrix3f config_to_rotationMatrix(float alpha, float beta);
visualization_msgs::MarkerArray backbone_generation(int id, int resolution, float alpha, float beta, float arcLength, Eigen::Matrix3f last_Rot, Eigen::Vector3f last_enddisk_pos);
void disk_visualization(int id, int resolution, float alpha, float beta, float arcLength, Eigen::Matrix3f last_Rot, Eigen::Vector3f last_enddisk_pos) ;
 
int main( int argc, char** argv )
{
    ros::init(argc, argv, "model_visualization");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);

    //----------- visualization publisher
    ros::Publisher robotFrame_pub1   = nh.advertise<visualization_msgs::Marker>( "robotFrame1", 0 );
    ros::Publisher robotFrame_pub2   = nh.advertise<visualization_msgs::Marker>( "robotFrame2", 0 );
    ros::Publisher robotFrame_pub3   = nh.advertise<visualization_msgs::Marker>( "robotFrame3", 0 );

	ros::Publisher arm_section1      = nh.advertise<visualization_msgs::MarkerArray>("arm_section1", 10);
	ros::Publisher arm_section2      = nh.advertise<visualization_msgs::MarkerArray>("arm_section2", 10);
	ros::Publisher arm_section3      = nh.advertise<visualization_msgs::MarkerArray>("arm_section3", 10);
    //----------- visualization publisher

    tf::TransformBroadcaster tf_broadcaster;
    float t = 0;

    struct Pcc_config sec1;
    struct Pcc_config sec2;
    struct Pcc_config sec3;

    /*  arm parameter  */
    float arc_Length = 2.1; // 0.21m
    int resolution = 30;
    /*  arm parameter  */

    // Moving demonstration:
    while (ros::ok())
    {  
        t = t + 0.005;
        sec1.alpha = 30 ;
        sec1.beta  = 180 - t * 90 / (M_PI / 2.0);
        sec2.alpha = 30;
        sec2.beta  = 180 - t * 90 / (M_PI / 2.0);
        sec3.alpha = 20;
        sec3.beta  = 180 - t * 90 / (M_PI / 2.0);
        ROS_INFO( "sec1_alpha %f  sec1_beta %f ", sec1.alpha, sec1.beta);
        ROS_INFO( "sec2_alpha %f  sec2_beta %f ", sec2.alpha, sec2.beta);
        ROS_INFO( "sec3_alpha %f  sec3_beta %f \n", sec3.alpha, sec3.beta);

        /* world frame -> arm base frame */
        Eigen::Vector3f base_position(sin(t)*4, cos(t)*4, 0*3);
        Eigen::Vector3f uav_euler_angle(0, M_PI + 0, 0);  //  yaw pitch row 
        Eigen::Matrix3f rotation_matrix_from_euler;
        rotation_matrix_from_euler = Eigen::AngleAxisf(uav_euler_angle[0], Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(uav_euler_angle[1], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(uav_euler_angle[2], Eigen::Vector3f::UnitX());
        Eigen::Matrix3f Rot_world_base;
        Rot_world_base << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Rot_world_base = Rot_world_base * rotation_matrix_from_euler;
        tf::TransformBroadcaster uav;
        tf::Transform uav_tf;
        uav_tf.setOrigin(tf::Vector3(base_position(0) + 3.6, base_position(1) - 2.2, base_position(2) + 0.1)  );
        uav_tf.setRotation(tf::createQuaternionFromRPY(M_PI / 2 + 0 , 0 + 0, M_PI + 0));
        uav.sendTransform(tf::StampedTransform(uav_tf, ros::Time::now(),"base_link", "uav"));
        /* world frame -> arm base frame */

        /*  world frame -> arm end1 frame  */
        float alpha_1 = sec1.alpha / 180.0 * M_PI, beta_1 = sec1.beta / 180.0 * M_PI;
        Eigen::Matrix3f  Rot_base_end1 = Rot_world_base * config_to_rotationMatrix(alpha_1, beta_1);
        Eigen::Vector3f  Pos_base_end1 = base_position + Rot_world_base * config_to_translationMatrix(alpha_1, beta_1, arc_Length);
        Eigen::Quaternionf q_base_end1(Rot_base_end1);
        tf::Vector3 origin_1 (Pos_base_end1(0,0), Pos_base_end1(1,0), Pos_base_end1(2,0));
        tf::Quaternion rotation_1 (q_base_end1.x(), q_base_end1.y(), q_base_end1.z(), q_base_end1.w());
        tf::Transform t_1 (rotation_1, origin_1);
        tf::StampedTransform end_1_Marker (t_1, ros::Time::now(), "/base_link", "enddisk_1");
        tf_broadcaster.sendTransform(end_1_Marker);
        visualization_msgs::MarkerArray sec1_backbone = backbone_generation(1, resolution, alpha_1, beta_1, arc_Length, Rot_world_base, base_position);
        arm_section1.publish( sec1_backbone );
        disk_visualization(1, resolution, alpha_1, beta_1, arc_Length, Rot_world_base, base_position);
        /*  world frame -> arm end1 frame  */

        /*  world frame -> arm end2 frame  */
        float alpha_2 = sec2.alpha / 180.0 * M_PI, beta_2 = sec2.beta / 180.0 * M_PI;
        Eigen::Matrix3f  Rot_base_end2 = Rot_base_end1  * config_to_rotationMatrix(alpha_2, beta_2);
        Eigen::Vector3f  Pos_base_end2 = Pos_base_end1 + Rot_base_end1 * config_to_translationMatrix(alpha_2, beta_2, arc_Length);
        Eigen::Quaternionf q_base_end2(Rot_base_end2);
        tf::Vector3 origin_2 (Pos_base_end2(0,0), Pos_base_end2(1,0), Pos_base_end2(2,0));
        tf::Quaternion rotation_2 (q_base_end2.x(), q_base_end2.y(), q_base_end2.z(), q_base_end2.w());
        tf::Transform t_2 (rotation_2, origin_2);
        tf::StampedTransform end_2_Marker (t_2, ros::Time::now(), "/base_link", "enddisk_2");
        tf_broadcaster.sendTransform(end_2_Marker);
        visualization_msgs::MarkerArray sec2_backbone = backbone_generation(2, resolution, alpha_2, beta_2, arc_Length, Rot_base_end1, Pos_base_end1);
        arm_section2.publish( sec2_backbone );
        disk_visualization(2, resolution, alpha_2, beta_2, arc_Length, Rot_base_end1, Pos_base_end1);
        /*  world frame -> arm end2 frame  */

        /*  world frame -> arm end3 frame  */
        float alpha_3 = sec3.alpha / 180.0 * M_PI, beta_3 = sec3.beta / 180.0 * M_PI;
        Eigen::Matrix3f  Rot_base_end3 = Rot_base_end2  * config_to_rotationMatrix(alpha_3, beta_3);
        Eigen::Vector3f  Pos_base_end3 = Pos_base_end2 + Rot_base_end2 * config_to_translationMatrix(alpha_3, beta_3, arc_Length);
        Eigen::Quaternionf q_base_end3(Rot_base_end3);
        tf::Vector3 origin_3 (Pos_base_end3(0,0), Pos_base_end3(1,0), Pos_base_end3(2,0));
        tf::Quaternion rotation_3 (q_base_end3.x(), q_base_end3.y(), q_base_end3.z(), q_base_end3.w());
        tf::Transform t_3 (rotation_3, origin_3);
        tf::StampedTransform end_3_Marker (t_3, ros::Time::now(), "/base_link", "enddisk_3");
        tf_broadcaster.sendTransform(end_3_Marker);
        visualization_msgs::MarkerArray sec3_backbone = backbone_generation(3, resolution, alpha_3, beta_3, arc_Length, Rot_base_end2, Pos_base_end2);
        arm_section3.publish( sec3_backbone );
        disk_visualization(3, resolution, alpha_3, beta_3, arc_Length, Rot_base_end2, Pos_base_end2);
        /*  world frame -> arm end3 frame  */
 
        ////////////////////////////////////////////////
        ros::spinOnce();
        rate.sleep();
    }
}

Eigen::Vector3f config_to_translationMatrix(float alpha, float beta, float arcLength)
{
    // alpha can't be exactly 0;
    if(alpha == 0)
        alpha = 0.000001;

    float x_temp = arcLength/alpha * (1-cos(alpha)) * sin(beta);
    float y_temp = arcLength/alpha * (1-cos(alpha)) * cos(beta);
    float z_temp = arcLength/alpha * sin(alpha);
    Eigen::Vector3f pos_in_this_enddisk_frame(x_temp, y_temp, z_temp);

    return pos_in_this_enddisk_frame;
}



// rodrigeze formulation:
Eigen::Matrix3f config_to_rotationMatrix(float alpha, float beta)
{
    // alpha can't be exactly 0;
    if(alpha == 0)
        alpha = 0.000001;

    Eigen::Matrix3f Rot;
    Rot << cos(beta)*cos(beta)*(1-cos(alpha)) + cos(alpha), -cos(beta)*sin(beta)*(1-cos(alpha)), 			 sin(alpha)*sin(beta),
           -cos(beta)*sin(beta)*(1-cos(alpha))   		   ,  sin(beta)*sin(beta)*(1-cos(alpha)) + cos(alpha), sin(alpha)*cos(beta),
           -sin(alpha)*sin(beta)						   , -sin(alpha)*cos(beta),  						 cos(alpha)         ;
    return Rot;
}


visualization_msgs::MarkerArray backbone_generation(int id, int resolution, float alpha, float beta, float arcLength, Eigen::Matrix3f last_Rot, Eigen::Vector3f last_enddisk_pos) 
{
    // alpha can't be exactly 0;
    if(alpha == 0)
        alpha = 0.000001;

    visualization_msgs::MarkerArray section_backbone;

    for(int i = 0; i < resolution; i++)
    {
        float sample_length = i * arcLength / (resolution - 1);
        float sample_alpha  = i * alpha     / (resolution - 1);
        float sample_beta    = beta ;
        
        float x_temp = sample_length / sample_alpha * (1-cos(sample_alpha)) * sin(sample_beta);
        float y_temp = sample_length / sample_alpha * (1-cos(sample_alpha)) * cos(sample_beta);
        float z_temp = sample_length / sample_alpha * sin(sample_alpha);

        Eigen::Vector3f pos_sample_local_frame(x_temp, y_temp, z_temp);
        Eigen::Vector3f pos_sample_base_frame;
        pos_sample_base_frame = last_enddisk_pos + last_Rot * pos_sample_local_frame;

        ////////////////////

        visualization_msgs::Marker sample_point;
	    sample_point.header.frame_id = "base_link";
	    sample_point.header.stamp = ros::Time::now();
	    sample_point.ns = "basic_shapes";
	    sample_point.id = i;
	    sample_point.type   = visualization_msgs::Marker::SPHERE;
	    sample_point.action = visualization_msgs::Marker::ADD;
	    sample_point.scale.x = .05;
	    sample_point.scale.y = .05;
	    sample_point.scale.z = .05;
	    sample_point.color.a = 1.0;

        if(id == 1)
        {sample_point.color.r = 0.0; sample_point.color.b = 0.0; sample_point.color.g = 1.0;}
    
        if(id == 2)
        {sample_point.color.r = 0.0; sample_point.color.b = 1.0; sample_point.color.g = 0.0;}

        if(id == 3)
        {sample_point.color.r = 1.0; sample_point.color.b = 0.0; sample_point.color.g = 0.0;}

	    sample_point.lifetime = ros::Duration();

        sample_point.pose.position.x = pos_sample_base_frame.x();
        sample_point.pose.position.y = pos_sample_base_frame.y();
        sample_point.pose.position.z = pos_sample_base_frame.z();
        sample_point.pose.orientation.x = 0;  
        sample_point.pose.orientation.y = 0; 
        sample_point.pose.orientation.z = 0;   
        sample_point.pose.orientation.w = 1; 

        section_backbone.markers.push_back(sample_point);
    }

    return section_backbone;
}


void disk_visualization(int id, int resolution, float alpha, float beta, float arcLength, Eigen::Matrix3f last_Rot, Eigen::Vector3f last_enddisk_pos) 
{
    float disk_n = 7 ;
    // alpha can't be exactly 0;
    if(alpha == 0)
        alpha = 0.000001;

    for(int i = 1; i <= disk_n; i++)
    {
        float sample_length = i * arcLength / disk_n;
        float sample_alpha  = i * alpha     / disk_n;
        float sample_beta    = beta ;
        
        float x_temp = sample_length / sample_alpha * (1-cos(sample_alpha)) * sin(sample_beta);
        float y_temp = sample_length / sample_alpha * (1-cos(sample_alpha)) * cos(sample_beta);
        float z_temp = sample_length / sample_alpha * sin(sample_alpha);

        Eigen::Vector3f pos_sample_local_frame(x_temp, y_temp, z_temp);
        Eigen::Vector3f pos_sample_base_frame;
        pos_sample_base_frame = last_enddisk_pos + last_Rot * pos_sample_local_frame;

        Eigen::Matrix3f  Rot =  last_Rot * config_to_rotationMatrix(sample_alpha, sample_beta);
        Eigen::Quaternionf q(Rot);

        tf::Vector3 origin (pos_sample_base_frame.x(), pos_sample_base_frame.y(), pos_sample_base_frame.z());
        tf::Quaternion rotation (q.x(), q.y(), q.z(), q.w());
        tf::Transform t (rotation, origin);

        if(id == 1)
        {
            char disk_FrameName[30];
            tf::TransformBroadcaster tf_broadcaster;

            tf::Vector3 origin_base (last_enddisk_pos.x(), last_enddisk_pos.y(), last_enddisk_pos.z());
            Eigen::Quaternionf q_base(last_Rot);
            tf::Quaternion rotation_base (q_base.x(), q_base.y(), q_base.z(), q_base.w());
            tf::Transform t_base (rotation_base, origin_base);

            sprintf(disk_FrameName, "S%dL%d", 0, 0);
            tf::StampedTransform base_Marker (t_base, ros::Time::now(), "/base_link", disk_FrameName);
            tf_broadcaster.sendTransform(base_Marker);

            sprintf(disk_FrameName, "S%dL%d", id-1, i);
            tf::StampedTransform disk_Marker (t, ros::Time::now(), "/base_link", disk_FrameName);
            tf_broadcaster.sendTransform(disk_Marker);
        }    
        if(id == 2)
        {
            char disk_FrameName[30];
            tf::TransformBroadcaster tf_broadcaster;

            sprintf(disk_FrameName, "S%dL%d", id-1, i);
            tf::StampedTransform disk_Marker (t, ros::Time::now(), "/base_link", disk_FrameName);
            tf_broadcaster.sendTransform(disk_Marker);
        }

        if(id == 3)
        {
            char disk_FrameName[30];
            tf::TransformBroadcaster tf_broadcaster;

            sprintf(disk_FrameName, "S%dL%d", id-1, i);
            tf::StampedTransform disk_Marker (t, ros::Time::now(), "/base_link", disk_FrameName);
            tf_broadcaster.sendTransform(disk_Marker);
        }
    }

}
 
