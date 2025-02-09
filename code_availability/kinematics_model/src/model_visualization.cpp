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

  
        ////////////////////////////////////////////////
        ros::spinOnce();
        rate.sleep();
    }
}
 