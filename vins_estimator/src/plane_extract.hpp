#ifndef _PLANE_EXTRACT_H_
#define _PLANE_EXTRACT_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include "Eigen/Core"
using namespace pcl;
class plane_extract
{
private:
    /* data */
public:
    PointCloud<PointXYZ>::Ptr feature_buf;
    const float cube_size; 
    ros::Publisher local_feature_pub;
    
public:
    plane_extract(double Cube_size, ros::NodeHandle& nh)
    : cube_size(Cube_size)
    {
        local_feature_pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_extract/local_feature",10);
    }
    void add_new_feature(PointCloud<PointXYZ>::Ptr new_feature)
    {
        *feature_buf += *new_feature;
    }

    void extract_plane(geometry_msgs::PoseStamped cam_pose)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_feature (new pcl::PointCloud<pcl::PointXYZ>);
        float maxx=cam_pose.pose.position.x + cube_size/2.0;
        float maxy=cam_pose.pose.position.y + cube_size/2.0;
        float maxz=cam_pose.pose.position.z + cube_size/2.0;
        float minx=cam_pose.pose.position.x - cube_size/2.0;
        float miny=cam_pose.pose.position.y - cube_size/2.0;
        float minz=cam_pose.pose.position.z - cube_size/2.0;
    
        pcl::CropBox<pcl::PointXYZ> boxfilter;
        boxfilter.setInputCloud(feature_buf);
        boxfilter.setMin(Eigen::Vector4f(minx, miny, minz, 1.0));
        boxfilter.setMax(Eigen::Vector4f(maxx, maxy, maxz, 1.0));
        
        // boxfilter.setRotation();
        boxfilter.setNegative(true);
        boxfilter.filter(*local_feature);

        sensor_msgs::PointCloud2 vis_msg;
        pcl::toROSMsg(*local_feature,vis_msg);
        vis_msg.header.frame_id="/wrold";
        vis_msg.header.stamp=ros::Time::now();
        local_feature_pub.publish(vis_msg);

    }
    // ~plane_extract();
    

};

// plane_extract::plane_extract()
// {
// }

// plane_extract::~plane_extract()
// {
// }





#endif
