/*
 * viz.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: xsunaf
 */
#include "viz.h"
 void viz(const cv::Mat points4D,ros::Publisher& pub_pts,ros::Publisher& pub_cam,cv::Mat t[],Eigen::Quaternionf q0,Eigen::Quaternionf q[]){

	 	visualization_msgs::Marker pts_array, cam_1, cam_2;

        pts_array.header.frame_id = "/world_frame";
        cam_1.header.frame_id = "/world_frame";
        cam_2.header.frame_id = "/world_frame";

        pts_array.header.stamp = ros::Time::now();
        cam_1.header.stamp = ros::Time::now();
        cam_2.header.stamp = ros::Time::now();

        pts_array.ns = "points_and_cameras";
        cam_1.ns = "points_and_cameras";
        cam_2.ns = "points_and_cameras";
        pts_array.action = visualization_msgs::Marker::ADD;
        cam_1.action = visualization_msgs::Marker::ADD;
        cam_2.action = visualization_msgs::Marker::ADD;

        pts_array.id = 0;
        cam_1.id = 1;
        cam_2.id = 2;

        pts_array.type = visualization_msgs::Marker::POINTS;
        cam_1.type = visualization_msgs::Marker::ARROW;
        cam_2.type = visualization_msgs::Marker::ARROW;

        pts_array.scale.x = 0.01;
        pts_array.scale.y = 0.01;
        pts_array.scale.z = 0.01;

        cam_1.scale.x = 0.1;
        cam_1.scale.y = 0.1;
        cam_1.scale.z = 0.1;
        cam_2.scale.x = 0.1;
        cam_2.scale.y = 0.1;
        cam_2.scale.z = 0.1;

// TODO: Here all the points colors are in red. If possible change them according to their pixel value in the photos.
//
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        pts_array.color.a = 1.0;
        pts_array.color.r = 1.0;
        pts_array.color.g = 0.0;
        pts_array.color.b = 0.0;

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        cam_1.color.a = 1.0;
        cam_1.color.r = 0.0;
        cam_1.color.g = 0.0;
        cam_1.color.b = 1.0;
        cam_2.color.a = 1.0;
        cam_2.color.r = 0.0;
        cam_2.color.g = 0.0;
        cam_2.color.b = 1.0;

        /* cameras are initialized with Markers::pose */

        cam_1.pose.position.x = t0.at<float>(0);
        cam_1.pose.position.y = t0.at<float>(1);
        cam_1.pose.position.z = t0.at<float>(2);
        cam_2.pose.position.x = t[0].at<float>(0);
        cam_2.pose.position.y = t[0].at<float>(1);
        cam_2.pose.position.z = t[0].at<float>(2);

        cam_1.pose.orientation.x = q0.x();
        cam_1.pose.orientation.y = q0.y();
        cam_1.pose.orientation.z = q0.z();
        cam_1.pose.orientation.w = q0.w();
        cam_2.pose.orientation.x = q[0].x();
        cam_2.pose.orientation.y = q[0].y();
        cam_2.pose.orientation.z = q[0].z();
        cam_2.pose.orientation.w = q[0].w();

        /* points are initialized with geometry_msgs::Point */

        for (int i=0;i<n_matches[0];i++)
        {
            geometry_msgs::Point p;
            p.x = points4D.at<float>(0,i);
            p.y = points4D.at<float>(1,i);
            p.z = points4D.at<float>(2,i);

            pts_array.points.push_back(p);
        }

        pub_pts.publish(pts_array);
        pub_cam.publish(cam_1);
        pub_cam.publish(cam_2);

    }


