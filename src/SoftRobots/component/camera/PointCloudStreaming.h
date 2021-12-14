///******************************************************************************
//*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
//*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
//*                                                                             *
//* This library is free software; you can redistribute it and/or modify it     *
//* under the terms of the GNU Lesser General Public License as published by    *
//* the Free Software Foundation; either version 2.1 of the License, or (at     *
//* your option) any later version.                                             *
//*                                                                             *
//* This library is distributed in the hope that it will be useful, but WITHOUT *
//* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
//* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
//* for more details.                                                           *
//*                                                                             *
//* You should have received a copy of the GNU Lesser General Public License    *
//* along with this library; if not, write to the Free Software Foundation,     *
//* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
//*******************************************************************************
//*                              SOFA :: Framework                              *
//*                                                                             *
//* Authors: The SOFA Team (see Authors.txt)                                    *
//*                                                                             *
//* Contact information: contact@sofa-framework.org                             *
//******************************************************************************/
#ifndef SOFTROBOTS_CAMERA_POINTCLOUDSTREAMING_H
#define SOFTROBOTS_CAMERA_POINTCLOUDSTREAMING_H

#include <SoftRobots/component/initSoftRobots.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/type/vector.h>
#include <sofa/defaulttype/VecTypes.h>

#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class PointCloudStreaming
{
public:
    PointCloudStreaming(){}
    ~PointCloudStreaming(){}

    void initCamera();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr updatePcl();

private:
    rs2::pipeline pipe;

    rs2::pointcloud rs_pc;
    rs2::points rs_points;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_points;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_filtered_points;

    rs2::decimation_filter dec_filter;
    rs2::disparity_transform disparity_to_depth { false };
    rs2::disparity_transform depth_to_disparity;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

};



#endif //SOFTROBOTS_CAMERA_POINTCLOUDSTREAMING_H
