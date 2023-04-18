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
#include <SoftRobots/component/camera/PointCloudStreaming.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <vector>
#include <iostream>

namespace softrobots::camera {

using namespace std;

void PointCloudStreaming::initCamera(){
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::config cameraConfig;
    cameraConfig.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16);
    cameraConfig.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGB8);

    // Start streaming with default recommended configuration
    pipe.start();

    pcl_points.reset( new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_filtered_points.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
    // but you can also increase the following parameter to decimate depth more (reducing quality)
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

    // Define transformations from and to Disparity domain
    disparity_to_depth = false;

    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudStreaming::updatePcl()
{
    rs2::frameset frames = pipe.wait_for_frames();// Wait for the next set of frames from the camera

    auto colorFrame = frames.get_color_frame();
    rs_pc.map_to(colorFrame);//map the colored depth to the point cloud
    int width = colorFrame.get_width();
    int height = colorFrame.get_height();
    const unsigned char* colorData = static_cast<const unsigned char*>(colorFrame.get_data());
    auto color_intrinsic= colorFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    rs2::frame depthFrame = frames.get_depth_frame();
    // Decimation will reduce the resultion of the depth image,
    // closing small holes and speeding-up the algorithm
    depthFrame = dec_filter.process(depthFrame);
    // To make sure far-away objects are filtered proportionally
    // we try to switch to disparity domain
    depthFrame = depth_to_disparity.process(depthFrame);
    // Apply spatial filtering
    depthFrame = spat_filter.process(depthFrame);
    // Apply temporal filtering
    depthFrame = temp_filter.process(depthFrame);
    // If we are in disparity domain, switch back to depth
    depthFrame = disparity_to_depth.process(depthFrame);

    // Generate the pointcloud and texture mappings
    rs_points = rs_pc.calculate(depthFrame);//generate the pointcloud from the depth data

    auto rs_vertices = rs_points.get_vertices();
    auto textureCoordinates = rs_points.get_texture_coordinates();
    pcl_points->points.resize(rs_points.size());
    for (unsigned int i = 0; i < rs_points.size(); i++)
    {
        int x = static_cast<int>(textureCoordinates[i].u * width);
        int y = static_cast<int>(textureCoordinates[i].v * height);
        int colorLocation =(y * color_intrinsic.width + x) * 3;

        pcl_points->points[i].x = rs_vertices[i].x;
        pcl_points->points[i].y = rs_vertices[i].y;
        pcl_points->points[i].z = rs_vertices[i].z;
        pcl_points->points[i].r = colorData[colorLocation];
        pcl_points->points[i].g = colorData[colorLocation + 1];
        pcl_points->points[i].b = colorData[colorLocation + 2];
    }


    //////////////////////////////////////////////////////////////////Point cloud filter/////////////////////////////////////////////////
    //filter the original point cloud
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    // X-filtering
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.13, 0.13);
    pass.setInputCloud(pcl_points);
    //        pass.setKeepOrganized(true);
    pass.filter(*pcl_filtered_points);
    // Y-filtering
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.13, 0.13);
    pass.setInputCloud(pcl_filtered_points);
    //        pass.setKeepOrganized(true);
    pass.filter(*pcl_filtered_points);
    // Z-filtering
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.54);
    pass.setInputCloud(pcl_filtered_points);
    //        pass.setKeepOrganized(true);
    pass.filter(*pcl_filtered_points);

    //Downsampling using voxel grid
    double voxel_size_=0.0011;//the larger, the less points
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(pcl_filtered_points);
    grid.filter(*pcl_filtered_points);

    //radius outlier removal (simple and faster than statistical outlier removal)
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor_radius;
    sor_radius.setInputCloud(pcl_filtered_points);
    sor_radius.setRadiusSearch(0.005);
    sor_radius.setMinNeighborsInRadius(20);
    sor_radius.filter(*pcl_filtered_points);
    //////////////////////////////////////////////////////////////////Point cloud filter end/////////////////////////////////////////////////

    return pcl_filtered_points;
}

} // namespace
