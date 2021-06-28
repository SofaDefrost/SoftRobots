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
#ifndef SOFA_CORE_CAMERA_POINTCLOUDPROCESSING_H
#define SOFA_CORE_CAMERA_POINTCLOUDPROCESSING_H

#include "PointCloudStreaming.h"
#include <SoftRobots/component/initSoftRobots.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/vector.h>
#include<sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Mat.h>
#include <Eigen/Dense>


#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

namespace sofa
{

namespace component
{

namespace pointcloudprocessing
{

using sofa::core::objectmodel::BaseObject;
using sofa::core::objectmodel::Data;
using sofa::defaulttype::Vec3dTypes;

class PointCloudProcessing : public  BaseObject
{
    public:
        typedef Vec3dTypes::VecCoord VecCoord;

        SOFA_CLASS(PointCloudProcessing, BaseObject);

    public:
        PointCloudProcessing();
        ~PointCloudProcessing() override {}

        void handleEvent(core::objectmodel::Event *event) override;
        void init() override;
        void update();

        Data<VecCoord> d_effectorPositions;
        Data<VecCoord> d_goalPositions;
        Data<VecCoord> d_normalDirections;
        Data<helper::vector<int>> d_contactLocations;

        Data<defaulttype::Mat3x4d> d_M;
        Eigen::Matrix4d m_transform;

        /// Region growing segmentation tuning
        Data<float> d_distanceThreshold;
        Data<float> d_pointColorThreshold;
        Data<float> d_regionColorThreshold;
        Data<int> d_minClusterSize;

        /// K-NearestNeighbors
        Data<double> d_radiusSearch;
        Data<int> d_minNeighborsInRadius;

        /// EuclidianClusterExtraction
        Data<double> d_euclidianClusterTolerance;
        Data<int> d_minEuclidianClusterSize;
        Data<int> d_maxEuclidianClusterSize;

        Data<unsigned> d_effectorNumber;

    private:
        PointCloudStreaming m_pclcamera;
        void imageSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filteredPoints,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& contact_cluster,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_cluster);
        void computeEffectorsDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_effectors,
                                       pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);
        void computeDesiredRobotPosition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_cluster,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_effectors,
                                         pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_desiredeffectors);
        void locateContacts(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& contact_cluster,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_cluster,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_effectors,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& robot_ContactLocation,
                            Eigen::VectorXd& indexContactLocation);
};


} // namespace pointcloudprocessing

} // namespace component

} // namespace sofa

#endif //SOFA_CORE_CAMERA_POINTCLOUDPROCESSING_H
