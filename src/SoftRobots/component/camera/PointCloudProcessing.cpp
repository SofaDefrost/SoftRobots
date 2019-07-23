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
#include "PointCloudProcessing.h"


#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/helper/accessor.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/grabcut_segmentation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <pcl/point_types.h>
#include <pcl/surface/mls.h>

#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <Eigen/Dense>

using sofa::helper::ReadAccessor;
using sofa::helper::WriteOnlyAccessor;

namespace sofa
{

namespace component
{

namespace pointcloudprocessing
{

using namespace Eigen;
using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> createPointCloudViewer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_filtered);

PointCloudProcessing::PointCloudProcessing() :
    d_effectorPositions(initData(&d_effectorPositions, "effectorPositions", "Position of effectors.")),
    d_goalPositions(initData(&d_goalPositions, "goalPositions", "Position of the contact forces.")),
    d_normalDirections(initData(&d_normalDirections, "normalDirection", "The normal directions of effectors")),
    d_contactLocations(initData(&d_contactLocations, "contactLocations", "The index of contact locations"))
{
    pclcamera.initCamera();
}


void PointCloudProcessing::update()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredpoints = pclcamera.updatePcl();

    //user-defined
    //The following vector and matrix are the calibration matrix between the camera coordinate system and the robot coordinate system.
    double x0 = -14.18895169; //unit is millimeter
    double y0 = 35.32077316; //unit is millimeter
    double z0 = 578.52358217; //unit is millimeter
    //build rotation matrix
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    transform_1 (0,0) = -0.69419594;
    transform_1 (0,1) = -0.71476897;
    transform_1 (0,2) = 0.08483703;
    transform_1 (1,0) = -0.7196964;
    transform_1 (1,1) = 0.68741082;
    transform_1 (1,2) = -0.09748567;
    transform_1 (2,0) = 0.01136184;
    transform_1 (2,1) = -0.12873106;
    transform_1 (2,2) = -0.99161445;
    // Define a translation.
    transform_1 (0,3) = x0/1000;
    transform_1 (1,3) = y0/1000;
    transform_1 (2,3) = z0/1000;


    //////////////////////////////////////////////////////////////////Image segmentation/////////////////////////////////////////////////
    //Color-based region growing segmentation
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    //This line is responsible for pcl::RegionGrowingRGB instantiation. This class has two parameters:
    //PointT - type of points to use(in the given example it is pcl::PointXYZRGB)
    //NormalT - type of normals to use. Insofar as pcl::RegionGrowingRGB is derived from the pcl::RegionGrowing, it can use both tests at the same time: color test and normal test. The given example uses only the first one, therefore type of normals is not used.
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    //These lines provide the instance with the input cloud, and search method.
    reg.setInputCloud (filteredpoints);
    reg.setSearchMethod (tree);
    // It is used to determine whether the point is neighbouring or not.
    // If the point is located at a distance less than the given threshold, then it is considered to be neighbouring.
    // It is used for clusters neighbours search.
    reg.setDistanceThreshold (0.01);
    //This line sets the color threshold.
    //Just as angle threshold is used for testing points normals in pcl::RegionGrowing to determine if the point belongs to cluster, this value is used for testing points colors.
    reg.setPointColorThreshold (6);
    //Here the color threshold for clusters is set. This value is similar to the previous, but is used when the merging process takes place.
    reg.setRegionColorThreshold (4);
    //it is used for merging process mentioned in the beginning. If cluster has less points than was set through setMinClusterSize method, then it will be merged with the nearest neighbour.
    reg.setMinClusterSize (100); //set the size of the class

    //Here is the place where the algorithm is launched. It will return the array of clusters when the segmentation process will be over.
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;


    //The robot has the largest number of points
    //decide which cluster is the robot
    unsigned int point_number_robot=0;
    unsigned int cluster_number_robot=0;
    point_number_robot = clusters[0].indices.size();
    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
        if (point_number_robot < clusters[i].indices.size())
        {
            point_number_robot = clusters[i].indices.size();
            cluster_number_robot=i;
        }
    }


    //save the points of robot into robot_cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr robot_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr contact_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    robot_cluster->width = clusters[cluster_number_robot].indices.size();
    robot_cluster->height = 1;
    robot_cluster->is_dense = true;
    for (unsigned int j = 0; j < clusters[cluster_number_robot].indices.size(); ++j)
    {
        //Take the corresponding point of the filtered cloud from the indices for the new pcl
        robot_cluster->push_back(filteredpoints->at(clusters[cluster_number_robot].indices[j]));
    }
    //set the robot colour as blue
    for (unsigned int i = 0; i < robot_cluster->points.size(); i++)
    {
        robot_cluster->points[i].x = robot_cluster->points[i].x;
        robot_cluster->points[i].y = robot_cluster->points[i].y;
        robot_cluster->points[i].z = robot_cluster->points[i].z;
        robot_cluster->points[i].r = 0;
        robot_cluster->points[i].g = 0;
        robot_cluster->points[i].b = 255;//user-defined blue
    }


    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor_radius;
    sor_radius.setInputCloud(robot_cluster);
    sor_radius.setRadiusSearch(0.005);
    sor_radius.setMinNeighborsInRadius(23);
    sor_radius.filter(*robot_cluster);

    //save the points of contact into contact_cluster
    int contact_cluster_width = 0;
    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
        if (i != cluster_number_robot)
        {
            contact_cluster_width = contact_cluster_width + clusters[i].indices.size();
        }
    }
    contact_cluster->width = contact_cluster_width;
    contact_cluster->height = 1;
    contact_cluster->is_dense = true;

    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
        if (i != cluster_number_robot)
        {
            for (unsigned int j = 0; j < clusters[i].indices.size(); ++j)
            {
                contact_cluster->push_back(filteredpoints->at(clusters[i].indices[j]));
            }
        }
    }
    //set the contact colour as red
    for (unsigned int i = 0; i < contact_cluster->points.size(); i++)
    {
        contact_cluster->points[i].x = contact_cluster->points[i].x;
        contact_cluster->points[i].y = contact_cluster->points[i].y;
        contact_cluster->points[i].z = contact_cluster->points[i].z;
        contact_cluster->points[i].r = 255;
        contact_cluster->points[i].g = 0;
        contact_cluster->points[i].b = 0;//red
    }
    sor_radius.setInputCloud(contact_cluster);
    sor_radius.setRadiusSearch(0.005);
    sor_radius.setMinNeighborsInRadius(15);
    sor_radius.filter(*contact_cluster);

    ///////////////////convert the point cloud from the sensor coordinate system to robot coordinate system/////////////////////////////////
    pcl::transformPointCloud (*contact_cluster, *contact_cluster, transform_1);
    pcl::transformPointCloud (*robot_cluster, *robot_cluster, transform_1);
    //////////////////////////////////////////////////////////////////Image segmentation end/////////////////////////////////////////////////


    ////////////////////////////////////////////////////////image processing begin////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////compute the normal direction for each effector/////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr robot_effectors (new pcl::PointCloud<pcl::PointXYZRGB> ());
    unsigned int EffectorNumber = 51;
    robot_effectors->resize(EffectorNumber);//number of feature points

    ReadAccessor<Data<VecCoord>> effectors = d_effectorPositions; // the position vector of mechanical object

    for (unsigned int i=0;i<robot_effectors->points.size();i++)
    {
        robot_effectors->points[i].x = effectors[i].x()/1000.0;
        robot_effectors->points[i].y = effectors[i].y()/1000.0;
        robot_effectors->points[i].z = effectors[i].z()/1000.0;
        robot_effectors->points[i].r = 255;
        robot_effectors->points[i].g = 255;
        robot_effectors->points[i].b = 255;
    }

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (robot_effectors);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    //add point cloud to Kdtree
    treeNormal->setInputCloud (robot_effectors);
    ne.setSearchMethod (treeNormal);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //The number of neighbour points which need to be searched when to compute the normal direction
    ne.setKSearch (3);
    // Compute the features
    ne.compute (*cloud_normals);

    WriteOnlyAccessor<Data<VecCoord>> normals = d_normalDirections;
    normals.resize(cloud_normals->points.size());
    for( unsigned int i = 0 ; i < cloud_normals->points.size() ; i++ )
    {
        normals[i].set( cloud_normals->points[i].normal_x,
                        cloud_normals->points[i].normal_y,
                        cloud_normals->points[i].normal_z);
    }

    //////////////////////////////////////////////compute the desired 3D position (in real robot point cloud) for each FEMeffector/////////////////////////////////
    //define desired position for each effector
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr robot_desiredeffectors (new pcl::PointCloud<pcl::PointXYZRGB> ());
    robot_desiredeffectors->resize(EffectorNumber);//number of feature points

    //detect the hinden effectors
    double REx;
    double REy;
    double REz;
    double ENx;
    double ENy;
    double ENz;
    double RENx;
    double RENy;
    double RENz;
    double RENdistance;

    for(unsigned int i=0; i<EffectorNumber; i++)
    {
        double smallestRENdistance = 0.004;
        int IndexDesiredPoint = 0;
        for(unsigned int j = 0; j < robot_cluster->points.size(); j++)
        {
            //compute the distance between one point in robot_cluster and the effector point
            REx = robot_cluster->points[j].x - robot_effectors->points[i].x;
            REy = robot_cluster->points[j].y - robot_effectors->points[i].y;
            REz = robot_cluster->points[j].z - robot_effectors->points[i].z;

            ENx = cloud_normals->points[i].normal_x;
            ENy = cloud_normals->points[i].normal_y;
            ENz = cloud_normals->points[i].normal_z;

            RENx = REy*ENz - REz*ENy;
            RENy = REz*ENx - REx*ENz;
            RENz = REx*ENy - REy*ENx;

            RENdistance = sqrt(RENx*RENx + RENy*RENy + RENz*RENz)/sqrt(ENx*ENx + ENy*ENy + ENz*ENz);

            if (RENdistance < smallestRENdistance)
            {
                smallestRENdistance = RENdistance;
                IndexDesiredPoint = j;
            }
        }

        if(IndexDesiredPoint == 0) //the effector is hiden
        {
            robot_desiredeffectors->points[i].x = robot_effectors->points[i].x;
            robot_desiredeffectors->points[i].y = robot_effectors->points[i].y;
            robot_desiredeffectors->points[i].z = robot_effectors->points[i].z;
            robot_desiredeffectors->points[i].r = 255;
            robot_desiredeffectors->points[i].g = 255;
            robot_desiredeffectors->points[i].b = 0;

        }
        if(IndexDesiredPoint != 0)
        {
            robot_desiredeffectors->points[i].x = robot_cluster->points[IndexDesiredPoint].x;
            robot_desiredeffectors->points[i].y = robot_cluster->points[IndexDesiredPoint].y;
            robot_desiredeffectors->points[i].z = robot_cluster->points[IndexDesiredPoint].z;
            robot_desiredeffectors->points[i].r = 0;
            robot_desiredeffectors->points[i].g = 255;
            robot_desiredeffectors->points[i].b = 0;//green
        }

    }

    WriteOnlyAccessor<Data<VecCoord>> goals = d_goalPositions;
    goals.resize(robot_desiredeffectors->points.size());
    for(unsigned int i=0;i<robot_desiredeffectors->points.size();i++)
    {
        goals[i].set(1000*robot_desiredeffectors->points[i].x,
                     1000*robot_desiredeffectors->points[i].y,
                     1000*robot_desiredeffectors->points[i].z);
    }

    ///////////////////////////////////////////////////////compute the desired 3D position (in real robot point cloud) for each FEMeffector end////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////find if there are contacts and find the index of contact locations/////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////find if there are contacts and find the index of contact locations/////////////////////////////////////////////////////////////////////
    //define initial 6 contact locations [0,0,0,0,0,0]
    unsigned int LargestNumContact = 51;
    VectorXd IndexContactLocation;
    IndexContactLocation.resize(LargestNumContact);
    for(unsigned int i = 0; i < LargestNumContact; i++)
    {
        IndexContactLocation(i) = 0;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr robot_initialContactLocation (new pcl::PointCloud<pcl::PointXYZRGB>);
    robot_initialContactLocation->resize(robot_cluster->points.size());//number of feature points

    int NumberContacts = 0;//number of contact points obtained from the robot cloud


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ContactLocation_fromRobot (new pcl::PointCloud<pcl::PointXYZRGB>);


    if(contact_cluster->width != 0)
    {
        //search in the set of contact_cluster to find effector which is the most closet to the contact points
        // Neighbors within radius search
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_location;
        kdtree_location.setInputCloud (contact_cluster);
        float radius = 0.004;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        for(unsigned int i = 0; i < robot_cluster->points.size(); i++)
        {
            if ( kdtree_location.radiusSearch (robot_cluster->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
                //If there are more than 20 points in the contact set, the effector is regarded as the location
                if(pointIdxRadiusSearch.size() > 3)
                {
                    robot_initialContactLocation->points[NumberContacts].x = robot_cluster->points[i].x;
                    robot_initialContactLocation->points[NumberContacts].y = robot_cluster->points[i].y;
                    robot_initialContactLocation->points[NumberContacts].z = robot_cluster->points[i].z;
                    NumberContacts = NumberContacts + 1;
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr robot_ContactLocation (new pcl::PointCloud<pcl::PointXYZRGB> ());
        robot_ContactLocation->resize(NumberContacts);//number of feature points

        for(int i = 0; i < NumberContacts; i++)
        {
            robot_ContactLocation->points[i].x = robot_initialContactLocation->points[i].x;
            robot_ContactLocation->points[i].y = robot_initialContactLocation->points[i].y;
            robot_ContactLocation->points[i].z = robot_initialContactLocation->points[i].z;
            robot_ContactLocation->points[i].r = 255;
            robot_ContactLocation->points[i].g = 0;
            robot_ContactLocation->points[i].b = 255;
        }


        //Euclidean Cluster Extraction

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeEuclidean (new pcl::search::KdTree<pcl::PointXYZRGB>);
        treeEuclidean->setInputCloud (robot_ContactLocation);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.005); //set the search redius 5mm
        ec.setMinClusterSize (10);//set the minimum number of points in one cluster 20
        ec.setMaxClusterSize (2000);//set the maximum number of points in one cluster2000
        ec.setSearchMethod (treeEuclidean);//set the search method
        ec.setInputCloud (robot_ContactLocation);
        ec.extract (cluster_indices);//get the cluster from the point cloud and save the index in cluster_indices

        std::cout << "number of  Cluster: " << cluster_indices.end() - cluster_indices.begin()<< std::endl;

        VectorXd ContactLocationfromRobot; //the location obtained from robot point cloud
        ContactLocationfromRobot.resize(3*(cluster_indices.end() - cluster_indices.begin()));
        static VectorXd ContactLocationfromRobotEMA; //the location obtained from robot point cloud
        ContactLocationfromRobotEMA.resize(3*(cluster_indices.end() - cluster_indices.begin()));

        static int count = 0;
        count = count + 1;

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr ContactLocation_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                ContactLocation_cluster->points.push_back (robot_ContactLocation->points[*pit]);

            //compute the average 3D position of all points in one cluster
            double averagex = 0;
            double averagey = 0;
            double averagez = 0;
            for(unsigned int i = 0; i < ContactLocation_cluster->points.size (); i++)
            {
                averagex = averagex + ContactLocation_cluster->points[i].x;
                averagey = averagey + ContactLocation_cluster->points[i].y;
                averagez = averagez + ContactLocation_cluster->points[i].z;
            }

            ContactLocationfromRobot(3*j) = averagex/ContactLocation_cluster->points.size ();
            ContactLocationfromRobot(3*j+1) = averagey/ContactLocation_cluster->points.size ();
            ContactLocationfromRobot(3*j+2) = averagez/ContactLocation_cluster->points.size ();


            //find the effector of FEM which is closet to the ContactLocationfromRobot
            double InitailDistance;

            InitailDistance = (ContactLocationfromRobot(3*j)-robot_effectors->points[0].x)*(ContactLocationfromRobot(3*j)-robot_effectors->points[0].x) + (ContactLocationfromRobot(3*j+1)-robot_effectors->points[0].y)*(ContactLocationfromRobot(3*j+1)-robot_effectors->points[0].y) + (ContactLocationfromRobot(3*j+2)-robot_effectors->points[0].z)*(ContactLocationfromRobot(3*j+2)-robot_effectors->points[0].z);

            double Distance;
            int indexofcontact=0;
            for (unsigned int i=0;i<robot_effectors->points.size();i++)
            {
                Distance = (ContactLocationfromRobot(3*j)-robot_effectors->points[i].x)*(ContactLocationfromRobot(3*j)-robot_effectors->points[i].x) + (ContactLocationfromRobot(3*j+1)-robot_effectors->points[i].y)*(ContactLocationfromRobot(3*j+1)-robot_effectors->points[i].y) + (ContactLocationfromRobot(3*j+2)-robot_effectors->points[i].z)*(ContactLocationfromRobot(3*j+2)-robot_effectors->points[i].z);
                if( InitailDistance > Distance)
                {
                    InitailDistance = Distance;
                    indexofcontact = i;
                }
            }

            IndexContactLocation(indexofcontact) = 1;

            j++;
        }

        ContactLocation_fromRobot->resize(j);
        for(int i = 0; i < j; i++)
        {
            ContactLocation_fromRobot->points[i].x = ContactLocationfromRobot(3*i);
            ContactLocation_fromRobot->points[i].y = ContactLocationfromRobot(3*i+1);
            ContactLocation_fromRobot->points[i].z = ContactLocationfromRobot(3*i+2);
            ContactLocation_fromRobot->points[i].r = 255;
            ContactLocation_fromRobot->points[i].g = 255;
            ContactLocation_fromRobot->points[i].b = 0;//yellow
        }

    }

    WriteOnlyAccessor<decltype(d_contactLocations)> locations = d_contactLocations;
    locations.resize(LargestNumContact);
    for(unsigned int i = 0; i < LargestNumContact; i++)
    {
        locations[i] = IndexContactLocation(i);
    }

}


void PointCloudProcessing::handleEvent(core::objectmodel::Event *event)
{
    if (sofa::simulation::AnimateEndEvent::checkEventType(event))
    {
        update();
    }
}


////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template
using namespace sofa::defaulttype;

static int PointCloudProcessingClass = core::RegisterObject("Exposing a RealSense RGBD camera in the Sofa Scene")
        .add<PointCloudProcessing>();\
//////////////////////////////////////////////////////////////////////////////////////////////////////


} // namespace behavior

} // namespace core

} // namespace sofa
