/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidarSensor = new Lidar (cars,  0);

    // Do a scan of the LiDAR
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidarSensor->scan();
    
    //renderRays(viewer, lidarSensor->position, pointCloud);
    //renderPointCloud(viewer, pointCloud,"inputPointCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();


    // ***** Segmentation *****

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointCloudProcessor->SegmentPlane(pointCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(1,1,0));

    
    // **** Clustering ****

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    // loop through all clusters and render each
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointCloudProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // render additional BoundingBox around the pointcloud cluster
        BoxQ boundingBox = pointCloudProcessor->BoundingBoxQ(cluster);
        //std::cout << "minPoint (x,y,z) of the Box is: (" << boundingBoxQ.x_min << ", " << boundingBoxQ.y_min << ", " << boundingBox.z_min << ")" << endl;

        renderBox(viewer, boundingBox, clusterId);

        ++clusterId;
    }


}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Create pointCloudProcessor and load pcd file
    //ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointCloudProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // filter loaded point cloud (lower point-cloud density, crop out smaller ROI)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudProcessorI->FilterCloud(inputCloud, 0.3f, Eigen::Vector4f(-15.0f,-7.0f,-2.0f,1), Eigen::Vector4f(30.0f,7.0f,4.0f,1));

    // ***** Segmentation *****

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointCloudProcessorI->MySegmentPlane(filteredCloud, 100, 0.2);
    pointCloudProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentedCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentedCloud.second,"planeCloud",Color(0,1,0));


    // **** Clustering ****

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointCloudProcessorI->MyClustering(segmentedCloud.first, 0.53, 10, 10000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    // loop through all clusters and render each
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointCloudProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

        std::cout << "colorID: " << clusterId % 3 << std::endl;

        // render additional BoundingBox around the pointcloud cluster
        Box boundingBox = pointCloudProcessorI->BoundingBox(cluster);
        //std::cout << "minPoint (x,y,z) of the Box is: (" << boundingBoxQ.x_min << ", " << boundingBoxQ.y_min << ", " << boundingBox.z_min << ")" << endl;

        renderBox(viewer, boundingBox, clusterId);

        ++clusterId;
    }

    // render pointCloud
    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // render filtered pointCloud instead

    //renderPointCloud(viewer, filteredCloud, "filteredCould");

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Create pointCloudProcessor and load pcd file
    ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointCloudProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // filter loaded point cloud (lower point-cloud density, crop out smaller ROI)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudProcessorI->FilterCloud(inputCloud, 0.3f, Eigen::Vector4f(-15.0f,-7.0f,-2.0f,1), Eigen::Vector4f(30.0f,7.0f,4.0f,1));

    // ***** Segmentation *****

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointCloudProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentedCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentedCloud.second,"planeCloud",Color(0,1,0));


    // **** Clustering ****

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters_org = pointCloudProcessorI->Clustering(segmentedCloud.first, 0.53, 10, 10000);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointCloudProcessorI->MyClustering(segmentedCloud.first, 0.53, 10, 10000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    // loop through all clusters and render each
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        pointCloudProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

        //std::cout << "colorID: " << clusterId % 3 << std::endl;

        // render additional BoundingBox around the pointcloud cluster
        Box boundingBox = pointCloudProcessorI->BoundingBox(cluster);
        //std::cout << "minPoint (x,y,z) of the Box is: (" << boundingBoxQ.x_min << ", " << boundingBoxQ.y_min << ", " << boundingBox.z_min << ")" << endl;

        renderBox(viewer, boundingBox, clusterId);

        ++clusterId;
    }

    // render pointCloud
    //renderPointCloud(viewer,inputCloud,"inputCloud");

    // render filtered pointCloud instead

    //renderPointCloud(viewer, filteredCloud, "filteredCould");

}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}






int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);  // single frame

    ProcessPointClouds<pcl::PointXYZI>* pointCloudProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointCloudProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    


    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointCloudProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointCloudProcessorI, inputCloudI);

        // Next frame
        streamIterator++;

        // at the end -> jump back to first frame
        if (streamIterator == stream.end())
            streamIterator = stream.begin();


        viewer->spinOnce ();
    } 
}