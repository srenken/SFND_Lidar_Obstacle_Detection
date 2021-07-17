// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"
#include <unordered_set>
#include <array>
#include <pcl/impl/point_types.hpp>
//#include "cluster.cpp"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void ProcessPointClouds<PointT>::findPointsInProximity(int pointIdx, const std::vector<std::vector<float>> points, std::vector<int>& cluster, bool bPointsProcessed[], KdTree* tree, float distanceTol)
{
	/* mark point as processed
    add point to cluster
    nearby points = tree(point)
    Iterate through each nearby point
        If point has not been processed
            Proximity(cluster) */

	// mark point as processed
	bPointsProcessed[pointIdx] = true;

	// add point to cluster
	cluster.push_back(pointIdx);

	pcl::Indices pointsNearby = tree->search(points[pointIdx], distanceTol);

	for (int i = 0; i < pointsNearby.size(); i++)
	{
		// if point hasn't been processed yet, look for additional points nearby this to add to the cluster as well	
		if(bPointsProcessed[pointsNearby[i]] != true)
		{
			ProcessPointClouds<PointT>::findPointsInProximity(pointsNearby[i], points, cluster, bPointsProcessed, tree, distanceTol);
		}
	}

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	bool bPointProcessed[points.size()] = {};
	std::vector<std::vector<int>> clusters;
 
	for(int pointIdx = 0; pointIdx < points.size(); pointIdx++)
	{

		// Create new cluster if point was not yet processed
		if (bPointProcessed[pointIdx] != true)
		{
			// Create new cluster
			std::vector<int> cluster;

			ProcessPointClouds<PointT>::findPointsInProximity(pointIdx, points, cluster, bPointProcessed, tree, distanceTol);

			// append new cluster to the vector of clusters
			clusters.push_back(cluster);
		}

	}

	return clusters;

}



template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud_without_roof(new pcl::PointCloud<PointT>());
    pcl::Indices roofIndices;

    // voxel grid point reduction
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setInputCloud (cloud);
    voxelgrid.setLeafSize (filterRes, filterRes, filterRes);
    voxelgrid.filter (*filtered_cloud);

    // crop point cloud
    pcl::CropBox<PointT> pointCloudCropper(true);
    pointCloudCropper.setInputCloud(filtered_cloud);
    pointCloudCropper.setMin(minPoint);
    pointCloudCropper.setMax(maxPoint);
    pointCloudCropper.filter(*cropped_cloud);

    // remove roof points
    /* pointCloudCropper.setInputCloud(cropped_cloud);
    pointCloudCropper.setMin(Eigen::Vector4f(-2.0f,-1.3f,-0.5f,1));
    pointCloudCropper.setMax(Eigen::Vector4f(2.0f, 1.3f, 0.1f, 1));
    pointCloudCropper.filter(roofIndices);

    pcl::ExtractIndices<PointT> indciesExtracter;
    indciesExtracter.setInputCloud(cropped_cloud);
    indciesExtracter.setIndices(roofIndices);
    indciesExtracter.filter(*cropped_cloud_without_roof); */


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>* obstaclePointCloud = new pcl::PointCloud<PointT>();
    typename pcl::PointCloud<PointT>* groundPlanePointCloud = new pcl::PointCloud<PointT>();

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*groundPlanePointCloud);

    // Extract non-inliers (negative point cloud)
    extract.setNegative (true);
    extract.filter (*obstaclePointCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclePointCloud, groundPlanePointCloud);
    return segResult;
}





template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Set input cloud for Segmentation
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // init variables

	std::unordered_set<int> inliersResult;
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> pair_SegClouds;  // (1) Obstacle, (2) Ground Plane
	srand(time(NULL));
	
	// std::cout << "Num of Points in Cloud: " << cloud->points.size() << std::endl;

	
	// For max iterations 
	for (int i=0; i<=maxIterations; i++)
	{
		// std::cout << std::endl << "----------------------------------" << std::endl << "Iteration: " << i << std::endl;

		std::unordered_set<int> inliers;
        // typename pcl::PointCloud<PointT>::Ptr  cloudInliersTemp(new pcl::PointCloud<PointT>());
        // typename pcl::PointCloud<PointT>::Ptr cloudOutliersTemp(new pcl::PointCloud<PointT>());

		// Randomly sample subset and fit plane
		while (inliers.size()<3)
		{
			int randomIndex = rand()%cloud->points.size(); 
			inliers.insert(randomIndex);
			// std::cout << "RandomIndex: " << randomIndex << std::endl;
		} 

		auto inliersIndex = inliers.begin();

		PointT point1 = cloud->points[*inliersIndex];
		inliersIndex++;
		PointT point2 = cloud->points[*inliersIndex];
		inliersIndex++;
		PointT point3 = cloud->points[*inliersIndex];

		
		
		// calc vectors defining the plan with point1 as reference
		// v1 = < x2 - x1, y2 - y1, z2 - z1 >
		std::array<float, 3> vector1 {point2.x - point1.x, point2.y - point1.y, point2.z - point1.z};

		// v2 = < x3 - x1, y3 - y1, z3 - z1 >
		std::array<float, 3> vector2 {point3.x - point1.x, point3.y - point1.y, point3.z - point1.z};

		// NormalVector of plane (cross product of v1 x v2)
		// vector1 x vector2 = <(y2−y1)(z3−z1)−(z2−z1)(y3−y1), (z2−z1)(x3−x1)−(x2−x1)(z3−z1), (z2-z1)(x3-x1)-(x2-x1)(z3-z1),(z2−z1)(x3−x1)−(x2−x1)(z3−z1), (x2−x1)(y3−y1)−(y2−y1)(x3−x1)>(x2-x1)(y3-y1)-(y2-y1)(x3-x1)>(x2−x1)(y3−y1)−(y2−y1)(x3−x1)>
		// vector1 x vector2 = < i, j, k >
		// vector1 x vector2 = 
		std::array<float, 3> normalVector {vector1[1]*vector2[2] - vector1[2]*vector2[1], vector1[2]*vector2[0] - vector1[0]*vector2[2], vector1[0]*vector2[1] - vector1[1]*vector2[0] }; 


		// Equation of a Plane through Three Points: Ax+By+Cz+D=0
		float A = normalVector[0]; // A = i
		float B = normalVector[1]; // B = j
		float C = normalVector[2]; // C = k
		float D = -1 * (A*point1.x + B*point1.y + C*point3.z);

		
		
		// Measure distance between every point and fitted plane
		for(int j=0; j<=cloud->points.size(); j++)
		{
			// If point is not already part of the line
			if (inliers.count(j) == 0)
            {
				//d=∣Ax+By+Cz+D∣/sqrt(pow(A,2)+pow(B,2)+pow(C,2))
				float distance = std::fabs(A * cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D) / sqrt(pow(A, 2) + pow(B,2) + pow(C,2));

				// If distance is smaller than threshold count it as inlier
				if (distance < distanceThreshold) 
				{	
					inliers.insert(j);
                    //cloudInliersTemp->points.push_back(cloud->points[j]);
				}
                //else
                  //  cloudOutliersTemp->points.push_back(cloud->points[j]);
			}
			
		}

		//if (cloudInliersTemp->points.size() > cloudInliers->points.size() )
		if (inliers.size() > inliersResult.size())
		{
        	//cloudInliers = cloudInliersTemp;
            //cloudOutliers = cloudOutliersTemp;
            inliersResult = inliers; 
		}
		
	}


	
	// Separate points into two point clouds

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "My own implemented plane segmentation using RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    pair_SegClouds = std::make_pair(cloudOutliers, cloudInliers);  // (1) Obstacle, (2) Ground Plane

    return pair_SegClouds;

}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // loop through cluster_indices Vector
    for(std::vector<pcl::PointIndices>::const_iterator clusterIterator = cluster_indices.begin(); clusterIterator != cluster_indices.end(); ++clusterIterator)
    {
        // 
        typename pcl::PointCloud<PointT>::Ptr clustered_cloud (new pcl::PointCloud<PointT>);

        // Loop through all indices in the cluster
        for (const auto& index : clusterIterator->indices)
        {
            clustered_cloud->push_back ((*cloud)[index]);
        }
        clustered_cloud->width = clustered_cloud->size ();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;

        clusters.push_back(clustered_cloud);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



// ----------------- My Clustering implementation -----------------



template<typename PointT>
std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> ProcessPointClouds<PointT>::MyClustering(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    std::vector<std::vector<float>> points;

    // Creating the KdTree object for the search method of the extraction 
    KdTree* tree = new KdTree;
  
    
    // Insert Cloud points into kdTree 
    for (int i=0; i<cloud->points.size(); i++) 
    {
    	tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
        points.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    }

    // Cluster
    std::vector<std::vector<int>> clusters_incices = ProcessPointClouds<PointT>::euclideanCluster(points, tree, clusterTolerance);

    // loop through all clusters and create out of indices the actual clusters as point clouds
	int clusterId = 0;

    for(std::vector<int> cluster : clusters_incices)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(cloud->points[indice]);

  		
        clusterCloud->width = clusterCloud->size ();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        if (clusterCloud->size () >= minSize && clusterCloud->size () <= maxSize)
            clusters.push_back(clusterCloud);  
          
        ++clusterId;
  	}



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "My clustering Implementation took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}





template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                ///    the signs are different and the box doesn't get correctly oriented in some cases.


    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());



    BoxQ box;
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();      // Eigen::Vector3f bboxTransform;
	box.bboxQuaternion = eigenVectorsPCA;     // Eigen::Quaternionf bboxQuaternion;
	box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z -  minPoint.z;

    return box;
}



template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}