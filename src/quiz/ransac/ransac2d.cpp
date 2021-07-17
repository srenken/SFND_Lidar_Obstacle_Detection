/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include <array>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	std::cout << "Num of Points in Cloud: " << cloud->points.size() << std::endl;

	// TODO: Fill in this function

	// For max iterations 
	for (int i=0; i<=maxIterations; i++)
	{
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line
		while (inliers.size()<2)
		{
			int randomIndex = rand()%cloud->points.size(); 
			inliers.insert(randomIndex);
			std::cout << "RandomIndex: " << randomIndex << std::endl;
		} 

		auto inliersIndex = inliers.begin();

		pcl::PointXYZ point1 = cloud->points[*inliersIndex];
		inliersIndex++;
		pcl::PointXYZ point2 = cloud->points[*inliersIndex];

		// (y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)=0
		// calc fitted line coeffs
		double A = point1.y - point2.y;
		double B = point2.x - point1.x;
		double C = point1.x * point2.x - point2.x * point1.y;

		
		
		// Measure distance between every point and fitted line
		for(int j=0; j<=cloud->points.size(); j++)
		{
			// If point is not already part of the line
			if (inliers.count(j) == 0)
			{
				//d=∣Ax+By+C∣/sqrt(pow(A,2)+pow(B,2))
				double distance = std::fabs(A * cloud->points[j].x + B * cloud->points[j].y + C) / sqrt(pow(A, 2) + pow(B,2));

				// If distance is smaller than threshold count it as inlier
				if (distance < distanceTol) 
				{	
					inliers.insert(j);
				}
			}
			
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
		
	}
	

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	std::cout << "Num of Points in Cloud: " << cloud->points.size() << std::endl;

	// TODO: Fill in this function

	// For max iterations 
	for (int i=0; i<=maxIterations; i++)
	{
		std::cout << std::endl << "----------------------------------" << std::endl << "Iteration: " << i << std::endl;

		std::unordered_set<int> inliers;

		// Randomly sample subset and fit plane
		while (inliers.size()<3)
		{
			int randomIndex = rand()%cloud->points.size(); 
			inliers.insert(randomIndex);
			std::cout << "RandomIndex: " << randomIndex << std::endl;
		} 

		auto inliersIndex = inliers.begin();

		pcl::PointXYZ point1 = cloud->points[*inliersIndex];
		inliersIndex++;
		pcl::PointXYZ point2 = cloud->points[*inliersIndex];
		inliersIndex++;
		pcl::PointXYZ point3 = cloud->points[*inliersIndex];

		
		
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
				if (distance < distanceTol) 
				{	
					inliers.insert(j);
				}
			}
			
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
		
	}


	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "My own implemented plane segmentation using RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 20, 1.0);

	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D/3D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
