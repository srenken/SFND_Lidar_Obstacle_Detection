#include "kdtree.h"
#include <pcl/common/common.h>
#include <vector>
#include <algorithm>


// ************** Clustering Helper functions

void findPointsInProximity(int pointIdx, const std::vector<std::vector<float>> points, std::vector<int>& cluster, bool bPointsProcessed[], KdTree* tree, float distanceTol)
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
			findPointsInProximity(pointsNearby[i], points, cluster, bPointsProcessed, tree, distanceTol);
		}
	}

}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
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

			findPointsInProximity(pointIdx, points, cluster, bPointProcessed, tree, distanceTol);

			// append new cluster to the vector of clusters
			clusters.push_back(cluster);
		}

	}

	return clusters;

}


// **************