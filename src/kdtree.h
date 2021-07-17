/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_

// #include "../../render/render.h"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}

};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void recursiveInsert(Node **node, int depth, std::vector<float> arr, int setId)
	{

		// If node is empty, create new node assign arr = point
		if(*node == NULL)
		{
			*node = new Node(arr, setId);
		}
		else 
		{
			// Decide why axis to split (x = 0; y = 1; z = 2)
			int axisBasedOnDepth = depth % 3;

			// 
			if(arr[axisBasedOnDepth] < (*node)->point[axisBasedOnDepth])
			{
				recursiveInsert(&((*node)->left), depth + 1, arr, setId);
			}
			else
			{
				recursiveInsert(&((*node)->right), depth + 1, arr, setId);
			}
		}
	}
	
	void insert(std::vector<float> arr, int setId)
	{
		recursiveInsert(&root, 0, arr, setId);
	}	

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;		

		recursiveSearch(&root, &ids, 0, target, distanceTol);
		
		return ids;
	}

	void recursiveSearch(Node **node, std::vector<int> *ids, int depth, std::vector<float> target, float distanceTol)
	{
		
		if(*node == NULL)
		{
			return;
		}
		else
		{
			// is the point in the cube? 2 x distanceTol in x, y and z direction
			if ((*node)->point[0] < (target[0] + distanceTol) 
				&& (*node)->point[0] > (target[0] - distanceTol)
				&& (*node)->point[1] < (target[1] + distanceTol)
				&& (*node)->point[1] > (target[1] - distanceTol)
				&& (*node)->point[2] < (target[2] + distanceTol)
				&& (*node)->point[2] > (target[2] - distanceTol))
			{
				// Calc euclidean distance
				float euclDist = sqrt(pow(((*node)->point[0]-target[0]),2) + pow(((*node)->point[1]-target[1]),2) + pow(((*node)->point[2]-target[2]),2));
				if (euclDist <= distanceTol)
					ids->push_back((*node)->id);
			}

			
			// check left branch, if point x or y or z value (based on depth) is within box
			if((target[depth%3]-distanceTol) < (*node)->point[depth%3])
			{
				recursiveSearch(&((*node)->left), ids, depth+1, target, distanceTol);
			}

			// check right branch, if point x or y or z value (based on depth) is within box
			if((target[depth%3]+distanceTol) > (*node)->point[depth%3])
			{
				recursiveSearch(&((*node)->right), ids, depth+1, target, distanceTol);
			}
			
			
		}
	}
	

};
#endif /* KDTREE_H_ */



