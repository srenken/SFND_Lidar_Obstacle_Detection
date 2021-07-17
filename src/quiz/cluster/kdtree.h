/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
			// Decide why axis to split (x = 0; y = 1)
			int axisBasedOnDepth = depth % 2;

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
			// is the point in the box? 2 x distanceTol in x and y direction
			if ((*node)->point[0] < (target[0] + distanceTol) 
				&& (*node)->point[0] > (target[0] - distanceTol)
				&& (*node)->point[1] < (target[1] + distanceTol)
				&& (*node)->point[1] > (target[1] - distanceTol))
			{
				// Calc euclidean distance
				float euclDist = sqrt(pow(((*node)->point[0]-target[0]),2) + pow(((*node)->point[1]-target[1]),2));
				if (euclDist <= distanceTol)
					ids->push_back((*node)->id);
			}

			
			// check left branch, if point x or y value (based on depth) is within box
			if((target[depth%2]-distanceTol) < (*node)->point[depth%2])
			{
				recursiveSearch(&((*node)->left), ids, depth+1, target, distanceTol);
			}

			// check right branch, if point x or y value (based on depth) is within box
			if((target[depth%2]+distanceTol) > (*node)->point[depth%2])
			{
				recursiveSearch(&((*node)->right), ids, depth+1, target, distanceTol);
			}
			
			
		}
	}
	

};




