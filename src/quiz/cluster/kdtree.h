/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;//0: x, 1: y, 2:z
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node ** currNode, uint depth,std::vector<float> point, int id)
	{
		if(*currNode == NULL)
		{
			*currNode = new Node(point, id);
		}
		else
		{
			Node ** nodeToAdd = NULL;
			if(point[depth % point.size()] >= (*currNode)->point[depth % point.size()])
			{
				nodeToAdd = &((*currNode)->right);
			}
			else
			{
				nodeToAdd = &((*currNode)->left);	
			}
			insertHelper(nodeToAdd, depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);	
	}

	bool isInsideBox(std::vector<float> point, std::vector<float> target, float distanceTol)
	{
		for(int dimension = 0;dimension < point.size();++dimension)
		{
			float minN = target[dimension] - distanceTol;
			float maxN = target[dimension] + distanceTol;
			if(point[dimension] > maxN || point[dimension] < minN)
			{
				return false;
			}
		}
		return true;
	}

	float computeDist(std::vector<float> point1, std::vector<float> point2)
	{
		float retDist = 0;
		for(int dimension = 0;dimension < point1.size();++dimension)
		{
			float subN = point1[dimension] - point2[dimension];
			retDist += subN * subN;
		}

		retDist = sqrt(retDist);
		return retDist;
	}

	void searchRec(Node* currNode, uint level, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if(currNode == NULL)
			return;
		if(isInsideBox(currNode->point, target, distanceTol))
		{
			float dist = computeDist(target, currNode->point);
			if(dist <= distanceTol)
			{
				ids.push_back(currNode->id);
			}
		}
		uint numDimensions = target.size();
		if(currNode->point[level%numDimensions] >= (target[level%numDimensions]-distanceTol))
		{
			searchRec(currNode->left, level + 1, target, distanceTol, ids);
		}
		if(currNode->point[level%numDimensions] <= (target[level%numDimensions]+distanceTol))
		{
			searchRec(currNode->right, level + 1, target, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		Node* currNode = root;
		searchRec(root, 0, target, distanceTol, ids);
		
		return ids;
	}
	

};




