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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, point, id, 0);
	}

    void insertHelper(Node** node, std::vector<float> point, int id, int level)
    {
        if (*node == NULL)  {
            *node = new Node(point, id);
        } else {
            uint idx = level % 2;
            if ((*node)->point[idx] <= point[idx]) {
                insertHelper(&((*node)->right), point, id, level + 1);
            } else {
                insertHelper(&((*node)->left), point, id, level + 1);
            }
        }
    }

    void searchHelper(Node* node, std::vector<float> target, float distanceTol, std::vector<int>& ids, int level) {
	    if (node == NULL) return;
	    uint idx = level % 2;
	    if (node->point[idx] - distanceTol > target[idx]) {
            searchHelper(node->left, target, distanceTol, ids, level + 1);
	    } else if (node->point[idx] + distanceTol < target[idx]) {
            searchHelper(node->right, target, distanceTol, ids, level + 1);
	    } else {
	        if (sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2)) < distanceTol)
	            ids.push_back(node->id);
            searchHelper(node->left, target, distanceTol, ids, level + 1);
            searchHelper(node->right, target, distanceTol, ids, level + 1);
	    }

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(root, target, distanceTol, ids, 0);
		return ids;
	}

};




