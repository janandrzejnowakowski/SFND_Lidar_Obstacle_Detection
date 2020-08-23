// Structure to represent node of kd tree

template <typename PointT>
struct Node
{
    PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT point, int setId)
	:	point(point), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void insert(PointT point, int id)
	{
		insertHelper(&root, point, id, 0);
	}

    void insertHelper(Node<PointT>** node, PointT point, int id, int level)
    {
        if (*node == NULL)  {
            *node = new Node<PointT>(point, id);
        } else {
            uint idx = level % 3;
            float node_value, point_value;
            if (idx % 3 == 0) {
                node_value = (*node)->point.x;
                point_value = point.x;
            } else if (idx % 3 == 1) {
                node_value = (*node)->point.y;
                point_value = point.y;
            } else {
                node_value = (*node)->point.z;
                point_value = point.z;
            }
            if (node_value <= point_value) {
                insertHelper(&((*node)->right), point, id, level + 1);
            } else {
                insertHelper(&((*node)->left), point, id, level + 1);
            }
        }
    }

    void searchHelper(Node<PointT>* node, PointT point, float distanceTol, pcl::PointIndices& ids, int level) {
	    if (node == NULL) return;
	    uint idx = level % 3;

	    float node_value, point_value;
        if (idx % 3 == 0) {
            node_value = node->point.x;
            point_value = point.x;
        } else if (idx % 3 == 1) {
            node_value = node->point.y;
            point_value = point.y;
        } else {
            node_value = node->point.z;
            point_value = point.z;
        }

	    if (node_value - distanceTol > point_value) {
            searchHelper(node->left, point, distanceTol, ids, level + 1);
	    } else if (node_value + distanceTol < point_value) {
            searchHelper(node->right, point, distanceTol, ids, level + 1);
	    } else {
	        if (sqrt(pow(node->point.x - point.x, 2) + pow(node->point.y - point.y, 2) + pow(node->point.z - point.z, 2)) < distanceTol)
	            ids.indices.push_back(node->id);
            searchHelper(node->left, point, distanceTol, ids, level + 1);
            searchHelper(node->right, point, distanceTol, ids, level + 1);
	    }
	}

	// return a list of point ids in the tree that are within distance of point
	pcl::PointIndices search(PointT point, float distanceTol)
	{
		pcl::PointIndices ids;
        searchHelper(root, point, distanceTol, ids, 0);
		return ids;
	}
};




