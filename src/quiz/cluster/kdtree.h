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

    void insert(std::vector<float> point, int id)
    {
        insert(&root, std::move(point), id, SplitAxis::X_AXIS);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        search(target, distanceTol, ids, root, SplitAxis::X_AXIS);
        return ids;
    }
private:

    enum class SplitAxis {
        X_AXIS = 0,
        Y_AXIS = 1,
    };

    static float distanceSq(std::vector<float> target, std::vector<float> source) {
        return pow(target[0] - source[0], 2) + pow(target[1] - source[1], 2);
    }

    static bool closeEnough(std::vector<float> target, std::vector<float> source, float distanceTol) {
        return distanceSq(std::move(target), std::move(source)) <= (distanceTol*distanceTol);
    }

    void search(std::vector<float> target, float distanceTol, std::vector<int>& ids, Node* node, SplitAxis axis) {
        if (node == nullptr) {
            return;
        }
        if (closeEnough(target, node->point, distanceTol)) {
            ids.push_back(node->id);
        }
        const int index = static_cast<int>(axis);
        // Now check which (one or both) sides we want to descend, based on distanceTol*2 in the direction of axis.
        const float extent_min = target[index] - distanceTol;
        const float extent_max = target[index] + distanceTol;
        // The tree biases insertions with equal x/y values as the node towards the left child, so remember to do the same during search.
        const SplitAxis next_axis = axis == SplitAxis::X_AXIS ? SplitAxis::Y_AXIS : SplitAxis::X_AXIS;
        if (extent_min <= node->point[index]) {
            // Need to descend left.
            search(target, distanceTol, ids, node->left, next_axis);
        }
        if (extent_max >= node->point[index]) {
            search(target, distanceTol, ids, node->right, next_axis);
        }
    }

    void insert(Node* node_ptr, std::vector<float> point, int id, SplitAxis axis) {
        assert(node_ptr != nullptr);
        if (axis == SplitAxis::X_AXIS) {
            if (point[0] <= node_ptr->point[0]) {
                insert(&node_ptr->left, std::move(point), id, SplitAxis::Y_AXIS);
            } else {
                insert(&node_ptr->right, std::move(point), id, SplitAxis::Y_AXIS);
            }
        } else if (axis == SplitAxis::Y_AXIS) {
            if (point[1] <= node_ptr->point[1]) {
                insert(&node_ptr->left, std::move(point), id, SplitAxis::X_AXIS);
            } else {
                insert(&node_ptr->right, std::move(point), id, SplitAxis::X_AXIS);
            }
        }
    }

    void insert(Node** node_ptr_ref, std::vector<float> point, int id, SplitAxis axis) {
        if (*node_ptr_ref == nullptr) {
            *node_ptr_ref = new Node(std::move(point), id);
        } else {
            insert(*node_ptr_ref, std::move(point), id, axis);
        }
    }


};




