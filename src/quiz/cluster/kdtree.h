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
        return ids;
    }
private:

    enum class SplitAxis {
        X_AXIS = 0,
        Y_AXIS = 1,
    };

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




