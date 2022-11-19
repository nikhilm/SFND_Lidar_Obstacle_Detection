//
// Created by nikhil on 11/18/22.
//

#include <vector>
#include <cstddef>
#include <cmath>
#include <cassert>

#ifndef PLAYBACK_KDTREE3D_H
#define PLAYBACK_KDTREE3D_H

// Structure to represent node of kd tree
template <typename PointT>
struct Node {
    PointT point;
    int id;
    Node *left;
    Node *right;

    Node(PointT p, int setId)
            : point(p), id(setId), left(NULL), right(NULL) {}

    ~Node() {
        delete left;
        delete right;
    }
};

template <typename PointT>
struct KdTree {
    Node<PointT> *root;

    KdTree()
            : root(NULL) {}

    ~KdTree() {
        delete root;
    }

    void insert(PointT point, int id) {
        insert(&root, std::move(point), id, SplitAxis::X_AXIS);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol) {
        std::vector<int> ids;
        search(target, distanceTol, ids, root, SplitAxis::X_AXIS);
        return ids;
    }

private:

    enum class SplitAxis {
        X_AXIS = 0,
        Y_AXIS = 1,
        Z_AXIS = 2,
    };

    static float distanceSq(PointT target, PointT source) {
        return pow(target.x - source.x, 2) + pow(target.y - source.y, 2) + pow(target.z - source.z, 2);
    }

    static bool closeEnough(PointT target, PointT source, float distanceTol) {
        return distanceSq(std::move(target), std::move(source)) <= (distanceTol * distanceTol);
    }

    float pointDimensionFromAxis(const PointT& point, SplitAxis axis) {
        switch (axis) {
            case SplitAxis::X_AXIS:
                return point.x;
            case SplitAxis::Y_AXIS:
                return point.y;
            case SplitAxis::Z_AXIS:
                return point.z;
            default:
                abort();
        }
    }

    void search(PointT target, float distanceTol, std::vector<int> &ids, Node<PointT> *node, SplitAxis axis) {
        if (node == nullptr) {
            return;
        }
        if (closeEnough(target, node->point, distanceTol)) {
            ids.push_back(node->id);
        }
        // Now check which (one or both) sides we want to descend, based on distanceTol*2 in the direction of axis.
        const float extent_min = pointDimensionFromAxis(target, axis) - distanceTol;
        const float extent_max = pointDimensionFromAxis(target, axis) + distanceTol;
        // The tree biases insertions with equal x/y values as the node towards the left child, so remember to do the same during search.
        SplitAxis next_axis;
        switch (axis) {
            case SplitAxis::X_AXIS:
                next_axis = SplitAxis::Y_AXIS;
                break;
            case SplitAxis::Y_AXIS:
                next_axis = SplitAxis::Z_AXIS;
                break;
            case SplitAxis::Z_AXIS:
                next_axis = SplitAxis::X_AXIS;
                break;
        }
        if (extent_min <= pointDimensionFromAxis(node->point, axis)) {
            // Need to descend left.
            search(target, distanceTol, ids, node->left, next_axis);
        }
        if (extent_max >= pointDimensionFromAxis(node->point, axis)) {
            search(target, distanceTol, ids, node->right, next_axis);
        }
    }

    void insert(Node<PointT> *node_ptr, PointT point, int id, SplitAxis axis) {
        assert(node_ptr != nullptr);
        SplitAxis next_axis;
        switch (axis) {
            case SplitAxis::X_AXIS:
                next_axis = SplitAxis::Y_AXIS;
                break;
            case SplitAxis::Y_AXIS:
                next_axis = SplitAxis::Z_AXIS;
                break;
            case SplitAxis::Z_AXIS:
                next_axis = SplitAxis::X_AXIS;
                break;
        }
        if (pointDimensionFromAxis(point, axis) <= pointDimensionFromAxis(node_ptr->point, axis)) {
            insert(&node_ptr->left, std::move(point), id, next_axis);
        } else {
            insert(&node_ptr->right, std::move(point), id, next_axis);
        }
    }

    void insert(Node<PointT> **node_ptr_ref, PointT point, int id, SplitAxis axis) {
        if (*node_ptr_ref == nullptr) {
            *node_ptr_ref = new Node<PointT>(std::move(point), id);
        } else {
            insert(*node_ptr_ref, std::move(point), id, axis);
        }
    }


};

#endif //PLAYBACK_KDTREE3D_H
