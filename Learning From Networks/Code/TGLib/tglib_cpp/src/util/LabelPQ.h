/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file LabelPQ.h
 *  @brief A priority queue implementation
 */

#ifndef TGLIB_LABELPQ_H
#define TGLIB_LABELPQ_H

#include <list>
#include <cassert>
#include <memory>
#include <set>
#include <queue>
#include "../core/OrderedEdgeList.h"


namespace tglib {

/**
 * @brief A struct for label setting distance computations
 */
struct DistanceLabel {

    /**
     * @brief the node id where label is in list
     */
    tglib::NodeId nid = tglib::max_nid;

    /**
     * @brief start time
     */
    tglib::Time s = 0;

    /**
     * @brief arrival time
     */
    tglib::Time a = 0;

    /**
     * @brief distance
     */
    tglib::Time d = 0;

    /**
     * @brief true iff label was deleted
     */
    bool deleted = false;

    /**
     *
     * @param l1
     * @param l2
     * @return
     */
    friend bool operator==(DistanceLabel const& l1, DistanceLabel const& l2) {
        return l1.nid == l2.nid && l1.d == l2.d;
    };

    /**
     *
     * @param l1
     * @param l2
     * @return
     */
    friend bool operator<(DistanceLabel const& l1, DistanceLabel const& l2) {
        return l1.d < l2.d;
    };
};


/**
 * @brief A struct for label setting path computations
 */
struct PathLabel {
    /**
     * @brief the node id where label is in list
     */
    tglib::NodeId nid = tglib::max_nid;

    /**
     * @brief start time
     */
    tglib::Time s = 0;

    /**
     * @brief arrival time
     */
    tglib::Time a = 0;

    /**
     * @brief distance
     */
    tglib::Time d = 0;

    /**
     * @brief parent label
     */
    std::shared_ptr<PathLabel> parent;

    /**
     * @brief edge to parent
     */
    tglib::TemporalEdge e;

    /**
     * @brief true iff label was deleted
     */
    bool deleted = false;

    /**
     *
     * @param l1
     * @param l2
     * @return
     */
    friend bool operator==(PathLabel const& l1, PathLabel const& l2) {
        return l1.nid == l2.nid && l1.d == l2.d && l1.parent == l2.parent && l1.e == l2.e;
    };

    /**
     *
     * @param l1
     * @param l2
     * @return
     */
    friend bool operator<(PathLabel const& l1, PathLabel const& l2) {
        return l1.d < l2.d;
    };
};


/**
 * @brief A priority queue implementation
 */
template<typename L>
class PQ {

public:

    /**
     * @brief Pushes new element into PQ
     * @param t The element
     */
    void push(const std::shared_ptr<L> &t) {
        data.push_back(t);
        heapifyUp(data.size() - 1);
    }

    /**
     * @return Returns top label with smallest distance
     */
    std::shared_ptr<L>& top() {
        return data[0];
    }

    /**
     *
     * @return True if empty
     */
    bool empty() {
        return data.empty();
    }

    /**
     *
     * @return Number of labels in PQ
     */
    size_t size() {
        return data.size();
    }

    /**
     * @brief Removes label with smallest distance from PQ
     */
    void pop() {
        data[0] = data.back();
        data.pop_back();
        heapifyDown();
    }

private:

    /**
     * @brief holds the labels
     */
    std::vector<std::shared_ptr<L>> data;

    /**
     * Heapify up
     * @param p
     */
    void heapifyUp(size_t p) {
        size_t child = p;
        size_t parent = getParent(child);

        while (*(data[child]) < *(data[parent])) {
            swap(child, parent);
            child = parent;
            parent = getParent(child);
        }
    }

    /**
     * Heapify down
     * @param p
     */
    void heapifyDown(size_t p = 0) {
        size_t length = data.size();
        while (true) {
            auto left = getLeftChild(p);
            auto right = getRightChild(p);
            auto smallest = p;

            if (left < length && *(data[left]) < *(data[smallest]))
                smallest = left;

            if (right < length && *(data[right]) < *(data[smallest]))
                smallest = right;

            if (smallest != p) {
                swap(smallest, p);
                p = smallest;
            } else break;
        }
    }

    /**
     * Finds parent
     * @param child
     * @return
     */
    static size_t getParent(size_t child) {
        if (child == 0) return 0;
        if (child % 2 == 0)
            return (child / 2) - 1;
        else
            return child / 2;
    }

    /**
     * Get left child
     * @param parent
     * @return
     */
    static size_t getLeftChild(size_t parent){
        return 2 * parent + 1;
    }

    /**
     * Get right child
     * @param parent
     * @return
     */
    static size_t getRightChild(size_t parent){
        return 2 * parent + 2;
    }

    /**
     * swap
     * @param a
     * @param b
     */
    void swap(size_t a, size_t b) {
        data[a].swap(data[b]);
    }

};

} // tglib

#endif //TGLIB_LABELPQ_H
