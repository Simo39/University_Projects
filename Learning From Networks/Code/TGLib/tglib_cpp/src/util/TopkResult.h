/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TopkResult.h
 *  @brief Contains a class for handling top-k sets
 */

#ifndef TGLIB_TOPKRESULT_H
#define TGLIB_TOPKRESULT_H

#include <set>
#include <memory>
#include "../core/OrderedEdgeList.h"


namespace tglib {

/**
 * @brief Struct containing comparator function
 */
struct TopkValueComperator
{
    /**
     * Comperator
     * @tparam T
     * @param l
     * @param r
     * @return
     */
    template<typename T>
    bool operator()(const T& l, const T& r) const {
        return l.value > r.value;
    }
};

/**
 * @brief A struct for managing top-k results.
 */
template<typename T>
struct TopkEntry {

    /**
     * @brief Constructor
     * @param nid Node id
     * @param closeness value
     */
    TopkEntry(tglib::NodeId nid, T closeness) : value(closeness) {
        nids.push_back(nid);
    };

    /**
     * @brief The stored value
     */
    T value = 0;

    /**
     * @brief all nodes with value
     */
    std::vector<tglib::NodeId> nids;
};

/**
 * @brief Helper class for managing top-k results.
 * @tparam T Numerical result type
 */
template<typename T>
class TopkResult {

public:

    /**
     * Default construtor
     */
    TopkResult() = default;

    /**
     *
     * @param k
     */
    explicit TopkResult(const unsigned int k) :
            k(k), maxClosenessBound(0) {
    };

    /**
     *
     * @return the smallest of the top-k values
     */
    [[nodiscard]] double getMinMaxTopk() const {
        return maxClosenessBound;
    }

    /**
     * True if already k elements found
     * @return
     */
    [[nodiscard]] bool doApproximation() const {
        return topk.size() >= k;
    }

    /**
     * @return Size of elements
     */
    [[nodiscard]] size_t size() const {
        return topk.size();
    }

    /**
     *
     * @return The results
     */
    std::vector<std::pair<tglib::NodeId, T>> getResults() {
        std::vector<std::pair<NodeId, T>> results;
        for (auto &p : topk) {
            for (auto &n : p.nids) {
                results.emplace_back(n, p.value);
            }
        }
        return results;
    }

    /**
     * @brief Try adding a new element
     *
     * The element will be only stored if its value is larger or equal to the smallest of
     * the top-k elements found so far, or if not yet k elements added.
     *
     * @param nid The (node) id
     * @param value The value
     */
    void insert(tglib::NodeId nid, T value) {
        if (doApproximation() && maxClosenessBound > value)
            return;

        auto p = topk.insert(TopkEntry(nid, value));                 //fixed error: "missing template arguments before ‘(’ token" in build phase with <T>     
        if (!p.second) {
            const_cast<std::vector<NodeId>&>(p.first->nids).push_back(nid);
        }

        if (maxClosenessBound < value && topk.size() > k) {
            auto i = topk.find(TopkEntry(0, maxClosenessBound));     //fixed error: "missing template arguments before ‘(’ token" in build phase with <T>
            if (i != topk.end()) {
                topk.erase(i);
            }
        }
        maxClosenessBound = topk.rbegin()->value;
    }

private:

    /**
     * the k
     */
    unsigned int k{};

    /**
     * the bound
     */
    T maxClosenessBound{};

    /**
     * the stored values
     */
    std::set<TopkEntry<T>, TopkValueComperator> topk;

};

} // tglib

#endif //TGLIB_TOPKRESULT_H
