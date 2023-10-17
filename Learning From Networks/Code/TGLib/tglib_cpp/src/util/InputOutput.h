/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file InputOutput.h
 *  @brief Contains IO functionality
 */

#ifndef TGLIB_INPUTOUTPUT_H
#define TGLIB_INPUTOUTPUT_H


#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include "../core/OrderedEdgeList.h"
#include "../core/IncidentLists.h"
#include "../core/TRSGraph.h"
#include "../core/Transformations.h"


namespace tglib {

/**
 * @brief Mananges node ids.
 */
struct NodeIdManager {
    /**
     * @brief Constructor
     */
    NodeIdManager() : nid(0){};

    /**
     * @brief mapping between node ids
     */
    std::unordered_map<NodeId, NodeId> nidmap;

    /**
     *
     * @param id Node id from input graph
     * @return Node id if id is already assigned or new node id
     */
    NodeId getNodeId(NodeId id) {
        NodeId r;
        if (nidmap.find(id) == nidmap.end()) {
            r = nid;
            nidmap.emplace(id, nid++);
        } else {
            r = nidmap.at(id);
        }
        return r;
    }

    /**
     * @brief stores the number of nodes / used node ids
     */
    NodeId nid = 0;
};

/**
 * @brief Helper function for splitting lines
 * @param s the line
 * @param delim the delimiter
 * @return vector of ints
 */
inline std::vector<Time> split_string(const std::string& s, char delim) {
    std::vector<Time> result;
    std::stringstream ss(s);
    while (ss.good()) {
        std::string substr;
        getline(ss, substr, delim);
        if (!substr.empty())
            result.push_back(stoi(substr));
    }
    return result;
}

/**
 * @brief Loads a temporal graph in ordered edge list representation from a file
 * @param filename The file in which each line represents one temporal edge (u,v,t,tt)
 * @param directed If false, each edge will be inserted in both directions
 * @return The ordered edge stream
 */
template<typename E>
tglib::OrderedEdgeList<E> load_ordered_edge_list(const std::string &filename, bool directed) {
    std::ifstream fs;
    fs.open(filename);
    auto opened_file = fs.is_open();

    if (!opened_file) {
        throw std::runtime_error("Could not open file " + filename);
    }

    std::string line;

    std::vector<E> edges;
    TimeInterval ti{inf, 0};

    NodeIdManager nidman;
    while (getline(fs, line)) {
        if (line.empty()) continue;
        std::vector<Time> l = split_string(line, ' ');

        if (l.size() < 3) continue;

        NodeId u = nidman.getNodeId(l[0]);
        NodeId v = nidman.getNodeId(l[1]);

        Time t = l[2];

        Time tt = 1; // todo optional argument
        if (l.size() >= 4)
            tt = l[3];

        edges.push_back({u, v, t, tt});
        if (!directed) {
            edges.push_back({v, u, t, tt});
        }

        if (t < ti.first) {
            ti.first = t;
        }

        if (t + tt > ti.second) {
            ti.second = t + tt;
        }
    }
    fs.close();
    sort(edges.begin(), edges.end());

    OrderedEdgeList<E> tgs(nidman.nid, edges, ti, nidman.nidmap);

    return tgs;
}

/**
 * @brief Loads a directed temporal graph in ordered edge list representation from a file
 * @param filename The file in which each line represents one temporal edge (u,v,t,tt)
 * @return The ordered edge stream
 */
template<typename E>
tglib::OrderedEdgeList<E> load_ordered_edge_list(const std::string &filename) {
    return load_ordered_edge_list<E>(filename, true);
}

/**
 * @brief Loads a temporal graph in incident lists representation from a file
 * @param filename The file in which each line represents one temporal edge (u,v,t,tt)
 * @param directed If false, each edge will be inserted in both directions
 * @return The temporal graph in incidents list representation
 */
template<typename E, class T>
tglib::IncidentLists<T, E> load_incident_lists(const std::string &filename, bool directed) {
    OrderedEdgeList<E> tgs = load_ordered_edge_list<E>(filename, directed);
    return to_incident_lists<T, E>(tgs); // Possible Error from :  to_incident_lists(tgs); to = to_incident_lists<T, E>(tgs);
}

/**
 * @brief Loads a temporal graph in time-respecting static graph representation from a file
 * @param filename The file in which each line represents one temporal edge (u,v,t,tt)
 * @param directed If false, each edge will be inserted in both directions
 * @return The temporal graph in time-respecting static graph representation
 */
inline tglib::TRSGraph load_TRS(const std::string &filename, bool directed) {
    auto tgs = load_ordered_edge_list<TemporalEdge>(filename, directed);
    return to_trs_graph<TemporalEdge>(tgs);
}

} // tglib

#endif //TGLIB_INPUTOUTPUT_H
