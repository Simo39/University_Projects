/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file BasicTypes.h
 *  @brief Contains all basic types.
 */

#ifndef TGLIB_BASICTYPES_H
#define TGLIB_BASICTYPES_H

#include <cstdint>
#include <limits>
#include <utility>
#include <string>
#include <tuple>


namespace tglib {

/**
 * @brief Node id type
 */
using NodeId = int32_t;

/**
 * @brief Edge id type
 */
using EdgeId = int64_t;

/**
 * @brief Time type
 */
using Time = int64_t;

/**
 * @brief Distance type
 */
using Distance = int64_t;

/**
 * @brief The type for time intervals. Used for restricting the
 * algorithms to a specific time interval.
 */
using TimeInterval = std::pair<Time, Time>;

/**
 * @brief max time
 */
auto const inf = std::numeric_limits<Time>::max();

/**
 * @brief max node id
 */
auto const max_nid = std::numeric_limits<NodeId>::max();

/**
 * @brief max edge id
 */
auto const max_eid = std::numeric_limits<EdgeId>::max();

/**
 * @brief Distance type is used for defining the temporal distance of optimal temporal paths
 * in centrality measures like the temporal closeness centrality.
 */
enum class Distance_Type {
    Earliest_Arrival,
    Fastest,
    Latest_Departure,
    Minimum_Transition_Times,
    Minimum_Hops
};


/**
 * @brief The basic struct for representing temporal edges
 */
struct TemporalEdge {
    /**
     * @brief The tail
     */
    NodeId u;
    /**
     * @brief The head
     */
    NodeId v;
    /**
     * @brief The availability time
     */
    Time t;
    /**
     * @brief The transition time
     */
    Time tt;

};

/**
 * Get string representation of temporal edge
 * @param e The temporal edge
 * @return String representation
 */
inline std::string to_string(const TemporalEdge& e) {
    return "(" + std::to_string(e.u) + " " + std::to_string(e.v) + " "
            + std::to_string(e.t) + " " + std::to_string(e.tt) + ")";
}

/**
 * @brief << operator for TemporalEdge
 * @param stream
 * @param e
 * @return
 */
inline std::ostream& operator<<(std::ostream& stream, const TemporalEdge& e) {
    stream << to_string(e);
//    stream << "(" + std::to_string(e.u) + " ";
//    stream << std::to_string(e.v) + " ";
//    stream << std::to_string(e.t) + " ";
//    stream << std::to_string(e.tt) + ")";
    return stream;
}

/**
 * @brief < operator for TemporalEdge
 * @param e1
 * @param e2
 * @return
 */
inline bool operator<(const TemporalEdge &e1, const TemporalEdge &e2) {
    //todo fix
    return std::tie(e1.t, e1.tt, e1.u, e1.v) < std::tie(e2.t, e2.tt, e2.u, e2.v);
}

/**
 * @brief > operator for TemporalEdge
 * @param e1
 * @param e2
 * @return
 */
inline bool operator>(const TemporalEdge &e1, const TemporalEdge &e2){
    return (e2 < e1);
}

/**
 * @brief == operator for TemporalEdge
 * @param e1
 * @param e2
 * @return
 */
inline bool operator==(const TemporalEdge &e1, const TemporalEdge &e2){
    return !(e1 < e2 || e2 < e1);
}

/**
 * @brief != operator for TemporalEdge
 * @param e1
 * @param e2
 * @return
 */
inline bool operator!=(const TemporalEdge &e1, const TemporalEdge &e2){
    return !(e1 == e2);
}

/**
 * @brief Struct for temporal graph statistics.
 */
struct TemporalGraphStatistics {
    /**
     * @brief The number of nodes
     */
    size_t numberOfNodes;

    /**
     * @brief The number of edges
     */
    size_t numberOfEdges;

    /**
     * @brief The number of static edges in the aggregated underlying graph
     */
    size_t numberOfStaticEdges;

    /**
     * @brief The number of different time stamps
     */
    size_t numberOfTimeStamps;

    /**
     * @brief The number of different transition times
     */
    size_t numberOfTransitionTimes;

    /**
     * @brief The minimal time stamp
     */
    Time minimalTimeStamp;

    /**
     * @brief The maximal time stamp
     */
    Time maximalTimeStamp;

    /**
     * @brief The minimal transition time
     */
    Time minimalTransitionTime;

    /**
     * @brief The maximal transition time
     */
    Time maximalTransitionTime;

    /**
     * @brief The maximal temporal indegree
     */
    long maxTemporalInDegree;

    /**
    * @brief The minimal temporal indegree
    */
    long minTemporalInDegree;

    /**
     * @brief The maximal temporal outdegree
     */
    long maxTemporalOutDegree;

    /**
     * @brief The minimal temporal outdegree
     */
    long minTemporalOutDegree;

    /**
     * @brief
     * @return The string representation of the statistics
     */
    [[nodiscard]] std::string toString() const {
        std::string s;
        s += "number of nodes: " + std::to_string(numberOfNodes) + "\n";
        s += "number of edges: " + std::to_string(numberOfEdges) + "\n";
        s += "number of static edges: " + std::to_string(numberOfStaticEdges) + "\n";
        s += "number of time stamps: " + std::to_string(numberOfTimeStamps) + "\n";
        s += "number of transition times: " + std::to_string(numberOfTransitionTimes) + "\n";
        s += "min. time stamp: " + std::to_string(minimalTimeStamp) + "\n";
        s += "max. time stamp: " + std::to_string(maximalTimeStamp) + "\n";
        s += "min. transition time: " + std::to_string(minimalTransitionTime) + "\n";
        s += "max. transition time: " + std::to_string(maximalTransitionTime) + "\n";
        s += "min. temporal in-degree: " + std::to_string(minTemporalInDegree) + "\n";
        s += "max. temporal in-degree: " + std::to_string(maxTemporalInDegree) + "\n";
        s += "min. temporal out-degree: " + std::to_string(minTemporalOutDegree) + "\n";
        s += "max. temporal out-degree: " + std::to_string(maxTemporalOutDegree);
        return s;
    }
};


} // tglib

#endif //TGLIB_BASICTYPES_H
