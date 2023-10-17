// Amalgamated header of TGLib

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
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file OrderedEdgeList.h
 *  @brief Contains the ordered edge list representation data structure for temporal graphs
 */

#ifndef CPP_TEMPORALGRAPHSTREAM_H
#define CPP_TEMPORALGRAPHSTREAM_H

#include <limits>
#include <utility>
#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include "BasicTypes.h"


namespace tglib {

/**
 * @brief The ordered edge list contains all temporal edges in chronological order
 */
template<typename E>
class OrderedEdgeList {

public:

    /**
     * @brief Default constructor
     */
    OrderedEdgeList() = default;

    /**
     * @brief Constructor
     * @param num_nodes_ The number of nodes
     * @param edges_ The temporal edges
     * @param ti_ The time interval
     */
    OrderedEdgeList(size_t num_nodes_, std::vector<E> edges_, TimeInterval ti_) :
        num_nodes(num_nodes_), edges(edges_), ti(std::move(ti_)){};

    /**
     * @brief Constructor
     * @param num_nodes_ The number of nodes
     * @param edges_ The temporal edges
     * @param ti_ The time interval
     * @param node_mapping_ The node mapping
     */
    OrderedEdgeList(size_t num_nodes_, std::vector<E> edges_, TimeInterval ti_,
                        std::unordered_map<NodeId, NodeId> node_mapping_) :
        num_nodes(num_nodes_), edges(edges_), ti(std::move(ti_)){
        node_map = std::move(node_mapping_);
        reverse_node_map.resize(node_map.size(), 0);
        for (auto p : node_map) {
            reverse_node_map[p.second] = p.first;
        }
    };

    /**
     * @brief Getter for number of nodes
     * @return The number of nodes.
     */
    [[nodiscard]] size_t getNumberOfNodes() const {
        return num_nodes;
    }

     /**
     * @brief Getter for number of edges
     * @return The number of edges.
     */
    [[nodiscard]] size_t getNumberOfEdges() const {
        return edges.size();
    }

    /**
     * @brief Getter for edges
     * @return Const reference to edges.
     */
    const std::vector<E> &getEdges() const {
        return edges;
    }

    /**
     * @brief Getter for number of the time interval spanned by the temporal graph
     * @return The time interval.
     */
    [[nodiscard]] TimeInterval getTimeInterval() const {
        return ti;
    }

    /**
      * @brief Getter for mapping from new ids to the original ids used in input file
      * @return The mapping
      */
    const std::vector<NodeId> &getReverseNodeMap() const {
        return reverse_node_map;
    }

    /**
      * @brief Getter for mapping from new ids to the original ids used in input file
      * @return The mapping
      */
    const std::unordered_map<NodeId, NodeId> &getNodeMap() const {
        return node_map;
    }



private:

    /**
     * @brief The number of nodes.
     */
    size_t num_nodes{};

    /**
     * @brief The chronological ordered temporal edges. Ties are broken arbitrarily.
     */
    std::vector<E> edges;

    /**
     * @brief The time interval spanned by the temporal graph
     */
     TimeInterval ti;

     /**
      * @brief The mapping from new ids to the original ids used in input file
      */
     std::vector<NodeId> reverse_node_map;

     /**
       * @brief The mapping from the original ids used in input file to the new node ids in [0, num_nodes-1]
       */
     std::unordered_map<NodeId, NodeId> node_map;

};

/**
 * @brief == operator OrderedEdgeList<E>
 * @tparam E
 * @param e1
 * @param e2
 * @return
 */
template<typename E>
inline bool operator==(const OrderedEdgeList<E> &e1, const OrderedEdgeList<E> &e2){
    return e1.getNumberOfNodes() == e2.getNumberOfNodes() &&
        e1.getEdges() == e2.getEdges() &&
        e1.getTimeInterval() == e2.getTimeInterval();
}

/**
 * @brief != operator OrderedEdgeList<E>
 * @tparam E
 * @param e1
 * @param e2
 * @return
 */
template<typename E>
inline bool operator!=(const OrderedEdgeList<E> &e1, const OrderedEdgeList<E> &e2){
    return !(e1 == e2);
}


// todo add edge, remove edge
/**
 * @brief Computes the basic statistics of a temporal graph
 * @tparam E The edge type
 * @param tgs The temporal graph
 * @return The basic statistics
 */
template<typename E>
TemporalGraphStatistics get_statistics(OrderedEdgeList<E> const &tgs) {
    TemporalGraphStatistics statistics{};
    statistics.minTemporalInDegree = inf;
    statistics.minTemporalOutDegree = inf;
    statistics.maxTemporalInDegree = 0;
    statistics.maxTemporalOutDegree = 0;
    statistics.maximalTimeStamp = 0;
    statistics.minimalTimeStamp = inf;
    statistics.maximalTransitionTime = 0;
    statistics.minimalTransitionTime = inf;

    std::vector<long> inDegree(tgs.getNumberOfNodes(), 0);
    std::vector<long> outDegree(tgs.getNumberOfNodes(), 0);
    std::unordered_set<Time> times;
    std::unordered_set<Time> transition_times;
    std::set<std::pair<NodeId, NodeId>> static_edges;

    for (auto &e : tgs.getEdges()){
        inDegree[e.v]++;
        outDegree[e.u]++;
        times.insert(e.t);
        transition_times.insert(e.tt);
        static_edges.insert({e.u, e.v});

        if (statistics.maximalTimeStamp < e.t) statistics.maximalTimeStamp = e.t;
        if (statistics.minimalTimeStamp > e.t) statistics.minimalTimeStamp = e.t;
        if (statistics.maximalTransitionTime < e.tt) statistics.maximalTransitionTime = e.tt;
        if (statistics.minimalTransitionTime > e.tt) statistics.minimalTransitionTime = e.tt;
    }

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        if (statistics.minTemporalInDegree > inDegree[nid]) statistics.minTemporalInDegree = inDegree[nid];
        if (statistics.minTemporalOutDegree > outDegree[nid]) statistics.minTemporalOutDegree = outDegree[nid];
        if (statistics.maxTemporalInDegree < inDegree[nid])  statistics.maxTemporalInDegree = inDegree[nid];
        if (statistics.maxTemporalOutDegree < outDegree[nid])  statistics.maxTemporalOutDegree = outDegree[nid];
    }

    statistics.numberOfNodes = tgs.getNumberOfNodes();
    statistics.numberOfEdges = tgs.getEdges().size();
    statistics.numberOfStaticEdges = static_edges.size();
    statistics.numberOfTimeStamps = times.size();
    statistics.numberOfTransitionTimes = transition_times.size();

    return statistics;
}

} // tglib

#endif //CPP_TEMPORALGRAPHSTREAM_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file AggregatedGraph.h
 *  @brief Contains types for the aggregated graph representation.
 */

#ifndef TGLIB_AGGREGATEDGRAPH_H
#define TGLIB_AGGREGATEDGRAPH_H

#include "BasicTypes.h"

namespace tglib {

/**
 * @brief A simple weigted (non-temporal) static edge.
 */
struct StaticWeightedEdge {
    /**
     * @brief The tail
     */
    NodeId u;

    /**
     * @brief The head
     */
    NodeId v;

    /**
     * @brief The weight
     */
    int64_t weight;
};

} // tglib

#endif //TGLIB_AGGREGATEDGRAPH_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file IncidentLists.h
 *  @brief Contains the incident lists representation data structure for temporal graphs
 */

#ifndef TGLIB_INCIDENTLISTS_H
#define TGLIB_INCIDENTLISTS_H


#include "OrderedEdgeList.h"


namespace tglib {

/**
 * @brief Node type for incident list representation
 *
 * This is the full generic type that allows using customized edge types, e.g., weighted temporal edges.
 *
 * @tparam E the temporal edge type
 */
template<typename E>
struct TGNodeT {
    /**
     * @brief The node id.
     */
    NodeId id{};
    /**
     * @brief The outgoing temporal edges.
     */
    std::vector<E> outEdges;
};

/**
 * @brief Node type for incident list representation
 *
 * The default type used in the implemented algorithms.
 */
using TGNode = TGNodeT<TemporalEdge>;

/**
 * @brief Type for incident list representation of temporal graphs
 *
 * The incident lists representation holds a vector of its nodes.
 * Each node has a vector of the outgoing temporal edges.
 */
template<typename N, typename E>
class IncidentLists {

public:

    /**
     * @ default constructor
     */
    IncidentLists() = default;

    /**
     * @brief Constructs object from vector of edges
     * @param numberOfNodes The number of nodes
     * @param edges The edges
     */
    explicit IncidentLists(NodeId numberOfNodes, std::vector<E> const &edges) {
        addNodes(numberOfNodes);
        ti = {inf, 0};
        for (auto &e : edges) {
            addEdge(e);
            if (e.t < ti.first) ti.first = e.t;
            if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
        }
    }

    /**
     * @brief Getter for number of nodes
     * @return The number of nodes
     */
    [[nodiscard]] size_t getNumberOfNodes() const {
        return nodes.size();
    }

    /**
     * @brief Getter for number of edges
     * @return The number of edges
     */
    [[nodiscard]] size_t getNumberOfEdges() const {
        return num_edges;
    }

    /**
     * @brief Getter for the time interval
     * @return The time interval
     */
    [[nodiscard]] TimeInterval getTimeInterval() const {
        return ti;
    }

    /**
     * @brief Getter for a node
     * @param nid the nodes id
     * @return The node with node id nid
     */
    const N& getNode(size_t nid) const {
        return nodes[nid];
    }

    /**
     * @brief Getter for all nodes
     * @return The nodes
     */
    const std::vector<N> &getNodes() const {
        return nodes;
    }


private:

    /**
     * @brief The nodes
     */
    std::vector<N> nodes;

    /**
     * @brief The number of edges
     */
    EdgeId num_edges{};

    /**
     * @brief The time interval spanned by the temporal graph
     */
    TimeInterval ti;

    /**
    * @brief Adds num nodes with ids 0 to num-1
    * @param num the number of nodes to add
    */
    void addNodes(size_t num) {
        for (size_t i = 0; i < num; ++i) {
            N node;
            node.id = i;
            nodes.push_back(node);
        }
    }

    /**
     * Adds a new edge
     * @param e The edge to add
     */
    void addEdge(E e) {
        nodes[e.u].outEdges.push_back(e);
        num_edges++;
    }

};

} // tglib

// todo add edge, remove edge


#endif //TGLIB_INCIDENTLISTS_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TRSGraph.h
 *  @brief Contains the time-respecting static graph representation type.
 */


#ifndef TGLIB_TRSGRAPH_H
#define TGLIB_TRSGRAPH_H

#include <vector>
#include "BasicTypes.h"


namespace tglib {

/**
 * @brief A struct for edges in the time-respecting static graph representation.
 */
struct TRSEdge {
    /**
     * @brief The tail.
     */
    NodeId u;

    /**
     * @brief The head.
     */
    NodeId v;

    /**
     * @brief The traversal time
     */
    Time tt;
};

/**
 * @brief Time-nodes representing a node in the temporal graph at a specific time.
 */
using TimeNode = std::pair<NodeId, Time>;

/**
 * @brief A struct for nodes in the time-respecting static graph representation.
 */
struct TRSNode {
    /**
     * @brief The node id.
     */
    NodeId nid;

    /**
     * @brief The corresponding time-node.
     */
    TimeNode tn;

    /**
     * @brief The incident edges.
     */
    std::vector<TRSEdge> adjList;
};

/**
 * @brief The time-respecing static graph is a DAG that can be used for temporal distance computations.
 */
struct TRSGraph {
    /**
     * @brief The nodes.
     */
    std::vector<TRSNode> nodes;

    /**
     * @brief positionOfEarliestTimeNode[i] is position of the earliest node in nodes such that the
     * corresponding vertex in the temporal graph is i.
     */
    std::vector<long> positionOfEarliestTimeNode;

    /**
     * @brief positionOfLatestTimeNode[i] is position of the latest node in nodes such that the
     * corresponding vertex in the temporal graph is i.
     */
    std::vector<long> positionOfLatestTimeNode;

    /**
     * @brief The number of edges.
     */
    long numEdges;

    /**
     * @brief The number of nodes in the corresponding temporal graph.
     */
    long numTGNodes;

    /**
     * @brief The time interval spanned by the temporal graph
     */
    TimeInterval ti;
};

} // tglib

#endif //TGLIB_TRSGRAPH_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file DirectedLineGraph.h
 *  @brief Contains the DirectedLineGraph representation data structure for temporal graphs
 */

#ifndef TGLIB_DIRECTEDLINEGRAPH_H
#define TGLIB_DIRECTEDLINEGRAPH_H

#include <vector>
#include "BasicTypes.h"

namespace tglib {

/**
 * @brief Edge type for directed line graph.
 *
 * An edge in a directed line graph represents a temporal path of length two
 * in the temporal graph, i.e., ((tg_u, tg_v, u_t), (tg_v, tg_w, v_t))
 */
struct DLEdge {
    /**
     * @brief Id of the tail in the dlg
     */
    NodeId u;

    /**
     * @brief Id of the head in the dlg
     */
    NodeId v;

    /**
     * @brief Id of the tail of the temporal first edge
     */
    NodeId tg_u;

    /**
     * @brief Id of the head of the temporal first edge
     */
    NodeId tg_v;

    /**
     * @brief Id of the head of the temporal second edge
     */
    NodeId tg_w;

    /**
     * @brief Availability time of the first edge
     */
    Time u_t;

    /**
     * @brief Availability time of the second edge
     */
    Time v_t;
};

/**
 * @brief Node type for directed line graph
 */
struct DLNode {
    /**
     * The
     */
    NodeId id;

    /**
     * @brief The corresponding temporal edge
     */
    TemporalEdge e;

    /**
     * @brief The incident edges
     */
    std::vector<DLEdge> outedges;
};

/**
 * @brief The directed line graph data structure
 */
struct DirectedLineGraph {
    /**
     * @brief Number of nodes in the corresponding temporal graph
     */
    unsigned long num_tg_nodes{};

    /**
     * @brief Number of edges in the DLG
     */
    unsigned long num_edges{};

    /**
     * @brief The nodes of the DLG
     */
    std::vector<DLNode> nodes;
};


} // tglib


#endif //TGLIB_DIRECTEDLINEGRAPH_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file AggregatedGraph.h
 *  @brief Contains types for the aggregated graph representation.
 */

#ifndef TGLIB_AGGREGATEDGRAPH_H
#define TGLIB_AGGREGATEDGRAPH_H

#include "BasicTypes.h"

namespace tglib {

/**
 * @brief A simple weigted (non-temporal) static edge.
 */
struct StaticWeightedEdge {
    /**
     * @brief The tail
     */
    NodeId u;

    /**
     * @brief The head
     */
    NodeId v;

    /**
     * @brief The weight
     */
    int64_t weight;
};

} // tglib

#endif //TGLIB_AGGREGATEDGRAPH_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file Transformations.h
 *  @brief Contains functions for transforming temporal graph between different representations
 */

#ifndef TGLIB_TRANSFORMATIONS_H
#define TGLIB_TRANSFORMATIONS_H

#include <map>
#include <algorithm>
#include "OrderedEdgeList.h"
#include "IncidentLists.h"
#include "TRSGraph.h"
#include "DirectedLineGraph.h"
#include "AggregatedGraph.h"
#include <cassert>
#include <cmath>


namespace tglib {

/**
 * @brief A function for transforming ordered edge list representation to incident lists representation.
 *
 * @tparam N The node type of the IncidentLists representation
 * @tparam E The temporal edge type of the OrderedEdgeList
 * @param tgs A temporal graph in ordered edge list representation.
 * @return The temporal graph in incident lists representation.
 */
template<typename N, typename E>
tglib::IncidentLists<N, E> to_incident_lists(tglib::OrderedEdgeList<E> const &tgs) {
    IncidentLists<N, E> g(tgs.getNumberOfNodes(), tgs.getEdges());
    return g;
}

/**
 * A function for transforming incident lists representation to ordered edge list representation.
 * @param tg A temporal graph in incident lists representation.
 * @return The temporal graph in ordered edge list representation.
 */
template<typename N, typename E>
tglib::OrderedEdgeList<E> to_ordered_edge_list(tglib::IncidentLists<N, E> const &tg) {
    // todo nodemapping
    OrderedEdgeList<E> tgs;

    for (auto &n : tg.nodes) {
        for (auto &e : n.outEdges) {
            tgs.edges.push_back(e);
        }
    }

    sort(tgs.edges.begin(), tgs.edges.end(),
              [](auto const &e1, auto const &e2){return e1.t < e2.t;});

    tgs.num_nodes = tg.nodes.size();
    tgs.ti = tg.ti;

    return tgs;
}

/**
 * A function for transforming ordered edge list representation to time-respecting static graph.
 * @param tgs A temporal graph in ordered edge list representation.
 * @return The temporal graph in incident lists representation.
 */
template<typename E>
tglib::TRSGraph to_trs_graph(tglib::OrderedEdgeList<E> const &tgs) {
    TRSGraph trs;
    trs.positionOfEarliestTimeNode.resize(tgs.getNumberOfNodes(), inf);
    trs.positionOfLatestTimeNode.resize(tgs.getNumberOfNodes(), inf);
    trs.numTGNodes = tgs.getNumberOfNodes();
    trs.numEdges = 0;

    std::set<TimeNode> timeNodes;
    std::vector<Time> maxArrivalTimes(tgs.getNumberOfNodes(), -1);
    std::vector<std::set<Time>> startingTimes(tgs.getNumberOfNodes(), std::set<Time>());

    std::map<TimeNode, NodeId> nodeMap;

    NodeId nid = 0;

    for (auto &e : tgs.getEdges()) {
        startingTimes[e.u].insert(e.t);
        if (maxArrivalTimes[e.v] < e.t + e.tt) {
            maxArrivalTimes[e.v] = e.t + e.tt;
        }
    }
    for (size_t i = 0; i < tgs.getNumberOfNodes(); ++i) {
        if (maxArrivalTimes[i] > -1) {
            timeNodes.insert({i, maxArrivalTimes[i]});
            startingTimes[i].insert(maxArrivalTimes[i]);
        }

        auto numberOfStartingTimes = startingTimes[i].size();
        for (Time startingTime : startingTimes[i]){
            TRSNode trsNode;
            trsNode.tn = {i, startingTime};
            trsNode.nid = nid++;

            timeNodes.insert(trsNode.tn);
            nodeMap[trsNode.tn] = trsNode.nid;

            if (numberOfStartingTimes > 1) {
                TRSEdge trsEdge{};
                trsEdge.u = trsNode.nid;
                trsEdge.v = nid;
                trsEdge.tt = 0;
                trsNode.adjList.push_back(trsEdge);
                trs.numEdges++;
            }
            trs.nodes.push_back(trsNode);

            if (numberOfStartingTimes == startingTimes[i].size()) {
                trs.positionOfEarliestTimeNode[i] = trs.nodes.size()-1;
            }

            numberOfStartingTimes--;

            if (numberOfStartingTimes == 0) {
                trs.positionOfLatestTimeNode[i] = trs.nodes.size()-1;
            }

        }
    }
    for (auto &e : tgs.getEdges()) {
        NodeId u = nodeMap[{e.u, e.t}];
        auto it = lower_bound(startingTimes[e.v].begin(), startingTimes[e.v].end(), e.t + e.tt);
        assert(it != startingTimes[e.v].end());
        Time arrTime = *(it);
        NodeId v = nodeMap[{e.v, arrTime}];
        TRSEdge trsEdge{u, v, e.tt};
        trs.nodes[u].adjList.push_back(trsEdge);
        trs.numEdges++;
    }

    trs.ti = tgs.getTimeInterval();

    return trs;
}

/**
 * @brief Transformation to directed line graph.
 * @tparam N The node type of the input graph
 * @param tg The input graph
 * @return The corresponding directed line graph
 */
template<typename N, typename E>
tglib::DirectedLineGraph to_directed_line_graph(tglib::IncidentLists<N, E> const &tg) {
    DirectedLineGraph dlg;

    dlg.nodes.clear();
    dlg.nodes.resize(tg.getNumberOfEdges());
    std::vector<bool> nodeset(tg.getNumberOfEdges(), false);

    std::map<TemporalEdge, EdgeId> eids;
    EdgeId eid = 0;
    for (auto &tgn : tg.getNodes()) {
        for (auto &e: tgn.outEdges) {
            eids[e] = eid++;
        }
    }

    for (auto &tgn : tg.getNodes()) {
        for (auto &e : tgn.outEdges) {
            if (!nodeset.at(eids[e])) {
                DLNode node;
                node.id = eids[e];
                node.e = e;
                dlg.nodes.at(node.id) = node;
                nodeset.at(eids[e]) = true;
            }
            for (auto &f : tg.getNode(e.v).outEdges) {
                if (e.t + e.tt > f.t) continue;
                if (!nodeset.at(eids[f])) {
                    DLNode node;
                    node.id = eids[f];
                    node.e = f;
                    dlg.nodes.at(node.id) = node;
                    nodeset.at(eids[f]) = true;
                }
                auto &m = dlg.nodes.at(eids[f]);
                auto &n = dlg.nodes.at(eids[e]);
                auto de = DLEdge();
                de.u = n.id;
                de.v = m.id;
                de.u_t = n.e.t;
                de.v_t = m.e.t;
                de.tg_u = n.e.u;
                de.tg_w = m.e.v;
                de.tg_v = n.e.v;
                n.outedges.push_back(de);
//                m.inedges.push_back(de);
                dlg.num_edges++;
            }
        }
    }
    dlg.num_tg_nodes = tg.getNumberOfNodes();

    return dlg;
}

/**
 * @brief Transformation to directed line graph.
 * @tparam E The edge type of the input graph
 * @param tgs The input graph
 * @return The corresponding directed line graph
 */
template<class E>
tglib::DirectedLineGraph to_directed_line_graph(tglib::OrderedEdgeList<E> const &tgs){
    auto tg = to_incident_lists<TGNode, E>(tgs);
    return to_directed_line_graph<TGNode>(tg);
}

/**
 * @brief Computes the underlying aggregated graph in which the edges are weighted with the frequency
 * of their temporal contacts.
 * @tparam E The edge type of the input graph
 * @param tgs The input graph
 * @return An edge-weighted static graph as list of static edges
 */
template<typename E>
std::vector<StaticWeightedEdge> to_aggregated_edge_list(tglib::OrderedEdgeList<E> const &tgs) {
    std::map<std::pair<NodeId, NodeId>, long> edges;
    for (auto &e : tgs.getEdges()) {
        edges[{e.u, e.v}] += 1;
    }
    std::vector<StaticWeightedEdge> aggregated_edge_list;
    for (auto &p : edges) {
        aggregated_edge_list.push_back({p.first.first, p.first.second, p.second});
    }
    return aggregated_edge_list;
}

/**
 * @brief Normalizes a temporal graph.
 *
 * Multiple edges will be removed and the resulting edge stream will be chronologically ordered.
 *
 * @tparam E The edge type
 * @param tgs The input graph
 * @param removeLoops If true, self-loops will be removed
 * @return Normalized temporal graph
 */
template<typename E>
OrderedEdgeList<E> normalize(tglib::OrderedEdgeList<E> const &tgs, bool removeLoops) {
    std::set<E> edges;
    for (auto &e : tgs.getEdges()) {
        if (removeLoops && e.u == e.v) continue;
        edges.insert(e);
    }
    std::vector<E> filtered_edges;
    TimeInterval ti = {inf, 0};
    for (auto &e : edges){
        filtered_edges.push_back(e);
        if (e.t < ti.first) ti.first = e.t;
        if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
    }
    OrderedEdgeList<E> normalized_graph(tgs.getNumberOfNodes(), filtered_edges, ti, tgs.getNodeMap());
    return normalized_graph;
}

/**
 * @brief Scales the time stamps by a given factor.
 * @tparam E The temporal edge type.
 * @param tgs The input graph.
 * @param factor The scaling factor.
 * @return The temporal graph with scaled time stamps.
 */
template<typename E>
OrderedEdgeList<E> scale_timestamps(tglib::OrderedEdgeList<E> const &tgs, double factor) {
    std::set<E> edges;
    for (auto &e : tgs.getEdges()) {
        auto f = e;
        f.t = std::round(f.t * factor); //todo maybe floor?
        edges.insert(f);
    }
    std::vector<E> filtered_edges;
    TimeInterval ti = {inf, 0};
    for (auto &e : edges){
        filtered_edges.push_back(e);
        if (e.t < ti.first) ti.first = e.t;
        if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
    }
    OrderedEdgeList<E> normalized_graph(tgs.getNumberOfNodes(), filtered_edges, ti, tgs.getNodeMap());
    return normalized_graph;
}

/**
 * @brief Replaces the transition times with a given value equal for all edges.
 * @tparam E The temporal edge type
 * @param tgs The input graph
 * @param val The new transition time for all edges
 * @return The resulting temporal graph
 */
template<typename E>
OrderedEdgeList<E> unit_transition_times(tglib::OrderedEdgeList<E> const &tgs, Time val) {
    TimeInterval ti = {inf, 0};
    std::vector<E> filtered_edges;
    for (auto e : tgs.getEdges()){
        e.tt = val;
        filtered_edges.push_back(e);
        if (e.t < ti.first) ti.first = e.t;
        if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
    }
    OrderedEdgeList<E> normalized_graph(tgs.getNumberOfNodes(), filtered_edges, ti, tgs.getNodeMap());
    return normalized_graph;
}

} // tglib



#endif //TGLIB_TRANSFORMATIONS_H
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

        auto p = topk.insert(TopkEntry(nid, value));
        if (!p.second) {
            const_cast<std::vector<NodeId>&>(p.first->nids).push_back(nid);
        }

        if (maxClosenessBound < value && topk.size() > k) {
            auto i = topk.find(TopkEntry(0, maxClosenessBound));
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
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file UtilFunctions.h
 *  @brief Helpful utility functions
 */

#ifndef TGLIB_UTILFUNCTIONS_H
#define TGLIB_UTILFUNCTIONS_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <valarray>
#include <chrono>
#include <iostream>


namespace tglib {

/**
 * Computes mean and standard deviation.
 * @tparam T a numerical type
 * @param values vector of numerical values
 * @param mean the mean
 * @param stdev the standard deviation
 */
template<typename T>
void get_mean_std(std::vector<T> &values, double &mean, double &stdev) {
    double sum = std::accumulate(std::begin(values), std::end(values), 0.0);
    mean = sum / values.size();
    double accum = 0.0;
    std::for_each (std::begin(values), std::end(values), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });
    stdev = sqrt(accum / (values.size()));
}

} // tglib

#endif //TGLIB_UTILFUNCTIONS_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalBetweenness.h
 *  @brief Contains function for computing temporal edge betweenness
 */

#ifndef TGLIB_TEMPORALBETWEENNESS_H
#define TGLIB_TEMPORALBETWEENNESS_H

#include <stack>
#include <queue>
#include <map>
#include "../core/DirectedLineGraph.h"
#include "../core/OrderedEdgeList.h"
#include "../core/Transformations.h"

namespace tglib {

/**
 * @brief Computes the temporal edge betweenness
 *
 * The temporal edge betweenness is defined as the number of min hop paths
 * passing over an edge.
 *
 * @param dlg Directed line graph representation of the temporal graph
 * @return The temporal edge betweenness of each edge
 */
inline std::vector<double> temporal_edge_betweenness(tglib::DirectedLineGraph const &dlg) {

    std::vector<double> betweenness(dlg.num_tg_nodes, 0);
    std::vector<double> betweenness_dlg(dlg.nodes.size(), 0);

    // For every node being starting node
    for (size_t s = 0; s < dlg.nodes.size(); s++) {
        std::stack<NodeId> S;

        std::vector<Distance> d(dlg.nodes.size(), -1);
        d[s] = 0;

        std::vector<NodeId> prev(dlg.nodes.size(), 0);
        std::queue<NodeId> Q;
        Q.push(s);

        std::vector<double> sigma(dlg.nodes.size(), 0);
        sigma[s] = 1;

        std::map<NodeId, std::vector<NodeId>> P; // neighbours visited

        // Brandes' alg.
        while(!Q.empty()) {
            int u = Q.front();
            Q.pop();
            S.push(u);

            for (auto &e : dlg.nodes[u].outedges) {
                NodeId v = e.v;
                if(d[v] < 0) {
                    Q.push(v);
                    d[v] = d[u] + 1;
                }

                // Update sigma if shortest path
                if(d[v] == d[u] + 1) {
                    sigma[v] = sigma[v] + sigma[u];
                    P[v].push_back(u);
                }
            }
        }
        std::vector<double> delta(dlg.nodes.size(), 0);

        while (!S.empty()) {
            int w = S.top();
            S.pop();
            for(auto it = P[w].begin(); it != P[w].end(); ++it)    {
                int v = *it;
                delta[v] += ((sigma[v]/sigma[w])*(1+ delta[w]));
            }
            if ((size_t)w != s) {
                betweenness_dlg[w] += delta[w];
            }
        }
    }

    return betweenness_dlg;
}

/**
 * @brief Computes the temporal edge betweenness
 *
 * The temporal edge betweenness is defined as the number of min hop paths
 * passing over an edge.
 * @tparam E The temporal edge type
 * @param tgs The temporal graph
 * @return The temporal edge betweenness of each edge
 */
template<typename E>
std::vector<double> temporal_edge_betweenness(tglib::OrderedEdgeList<E> const &tgs) {
    auto tg = to_incident_lists<TGNode>(tgs);
    auto dlg = to_directed_line_graph(tg);
    return temporal_edge_betweenness(dlg);
}

} // tglib

#endif //TGLIB_TEMPORALBETWEENNESS_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalClusteringCoefficient.h
 *  @brief Contains function for computing the temporal clustering coefficient.
 */

#ifndef TGLIB_TEMPORALCLUSTERINGCOEFFICIENT_H
#define TGLIB_TEMPORALCLUSTERINGCOEFFICIENT_H

#include "../core/BasicTypes.h"
#include "../core/IncidentLists.h"

namespace tglib {

/**
 * @brief Computes the temporal clustering coefficient
 *
 * The temporal clustering coefficient is defined as
 * \f[
 * C_C(u) = \frac{\sum_{t\in T(\mathcal{G})} \pi_t(u)}{|T(\mathcal{G})|{|N(u)| \choose 2}},
 * \f]
 * where \f$\pi_t(u)=|\{(v,w,t,\lambda)\in\mathcal{E}\mid v,w\in N(u)\}|\f$
 * and \f$N(u)\f$ the neighbors of \f$u\f$ [1].
 *
 * [1] Tang, John, et al. "Temporal distance metrics for social network analysis."
 * Proceedings of the 2nd ACM workshop on Online social networks. 2009.
 *
 * @tparam N Node type
 * @param tg The temporal graph
 * @param nid The node for which the temporal clustering coefficient is computed
 * @param ti A restrictive time interval
 * @return The temporal clustering coefficient only considering edges in ti
 */
template<typename N, typename E>
double temporal_clustering_coefficient(IncidentLists<N, E> const& tg, NodeId nid, TimeInterval ti){

    std::set<NodeId> neighbors;
    std::set<Time> timesteps;
    for (auto &e : tg.getNode(nid).outEdges) {
        if (e.t < ti.first || e.t > ti.second) continue;
        neighbors.insert(e.v);
        timesteps.insert(e.t);
    }

    double count = 0;
    for (auto &v : neighbors) {
        for (auto &e : tg.getNode(v).outEdges) {
            if (neighbors.find(e.v) != neighbors.end()) {
                count += 1;
            }
        }
    }

    if (count == 0) return 0;

    auto m = (double)(neighbors.size() * (neighbors.size() - 1));
    double result = (1.0 / (double)timesteps.size()) * (count / m);

    return result;
}

}

#endif //TGLIB_TEMPORALCLUSTERINGCOEFFICIENT_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalKatz.h
 *  @brief Contains function declarations for computing temporal Katz centrality
 */

#ifndef TGLIB_TEMPORALKATZ_H
#define TGLIB_TEMPORALKATZ_H

#include <vector>
#include <map>
#include "../core/OrderedEdgeList.h"

namespace tglib {

/**
 * @brief Computes the temporal Katz centrality [1].
 *
 *
 *  [1] Béres, Ferenc, et al. "Temporal walk based centrality metric for graph streams."
 *  Applied network science 3.1 (2018): 1-26.
 *
 * @tparam E Temporal edge type
 * @param tgs Temporal edge stream
 * @param beta Parameter for weighting the walks
 * @return The temporal Katz centrality for each node
 */
template<typename E>
std::vector<double> temporal_katz_centrality(OrderedEdgeList<E> const &tgs, double beta) {

    std::vector<double> r(tgs.getNumberOfNodes(), 0);

    std::vector<std::map<Time, double>> walks(tgs.getNumberOfNodes(), std::map<Time, double>());
    for (auto &e : tgs.getEdges()) {
        walks.at(e.v)[e.t+1] += 1; //todo tt
        for (auto &x : walks.at(e.u)) {
            if (x.first > e.t) {
                continue;
            }
            walks.at(e.v)[e.t + 1] += (beta * x.second); //todo tt
        }
    }
    for (size_t i = 0; i < walks.size(); ++i) {
        for (auto &x : walks[i]) {
            r[i] += x.second;
        }
    }

    return r;
}

} // tglib

#endif //TGLIB_TEMPORALKATZ_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalPageRank.h
 *  @brief Contains function declarations for computing TemporalPageRank centrality
 */

#ifndef TGLIB_TEMPORALPAGERANK_H
#define TGLIB_TEMPORALPAGERANK_H

#include <vector>
#include "../core/OrderedEdgeList.h"


namespace tglib {
/**
 * @brief Computes the temporal PageRank [1]
 *
 *
 * [1] Rozenshtein, Polina, and Aristides Gionis. "Temporal pagerank."
 * Joint European Conference on Machine Learning and Knowledge Discovery in Databases.
 * Springer, Cham, 2016.
 *
 * @tparam E The temporal edge type
 * @param tgs The ordered edge stream with edge type E
 * @param alpha Weighting factor
 * @param beta Weighting factor
 * @param gamma Weighting factor
 * @return The temporal PageRank centrality for each node
 */
template<typename E>
std::vector<double> temporal_pagerank(OrderedEdgeList<E> const &tgs, double alpha, double beta, double gamma) {

    std::vector<double> degrees(tgs.getNumberOfNodes(), 0.0);
    for (auto &e : tgs.getEdges()) {
        degrees.at(e.u)++;
    }
    for (auto &d : degrees) d /= tgs.getEdges().size();

    double normalization = 1.0;
    if (beta == 1.0) beta = 0.0;

    std::vector<double> r(tgs.getNumberOfNodes(), 0.0);
    std::vector<double> s(tgs.getNumberOfNodes(), 0.0);

    for (auto &e : tgs.getEdges()) {
        r[e.u] = r[e.u] * gamma + 1.0 * (1.0 - alpha) * degrees[e.u] * alpha * normalization;
        r[e.v] = r[e.v] * gamma + s[e.u] + 1.0 * (1.0 - alpha) * degrees[e.u] * alpha * normalization;
        s[e.v] = s[e.v] + (s[e.u] + 1.0 * (1.0 - alpha)* s[e.u]) * alpha * (1.0 - beta);
        s[e.u] = s[e.u] * beta;
    }

    return r;
}

} // tglib


#endif //TGLIB_TEMPORALPAGERANK_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file Burstiness.h
 *  @brief Contains function declarations for computing edge_burstiness
 */

#ifndef TGLIB_BURSTINESS_H
#define TGLIB_BURSTINESS_H


#include "../core/OrderedEdgeList.h"
#include "../util/UtilFunctions.h"
#include <map>

namespace tglib {

/**
 * @brief Computes the edge burstiness for all (static) edges
 *
 * Burstiness measures how much a sequence of contacts \f$\tau\f$
 * of a pair of edges deviates from the
 * memoryless random Poisson process [1].
 * It is defined as
 * \f[
 *      B(\tau)=\frac{\sigma_\tau - m_\tau}{\sigma_\tau + m_\tau} \in [-1,1],
 * \f]
 * where \f$\sigma_\tau\f$ and \f$m_\tau\f$ denote the standard deviation and mean
 * of the inter-contact times \f$\tau\f$, respectively [1].
 * A value close to one indicates a very bursty sequence, and a value
 * close to minus one a more periodic sequence.
 *
 * [1] Goh, K-I., and A-L. Barabási. "Burstiness and memory in complex systems."
 * EPL (Europhysics Letters) 81.4 (2008): 48002.
 *
 * @tparam E The temporal edge type
 * @param tgs The temporal graph as ordered edge list
 * @param ti The time interval for which the edge_burstiness is computed
 * @return For each pair of nodes with at least one temporal edge connecting them, the edge_burstiness of the tie
 */
template<typename E>
std::map<std::pair<tglib::NodeId, tglib::NodeId>, double>
edge_burstiness(tglib::OrderedEdgeList<E> const &tgs,
                tglib::TimeInterval ti) {
    std::map<std::pair<NodeId, NodeId>, double> result;
    std::map<std::pair<NodeId, NodeId>, std::vector<Time>> inter_contact_times;
    std::vector<Time> last_time(tgs.getEdges().size());

    for (auto &e : tgs.getEdges()) {
        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;
        inter_contact_times[{e.u, e.v}].push_back(e.t);
    }


#pragma omp parallel default(none) shared(inter_contact_times, result)
    for (auto &p : inter_contact_times) {
        if (p.second.size() < 2) continue;
        Time lt = p.second[0];
        std::vector<Time> ict(p.second.size()-1);
        for (size_t i = 1; i < p.second.size(); ++i) {
            Time nt = p.second[i];
            ict[i - 1] = nt - lt;
            lt = nt;
        }

        double m = 0, s = 0;
        get_mean_std(ict, m, s);
#pragma omp critical
        result[p.first] = (s - m) / (s + m);
    }

    return result;
}

/**
 * @brief Computes the edge burstiness for all nodes
 *
 * Burstiness measures how much a sequence of contacts \f$\tau\f$
 * of a single node  deviates from the
 * memoryless random Poisson process [1].
 * It is defined as
 * \f[
 *      B(\tau)=\frac{\sigma_\tau - m_\tau}{\sigma_\tau + m_\tau} \in [-1,1],
 * \f]
 * where \f$\sigma_\tau\f$ and \f$m_\tau\f$ denote the standard deviation and mean
 * of the inter-contact times \f$\tau\f$, respectively [1].
 * A value close to one indicates a very bursty sequence, and a value
 * close to minus one a more periodic sequence.
 *
 * [1] Goh, K-I., and A-L. Barabási. "Burstiness and memory in complex systems."
 * EPL (Europhysics Letters) 81.4 (2008): 48002.
 *
 * @tparam E The temporal edge type
 * @param tgs The temporal graph as ordered edge list
 * @param ti The time interval for which the node burstiness is computed
 * @return For each node the node burstiness
 */
template<typename E>
std::vector<double>
node_burstiness(tglib::OrderedEdgeList<E> const &tgs,
                tglib::TimeInterval ti){
    std::vector<double> result(tgs.getNumberOfNodes(), 0);
//    std::vector<double> result(tgs.getNumberOfNodes(), std::numeric_limits<double>::max());
    std::vector<std::vector<Time>> inter_contact_times(tgs.getNumberOfNodes(), std::vector<Time>());
    std::vector<Time> last_time(tgs.getEdges().size());

    for (auto &e : tgs.getEdges()) {
        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;
        inter_contact_times[e.u].push_back(e.t);
    }

#pragma omp parallel default(none) shared(inter_contact_times, tgs, result)
    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto &p = inter_contact_times[nid];
        if (p.size() < 2) continue;
        std::vector<Time> ict(p.size()-1);
        Time lt = p[0];
        for (size_t i = 1; i < p.size(); ++i) {
            Time nt = p[i];
            ict[i - 1] = nt - lt;
            lt = nt;
        }

        double m = 0, s = 0;
        get_mean_std(ict, m, s);
#pragma omp critical
        result[nid] = (s - m) / (s + m);
    }

    return result;
}

} // tglib

#endif //TGLIB_BURSTINESS_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalDistances.h
 *  @brief Contains function declarations for computing temporal distances.
 */

#ifndef TGLIB_TEMPORALDISTANCES_H
#define TGLIB_TEMPORALDISTANCES_H


#include <vector>
#include <list>
#include <memory>
#include "../core/BasicTypes.h"
#include "../core/OrderedEdgeList.h"
#include "../core/TRSGraph.h"
#include "../core/IncidentLists.h"
#include "../util/LabelPQ.h"

namespace tglib {

/**
 * @brief Computes the durations of fastest paths from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The durations of the fastest paths
 */
template<typename E>
std::vector<tglib::Time>
minimum_durations(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> durs(tgs.getNumberOfNodes(), inf);
    durs[nid] = 0;
    std::vector<std::set<std::pair<Time, Time>>> node_labels(tgs.getNumberOfNodes(), std::set<std::pair<Time, Time>>());

    for (auto &e: tgs.getEdges()) {
        if (e.t >= ti.second) break;
        if (e.t + e.tt > ti.second || e.t < ti.first) continue;

        if (e.u == nid) {
            node_labels[e.u].insert(std::make_pair(e.t, e.t));
        }

        if (node_labels[e.u].empty()) continue;

        auto nodeLabelIt = node_labels[e.u].upper_bound(std::make_pair(e.t, inf));

        if (nodeLabelIt != node_labels[e.u].begin()) {
            nodeLabelIt--;

            Time arrivalTime = e.t + e.tt;
            Time startTime = nodeLabelIt->second;

            if (arrivalTime - startTime < durs[e.v]) {
                durs[e.v] = arrivalTime - startTime;
            }

            if (node_labels[e.v].empty()) {
                node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                continue;
            }

            auto lowerBound = node_labels[e.v].lower_bound(std::make_pair(arrivalTime, startTime));

            if (lowerBound == node_labels[e.v].begin()) {
                if (lowerBound->first > arrivalTime) {
                    auto upperBound = lowerBound;
                    for (; upperBound != node_labels[e.v].end(); upperBound++) {
                        if (upperBound->second > startTime) {
                            break;
                        }
                    }

                    node_labels[e.v].erase(lowerBound, upperBound);
                    node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                }
            } else if (lowerBound == node_labels[e.v].end()) {
                lowerBound--;
                if (lowerBound->first == arrivalTime) {
                    node_labels[e.v].erase(lowerBound);
                    node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                } else if (lowerBound->second < startTime) {
                    node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                }
            } else {
                if (lowerBound->first > arrivalTime) {
                    auto oneBeforeLowerBound = lowerBound;
                    oneBeforeLowerBound--;

                    if (oneBeforeLowerBound->first == arrivalTime) {
                        for (; lowerBound != node_labels[e.v].end(); lowerBound++) {
                            if (lowerBound->second > startTime) {
                                break;
                            }
                        }

                        node_labels[e.v].erase(oneBeforeLowerBound, lowerBound);
                        node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                    } else if (oneBeforeLowerBound->second < startTime) {
                        auto upperBound = lowerBound;
                        for (; upperBound != node_labels[e.v].end(); upperBound++) {
                            if (upperBound->second > startTime) {
                                break;
                            }
                        }

                        node_labels[e.v].erase(lowerBound, upperBound);
                        node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                    }
                }
            }

        }
    }

    return durs;
}

/**
 * @brief Computes the earliest arrival times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @tparam E the edge type
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
template<typename E>
std::vector<tglib::Time>
earliest_arrival_times(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrival_time(tgs.getNumberOfNodes(), inf);
    arrival_time[nid] = 0;

    for (auto &e : tgs.getEdges()) {

        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;

        if (arrival_time[e.u] <= e.t && arrival_time[e.v] > e.t + e.tt) {
            arrival_time[e.v] = e.t + e.tt;
        }
    }

    return arrival_time;
}


/**
 * @brief Computes the latest departure times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The lastest departure times
 */
template<typename E>
std::vector<tglib::Time>
latest_departure_times(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrival_time(tgs.getNumberOfNodes(), -1);
    arrival_time[nid] = 0;

    for (long i = (long)tgs.getEdges().size()-1; i >= 0; --i) {
        auto &e = tgs.getEdges()[i];

        if (e.t < ti.first) break;

        if (arrival_time[e.u] >= e.t + e.tt && arrival_time[e.v] < e.t) {
            arrival_time[e.v] = e.t;
        }
    }

    return arrival_time;
}

/**
 * @brief Computes the minimum number of hops from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum number of hops
 */
template<typename E>
std::vector<tglib::Distance>
minimum_hops(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> distances(tgs.getNumberOfNodes(), inf);

    distances[nid] = 0;
    std::vector<std::set<std::pair<Time, Distance> > > nodeLabels;
    nodeLabels.resize(tgs.getNumberOfNodes());

    for (auto &e: tgs.getEdges()) {
        if (e.t >= ti.second) break;
        if (e.t + e.tt > ti.second || e.t < ti.first) continue;

        if (e.u == nid) {
            nodeLabels[e.u].insert(std::make_pair(e.t, 0));
        }

        if (nodeLabels[e.u].empty()) continue;

        auto upperBound = nodeLabels[e.u].upper_bound({e.t, inf});

        if (upperBound != nodeLabels[e.u].begin()) {
            upperBound--;

            auto arrivalTime = e.t + e.tt;
            auto distance = upperBound->second + 1;

            if (distance < distances[e.v])
                distances[e.v] = distance;

            if (!nodeLabels[e.v].empty()) {
                auto bound = nodeLabels[e.v].upper_bound({arrivalTime, distance});

                if (bound == nodeLabels[e.v].begin()) {
                    auto boundLow = bound, bouldUp = bound;
                    for (; bouldUp != nodeLabels[e.v].end(); bouldUp++) {
                        if (bouldUp->second < distance) {
                            break;
                        }
                    }

                    nodeLabels[e.v].erase(boundLow, bouldUp);
                    nodeLabels[e.v].insert({arrivalTime, distance});

                } else if (bound == nodeLabels[e.v].end()) {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                } else {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        auto boundLow = bound, boundUp = bound;
                        for (; boundUp != nodeLabels[e.v].end(); boundUp++) {
                            if (boundUp->second < distance) {
                                break;
                            }
                        }

                        nodeLabels[e.v].erase(boundLow, boundUp);
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                }
            } else {
                nodeLabels[e.v].insert({arrivalTime, distance});
            }
        }
    }
    return distances;
}

/**
 * @brief Computes the minimum sum of transition times from the node with id nid to all, other during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum sum of transition times
 */
template<typename E>
std::vector<tglib::Time>
minimum_transition_times(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid,
                         tglib::TimeInterval ti) {
    std::vector<Time> distances(tgs.getNumberOfNodes(), inf);

    distances[nid] = 0;
    std::vector<std::set<std::pair<Time, Distance> > > nodeLabels;
    nodeLabels.resize(tgs.getNumberOfNodes());

    for (auto &e: tgs.getEdges()) {
        if (e.t >= ti.second) break;
        if (e.t + e.tt > ti.second || e.t < ti.first) continue;

        if (e.u == nid) {
            nodeLabels[e.u].insert(std::make_pair(e.t, 0));
        }

        if (nodeLabels[e.u].empty()) continue;

        auto upperbound = nodeLabels[e.u].upper_bound({e.t, inf});

        if (upperbound != nodeLabels[e.u].begin()) {
            upperbound--;

            auto arrivalTime = e.t + e.tt;
            auto distance = upperbound->second + e.tt;

            if (distance < distances[e.v])
                distances[e.v] = distance;

            if (!nodeLabels[e.v].empty()) {
                auto bound = nodeLabels[e.v].upper_bound({arrivalTime, distance});

                if (bound == nodeLabels[e.v].begin()) {
                    auto boundLow = bound, boundUp = bound;
                    for (; boundUp != nodeLabels[e.v].end(); boundUp++) {
                        if (boundUp->second < distance) {
                            break;
                        }
                    }

                    nodeLabels[e.v].erase(boundLow, boundUp);
                    nodeLabels[e.v].insert({arrivalTime, distance});

                } else if (bound == nodeLabels[e.v].end()) {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                } else {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        auto boundLow = bound, boundUp = bound;
                        for (; boundUp != nodeLabels[e.v].end(); boundUp++) {
                            if (boundUp->second < distance) {
                                break;
                            }
                        }

                        nodeLabels[e.v].erase(boundLow, boundUp);
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                }
            } else {
                nodeLabels[e.v].insert({arrivalTime, distance});
            }
        }
    }

    return distances;
}



/**
 * @brief Computes the durations of fastest paths from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @tparam The node type
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The durations of the fastest paths
 */
template<typename N, typename E>
std::vector<tglib::Time>
minimum_durations(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> durations(tg.getNumberOfNodes(), inf);
    durations[nid] = 0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector<std::list<std::shared_ptr<DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels[nid].push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    size_t num_f = 0;

    while (!q.empty() && num_f < tg.getNumberOfNodes()) {

        auto cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        if (!visited[cur->nid]) {
            visited[cur->nid] = true;
            ++num_f;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                if (cur->s == 0) lnew->s = e.t;
                else lnew->s = cur->s;
                lnew->a = e.t + e.tt;
                lnew->d = lnew->a - lnew->s;

                bool dom = false;

                auto i = nodelabels[e.v].begin();
                while (i != nodelabels[e.v].end()) {
                    if (((*i)->s < lnew->s && (*i)->a >= lnew->a) || ((*i)->s == lnew->s && (*i)->a > lnew->a)) {
                        (*i)->deleted = true;
                        i = nodelabels[e.v].erase(i);
                        continue;
                    }
                    if (((*i)->s >= lnew->s && (*i)->a <= lnew->a) || ((*i)->s == lnew->s && (*i)->a <= lnew->a)) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    durations[e.v] = durations[e.v] < lnew->d ? durations[e.v] : lnew->d;
                    nodelabels[e.v].push_back(lnew);
                    q.push(lnew);
                }
            }
        }
    }
    return durations;
}

/**
 * @brief Computes the earliest arrival times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @tparam E the node type
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
template<typename N, typename E>
std::vector<tglib::Time>
earliest_arrival_times(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrivalTimes(tg.getNumberOfNodes(), inf);
    arrivalTimes[nid] = 0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector < std::list < std::shared_ptr < DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->d = 0;
    l->nid = nid;
    nodelabels.at(nid).push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    while (!q.empty()) {

        auto cur = q.top();
        q.pop();
        if (!visited[cur->nid]) {
            visited[cur->nid] = true;
        } else {
            continue;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {
            if (e.t < ti.first || e.t + e.tt > ti.second) continue;
            if (e.t >= arrivalTimes[e.u]) {
                if (arrivalTimes[e.v] > e.t + e.tt) {
                    arrivalTimes[e.v] = e.t + e.tt;
                }
                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                lnew->d = e.t + e.tt;

                q.push(lnew);
            }
        }
    }

    return arrivalTimes;
}


/**
 * @brief Computes the minimum sum of transition times from the node with id nid to all, other during the
 * time interval ti reachable, nodes.
 *
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum sum of transition times
 */
template<typename N, typename E>
std::vector<tglib::Distance>
minimum_transition_times(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> distances(tg.getNumberOfNodes(), inf);
    distances[nid] = 0;

    std::vector<std::list<std::shared_ptr<DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels[nid].push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    while (!q.empty() ) {

        auto cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                lnew->a = e.t + e.tt;
                lnew->d = cur->d + e.tt;

                bool dom = false;

                auto i = nodelabels[e.v].begin();
                while (i != nodelabels[e.v].end()) {
                    //    d' < d and a' <= a
                    // or d' = d and a' < a
                    if ((lnew->d < (*i)->d && lnew->a <= (*i)->a) /*|| (lnew->d == (*i)->d && lnew->a < (*i)->a)*/) {
                        (*i)->deleted = true;
                        i = nodelabels[e.v].erase(i);
                        continue;
                    }
                    if (((*i)->d <= lnew->d && (*i)->a <= lnew->a) /*|| ((*i)->d == lnew->d && (*i)->a < lnew->a)*/) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    distances[e.v] = distances[e.v] < lnew->d ? distances[e.v] : lnew->d;
                    nodelabels[e.v].push_back(lnew);
                    q.push(lnew);
                }
            }
        }
    }
    return distances;
}

/**
 * @brief Computes the minimum number of hops from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum number of hops
 */
template<typename N, typename E>
std::vector<tglib::Distance>
minimum_hops(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> distances(tg.getNumberOfNodes(), inf);
    distances[nid] = 0;

    std::vector<std::list<std::shared_ptr<DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels[nid].push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    while (!q.empty() ) {

        auto cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                lnew->a = e.t + e.tt;
                lnew->d = cur->d + 1;

                bool dom = false;

                auto i = nodelabels[e.v].begin();
                while (i != nodelabels[e.v].end()) {
                    //    d' < d and a' <= a
                    // or d' = d and a' < a
                    if ((lnew->d < (*i)->d && lnew->a <= (*i)->a) /*|| (lnew->d == (*i)->d && lnew->a < (*i)->a)*/) {
                        (*i)->deleted = true;
                        i = nodelabels[e.v].erase(i);
                        continue;
                    }
                    if (((*i)->d <= lnew->d && (*i)->a <= lnew->a) /*|| ((*i)->d == lnew->d && (*i)->a < lnew->a)*/) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    distances[e.v] = distances[e.v] < lnew->d ? distances[e.v] : lnew->d;
                    nodelabels[e.v].push_back(lnew);
                    q.push(lnew);
                }
            }
        }
    }
    return distances;
}


/**
 * @brief Computes the durations of fastest paths from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph in TRSGraph representation
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The durations of the fastest paths
 */
inline std::vector<tglib::Time>
minimum_durations(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<bool> visited(trs.nodes.size(), false);
    std::vector<Time> durs(trs.numTGNodes, inf);
    durs[nid] = 0;
    std::vector<NodeId> stack;
    stack.reserve(trs.nodes.size());

    auto startPos = trs.positionOfLatestTimeNode[nid];

    // the nodes in trs are ordered sequential for a tg-node in increasing time
    while (startPos >=0 && trs.nodes[startPos].tn.first == nid) {
        auto &nextNode = trs.nodes[startPos];
        startPos--;

        if (nextNode.tn.second < ti.first) break;    // all next nodes would be not later
        if (nextNode.tn.second > ti.second) continue;

        visited[nextNode.nid] = true;
        stack.push_back(nextNode.nid);

        Time startTime = nextNode.tn.second;

        while (!stack.empty()) {
            auto &curNode = trs.nodes[stack.back()];
            stack.pop_back();

            for (auto &e : curNode.adjList) {
                Time arrTime = curNode.tn.second + e.tt;
                Time duration = arrTime - startTime;
                NodeId v = trs.nodes[e.v].tn.first;
                if (arrTime <= ti.second && duration < durs[v]) {
                    durs[v] = duration;
                }

                if (!visited[e.v]) {
                    visited[e.v] = true;
                    if (trs.nodes[e.v].tn.second <= ti.second) {
                        stack.push_back(e.v);
                        size_t s = e.v;
                        while (s < trs.nodes.size() && trs.nodes[s].tn.first == e.v) {
                            if (trs.nodes[s].tn.second > ti.second) break;
                            if (!visited[trs.nodes[s].nid]) {
                                visited[trs.nodes[s].nid] = true;
                                stack.push_back(trs.nodes[s].nid);
                            }
                            s++;
                        }
                    }
                }
            }
        }
    }

    return durs;
}


/**
 * @brief Computes the earliest arrival times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph in TRSGraph representation
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
inline std::vector<tglib::Time>
earliest_arrival_times(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<bool> visited(trs.nodes.size(), false);
    std::vector<Time> arrival_times(trs.numTGNodes, inf);
    arrival_times[nid] = 0;
    std::vector<NodeId> stack;
    stack.reserve(trs.nodes.size());

    auto startPos = trs.positionOfEarliestTimeNode[nid];

    // the nodes in trs are ordered sequential for a tg-node in increasing time
    while (startPos <= trs.positionOfLatestTimeNode[nid]) {
        auto &nextNode = trs.nodes[startPos];
        startPos++;

        if (nextNode.tn.second < ti.first) continue;
        if (nextNode.tn.second > ti.second) break;

        visited[nextNode.nid] = true;
        stack.push_back(nextNode.nid);

        while (!stack.empty()) {
            auto &curNode = trs.nodes[stack.back()];
            stack.pop_back();

            for (auto &e : curNode.adjList) {
                if (visited[e.v]) continue;
                visited[e.v] = true;
                stack.push_back(e.v);

                Time arrTime = curNode.tn.second + e.tt;
                NodeId v = trs.nodes[e.v].tn.first;
                if (arrTime < ti.second && arrTime < arrival_times[v]) {
                    arrival_times[v] = arrTime;
                }
            }

        }
    }

    return arrival_times;
}


/**
 * @brief Computes the minimum number of hops from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum number of hops
 */
inline std::vector<tglib::Distance>
minimum_hops(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti){
    std::vector<Distance> distances(trs.nodes.size(), inf);

    // todo corner case: vertex is isolated
    if (trs.positionOfEarliestTimeNode[nid] == inf) {
        std::vector<Distance> result(trs.numTGNodes, inf);
        result[nid] = 0;
        return result;
    }

    distances[trs.positionOfEarliestTimeNode[nid]] = 0;

    std::vector<bool> visited(trs.nodes.size(), false);

    auto cmp=[](const std::pair<Distance, NodeId>& a,const std::pair<Distance, NodeId>& b){
        return a.first > b.first;
    };
    std::priority_queue<std::pair<Distance, NodeId>,std::vector<std::pair<Distance, NodeId>>,decltype(cmp)> q(cmp);

    q.push({0, trs.positionOfEarliestTimeNode[nid]});

    while (!q.empty()) {

        auto cur = q.top();
        q.pop();

        auto &u = trs.nodes[cur.second];
        Time u_t = u.tn.second;

        if (visited[u.nid] || u_t > ti.second || u_t < ti.first) {
            continue;
        }
        visited[u.nid] = true;

        for (auto &e : u.adjList) {
            auto &v = trs.nodes[e.v];
            if (v.tn.first == u.tn.first) {
                if (distances[v.nid] > cur.first)
                    distances[v.nid] = cur.first;
                q.push({distances[v.nid], v.nid});
                continue;
            }
            if (distances[v.nid] > cur.first + 1)
                distances[v.nid] = distances[u.nid] + 1;
            q.push({distances[v.nid], v.nid});
        }
    }

    std::vector<Distance> result(trs.numTGNodes, inf);
    for (size_t i = 0; i < trs.nodes.size(); ++i) {
        result[trs.nodes[i].tn.first] = std::min(distances[i], result[trs.nodes[i].tn.first]);
    }
    return result;
}


/**
 * @brief Computes the minimum sum of transition times from the node with id nid to all, other during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum sum of transition times
 */
inline std::vector<tglib::Distance>
minimum_transition_times(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti){
    std::vector<Distance> distances(trs.nodes.size(), inf);

    // todo corner case: vertex is isolated
    if (trs.positionOfEarliestTimeNode[nid] == inf) {
        std::vector<Distance> result(trs.numTGNodes, inf);
        result[nid] = 0;
        return result;
    }

    distances[trs.positionOfEarliestTimeNode[nid]] = 0;

    std::vector<bool> visited(trs.nodes.size(), false);

    auto cmp=[](const std::pair<Distance, NodeId>& a,const std::pair<Distance, NodeId>& b){
        return a.first > b.first;
    };
    std::priority_queue<std::pair<Distance, NodeId>,std::vector<std::pair<Distance, NodeId>>,decltype(cmp)> q(cmp);

    q.push({0, trs.positionOfEarliestTimeNode[nid]});

    while (!q.empty()) {

        auto cur = q.top();
        q.pop();

        auto &u = trs.nodes[cur.second];
        Time u_t = u.tn.second;

        if (visited[u.nid] || u_t > ti.second || u_t < ti.first) {
            continue;
        }
        visited[u.nid] = true;

        for (auto &e : u.adjList) {
            auto &v = trs.nodes[e.v];
            if (v.tn.first == u.tn.first) {
                if (distances[v.nid] > cur.first)
                    distances[v.nid] = cur.first;
                q.push({distances[v.nid], v.nid});
                continue;
            }
            if (distances[v.nid] > cur.first + e.tt)
                distances[v.nid] = distances[u.nid] + e.tt;
            q.push({distances[v.nid], v.nid});
        }
    }

    std::vector<Distance> result(trs.numTGNodes, inf);
    for (size_t i = 0; i < trs.nodes.size(); ++i) {
        result[trs.nodes[i].tn.first] = std::min(distances[i], result[trs.nodes[i].tn.first]);
    }
    return result;
}


} // tglib


#endif //TGLIB_TEMPORALDISTANCES_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalCloseness.h
 *  @brief Contains function declarations for computing closeness centrality
 */

#ifndef TGLIB_TEMPORALCLOSENESS_H
#define TGLIB_TEMPORALCLOSENESS_H


#include "../core/OrderedEdgeList.h"
#include "../core/IncidentLists.h"
#include "../core/TRSGraph.h"
#include "../algorithms/TemporalDistances.h"

namespace tglib {

/**
 * @brief Computes the closeness of a single node
 *
 * @param tgs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param timeInterval The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename E>
double temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                         tglib::NodeId nid,
                         tglib::TimeInterval timeInterval,
                         tglib::Distance_Type distanceType) {
    std::vector<Time> distances;

    switch (distanceType) {

        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times<E>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations(tgs, nid, timeInterval);
            break;
        case Distance_Type::Latest_Departure:
            distances = latest_departure_times(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Hops:
            distances = minimum_hops(tgs, nid, timeInterval);
            break;
    }

    double closeness = 0;

    for (long distance : distances) {
        if (distance > 0 && distance < inf) {
            closeness += 1.0 / (double)distance;
        }
    }

    return closeness;
}

/**
 * @brief Computes the closeness of a single node
 *
 * @param tgs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param timeInterval The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename TempGraph>
double run_temporal_closeness(TempGraph const &tgs,
                         tglib::NodeId nid,
                         tglib::TimeInterval timeInterval,
                         tglib::Distance_Type distanceType) {
    std::vector<Time> distances;

    switch (distanceType) {

        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Latest_Departure:
            distances = latest_departure_times<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Hops:
            distances = minimum_hops<TempGraph>(tgs, nid, timeInterval);
            break;
    }

    double closeness = 0;

    for (long distance : distances) {
        if (distance > 0 && distance < inf) {
            closeness += 1.0 / (double)distance;
        }
    }

    return closeness;
}

/**
 * @brief Computes the closeness of a single node
 *
 * @param tgs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename E>
double temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                                 tglib::NodeId nid,
                                 tglib::Distance_Type distanceType) {
    return temporal_closeness(tgs, nid, tgs.getTimeInterval(), distanceType);
}

/**
 * Computes the closeness of a single node
 * @param tg The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param ti The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename N, typename E>
double temporal_closeness(tglib::IncidentLists<N, E> const &tg,
                         tglib::NodeId nid,
                         tglib::TimeInterval ti,
                         tglib::Distance_Type distanceType) {
    std::vector<Time> distances;
    switch (distanceType) {
        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times(tg, nid, ti);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations(tg, nid, ti);
            break;
        case Distance_Type::Latest_Departure:
// todo           latest_departure_times(tg, nid, ti);
            break;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times(tg, nid, ti);
            break;
        case Distance_Type::Minimum_Hops:
// todo           distances = minimum_hops(tg, nid, ti);
            break;
    }
    double closeness = 0;
    for (auto &d : distances) {
        if (d > 0 && d < inf) {
            closeness += 1.0 / (double)d;
        }
    }
    return closeness;
}

/**
 * Computes the closeness of a single node
 * @param trs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param ti The restrictive time interval.
 * @return The temporal closeness of the node with id nid.
 */
inline double temporal_closeness(tglib::TRSGraph &trs,
                         tglib::NodeId nid,
                         tglib::TimeInterval ti) {
    auto durations = minimum_durations(trs, nid, ti);
    double closeness = 0;
    for (auto &d : durations) {
        if (d > 0 && d < inf) {
            closeness += 1.0 / (double)d;
        }
    }
    return closeness;
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tgs The temporal graph.
 * @param timeInterval The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness values of all nodes
 */
template<typename E>
std::vector<double> temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                                      tglib::TimeInterval timeInterval,
                                      tglib::Distance_Type distanceType) {
    std::vector<double> closeness_values(tgs.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tgs, timeInterval, closeness_values, distanceType)
    for (size_t i = 0; i < tgs.getNumberOfNodes(); ++i) {
        auto c = temporal_closeness(tgs, i, timeInterval, distanceType);
        closeness_values[i] = c;
    }

    return closeness_values;
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tgs The temporal graph.
 * @param distanceType The distance type.
 * @return The temporal closeness values of all nodes
 */
template<typename E>
std::vector<double> temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                                      tglib::Distance_Type distanceType) {
    return temporal_closeness(tgs, tgs.getTimeInterval() , distanceType);
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tg The temporal graph.
 * @param ti The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness values of all nodes
 */
template<typename N, typename E>
std::vector<double> temporal_closeness(tglib::IncidentLists<N, E>&tg, tglib::TimeInterval ti,
                                      tglib::Distance_Type distanceType) {
    std::vector<double> closeness(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, ti, closeness, distanceType)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        closeness[nid] = temporal_closeness(tg, (NodeId)nid, ti, distanceType);
    }

    return closeness;
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tg The temporal graph.
 * @param distanceType The distance type
 * @return The temporal closeness values of all nodes
 */
template<typename TempGraph>
std::vector<double> run_temporal_closeness(TempGraph const& tg, tglib::Distance_Type distanceType) {
    std::vector<double> closeness(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, closeness, distanceType)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        closeness[nid] = temporal_closeness(tg, nid, tg.getTimeInterval(), distanceType);
    }

    return closeness;
}

} // tglib

#endif //TGLIB_TEMPORALCLOSENESS_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalWalkCentrality.h
 *  @brief Contains function declarations for computing temporal walk centrality.
 */

#ifndef TGLIB_TEMPORALWALKCENTRALITY_H
#define TGLIB_TEMPORALWALKCENTRALITY_H


#include <map>
#include "../core/OrderedEdgeList.h"

namespace tglib {

/**
 * @brief counts incoming walks
 * @tparam E Temporal edge type
 * @param tgs The temporal graph
 * @param alpha weighting factor
 * @return walk counts
 */
template<typename E>
std::vector<std::map<Time, double>> getIncomingWeightedWalks(OrderedEdgeList<E> const &tgs, double alpha) {
    std::vector<std::map<Time, double>> walks(tgs.getNumberOfNodes(), std::map<Time, double>());
    for (auto &e: tgs.getEdges()) {
        walks.at(e.v)[e.t + 1] += 1; //alpha;
        for (auto &x: walks.at(e.u)) {
            if (x.first > e.t) {
                continue;
            }
            walks.at(e.v)[e.t + 1] += (alpha * x.second);
        }
    }
    return walks;
}

/**
 * @brief counts outgoing walks
 * @tparam E Temporal edge type
 * @param tgs The temporal graph
 * @param beta weighting factor
 * @return walk counts
 */
template<typename E>
std::vector<std::map<Time, double>> getOutgoingWeightedWalks(OrderedEdgeList<E> const &tgs, double beta) {
    std::vector<std::map<Time, double>> walks(tgs.getNumberOfNodes(), std::map<Time, double>());
    for (unsigned long ep = tgs.getEdges().size() - 1;; --ep) {
        auto &e = tgs.getEdges()[ep];
        walks.at(e.u)[e.t] += 1; //beta;
        for (auto &x: walks.at(e.v)) {
            if (x.first < e.t + 1) {
                continue;
            }
            walks.at(e.u)[e.t] += (beta * x.second);
        }
        if (ep == 0) break;
    }
    return walks;
}

/**
 * @brief Combines weighted temporal walk counts.
 *
 * @tparam E Temporal edge type
 * @param tgs The temporal graph
 * @param ifct incoming walk counts
 * @param ofct outgoing walk counts
 * @return the temporal walk centrality for all vertices
 */
template<typename E>
std::vector<double>
computeCentrality(OrderedEdgeList<E> const &tgs, std::vector<std::map<Time, double>> &ifct, std::vector<std::map<Time, double>> &ofct) {
    std::vector<double> centrality(tgs.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(centrality, tgs, ifct, ofct)
    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {

        auto init = ifct[nid].begin();
        auto outit = ofct[nid].begin();

        double insum = 0;

        while (outit != ofct[nid].end()) {
            if (init != ifct[nid].end() && outit != ofct[nid].end() && init->first < outit->first) {
                insum += init->second;
                ++init;
            }

            if (init != ifct[nid].end() && outit != ofct[nid].end() && outit->first < init->first) {
                if (init->first <= outit->first)
                    insum += init->second;
                centrality[nid] += insum * outit->second;
                ++outit;
            }
            if (init != ifct[nid].end() && outit != ofct[nid].end() && init->first == outit->first) {
                if (init->first <= outit->first)
                    insum += init->second;
                centrality[nid] += insum * outit->second;
                ++init;
                ++outit;
            }

            if (init == ifct[nid].end() && outit != ofct[nid].end()) {
                while (outit != ofct[nid].end()) {
                    centrality[nid] += insum * outit->second;
                    ++outit;
                }
                break;
            }
        }
    }
    return centrality;
}

/**
 * @brief Computes the temporal walk centrality [1].
 *
 * [1] Oettershagen, Lutz, Petra Mutzel, and Nils M. Kriege.
 * "Temporal Walk Centrality: Ranking Nodes in Evolving Networks."
 * Proceedings of the ACM Web Conference 2022. 2022.
 *
 * @tparam The temporal edge type
 * @param tgs the temporal graph
 * @param alpha the weighting factor for incoming walks
 * @param beta the weighting factor for outgoing walks
 * @return the centrality for each vertex
 */
template<typename E>
std::vector<double> temporal_walk_centrality(tglib::OrderedEdgeList<E> const &tgs,
                                                    double alpha, double beta) {
    std::vector<double> centrality;
    auto outwalks = getOutgoingWeightedWalks(tgs, beta);
    auto inwalks = getIncomingWeightedWalks(tgs, alpha);
    centrality = computeCentrality(tgs, inwalks, outwalks);
    return centrality;
}

} // tglib


#endif //TGLIB_TEMPORALWALKCENTRALITY_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalEfficiency.h
 *  @brief Contains function declarations for computing temporal efficiency.
 */

#ifndef TGLIB_TEMPORALEFFICIENCY_H
#define TGLIB_TEMPORALEFFICIENCY_H

#include <vector>
#include "../core/BasicTypes.h"
#include "../algorithms/TemporalCloseness.h"

/**
 * @brief Computes the temporal efficiency [1] for the temporal graph.
 *
 * The temporal efficiency is a global statistic based on the temporal closeness values of the nodes [1].
 * It is defined as
 * \f[
 * T_{eff}(\mathcal{G})=\frac{1}{n(n-1)}\sum_{u\in V(\mathcal{G})}\sum_{v\neq u\in V(\mathcal{G})}\frac{1}{d(u,v)}
 * \f]
 * with \f$d(u,v)\f$ being a temporal distance and \f$1/\infty=0\f$ in case of non-reachable vertices.
 *
 * [1]Tang, John, et al. "Temporal distance metrics for social network analysis."
 * Proceedings of the 2nd ACM workshop on Online social networks. 2009.
 *
 * @tparam TempGraph Temporal graph type.
 * @param tg the temporal graph
 * @param timeInterval the restrictive time interval
 * @param distanceType the temporal distance type
 * @return the efficiency of tg
 */
template<typename TempGraph>
double temporal_efficiency(TempGraph const& tg, tglib::TimeInterval timeInterval, tglib::Distance_Type distanceType) {
    std::vector<double> closeness(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, closeness, distanceType, timeInterval)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        closeness[nid] = temporal_closeness(tg, nid, timeInterval, distanceType);
    }

    double efficiency = 0;
    for (auto &c : closeness) {
        efficiency += c;
    }
    return 1.0/(tg.getNumberOfNodes()*(tg.getNumberOfNodes()-1)) * efficiency;
}

#endif //TGLIB_TEMPORALEFFICIENCY_H
//
// Created by lutz on 02.09.22.
//

#ifndef TGLIB_TEMPORALDIAMETER_H
#define TGLIB_TEMPORALDIAMETER_H

#include <vector>
#include "../core/BasicTypes.h"
#include "TemporalDistances.h"

namespace tglib {

/**
 * @brief Computes the maximum optimal temporal distance to any reachable vertex
 *
 * @tparam TempGraph The temporal graph type
 * @param tg The temporal graph
 * @param nid The node id
 * @param timeInterval The restrictive time interval
 * @param distanceType The distance time
 * @return The eccentricity of node nid
 */
template<typename TempGraph>
Time temporal_eccentricity(TempGraph const &tg, NodeId nid, tglib::TimeInterval timeInterval, tglib::Distance_Type distanceType) {
    std::vector<Time> distances;
    switch (distanceType) {

        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times(tg, nid, timeInterval);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations(tg, nid, timeInterval);
            break;
      case Distance_Type::Latest_Departure:
//     todo       distances = latest_departure_times(tg, nid, timeInterval);
            return inf;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times(tg, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Hops:
            distances = minimum_hops(tg, nid, timeInterval);
            break;
    }

    Time max_d = 0;
    for (auto const &d : distances) {
        if (d < inf && d > max_d)
            max_d = d;
    }
    return max_d;
}


/**
 * @brief Computes the temporal diameter, e.g., max. eccentricity in the temporal graph
 *
 * @tparam TempGraph The temporal graph type
 * @param tg The temporal graph
 * @param timeInterval The restrictive time interval
 * @param distanceType The distance time
 * @return The temporal diameter of tg
 */
template<typename TempGraph>
Time temporal_diameter(TempGraph const &tg, tglib::TimeInterval timeInterval, tglib::Distance_Type distanceType) {
    std::vector<Time> ecc(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, ecc, distanceType, timeInterval)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        ecc[nid] = temporal_eccentricity(tg, nid, timeInterval, distanceType);
    }

    Time diameter = 0;
    for (auto &e: ecc) {
        if (e < inf && e > diameter)
            diameter = e;
    }

    return diameter;
}

} // tglib

#endif //TGLIB_TEMPORALDIAMETER_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file TopologicalOverlap.h
 * @brief Contains function declarations for computing topological overlap.
 */

#ifndef TGLIB_TOPOLOGICALOVERLAP_H
#define TGLIB_TOPOLOGICALOVERLAP_H


#include <complex>
#include <algorithm>
#include "../core/BasicTypes.h"
#include "../core/IncidentLists.h"

namespace tglib {

/**
 * @brief Computes the topological overlap of a node.
 *
 * The topological overlap of a nodes is defined as
 * \f[
 * T_{to}(u)=\frac{1}{T(\mathcal{G})}\sum_{t=1}^{T(\mathcal{G})}\frac{\sum_{v\in\mathcal{N}(u)}\phi^t_{uv}\phi^{t+1}_{uv}}{\sqrt{\sum_{v\in\mathcal{N}(u)}\phi^t_{uv}\sum_{v\in\mathcal{N}(u)}\phi^{t+1}_{uv}}},
 * \f]
 * where \f$\phi^{t}_{uv}=1\f$ iff. there exists a temporal edges between \f$u\f$ and \f$v\f$ at time \f$t\f$ and zero otherwise [1].
 * In case that the denominator equals one, we define \f$T_{to}(u)=1\f$.
 * And, the global topological overlap is defined as
 * \f$T_{to}(\mathcal{G})=\frac{1}{N}\sum_{u\in V}T_{to}(u)\f$.
 * The topological overlap lies in the range between zero and one.
 * A value close to zero, means that many edges change and a value close to one means many edges change.
 *
 * [1]
 *
 * @tparam N The node type
 * @param tg The graph
 * @param nid The node id for which the topological overlap is computed
 * @param ti A restrictive time interval
 * @return The topological overlap for node nid in time interval ti
 */
template<typename N, typename E>
double topological_overlap(IncidentLists<N, E> const& tg, NodeId nid, TimeInterval ti) {
    std::vector<TemporalEdge> edges = tg.getNode(nid).outEdges;
    if (edges.empty()) return 0;

    std::sort(edges.begin(), edges.end());

    std::vector<std::set<NodeId>> counts;
    Time curTime = edges[0].t;
    counts.emplace_back();
    for (auto &e : edges) {
        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;
        if (e.t != curTime) {
            curTime = e.t;
            counts.emplace_back();
        }
        counts.back().insert(e.v);
    }

    double to = 0;
    for (size_t i = 0; i < counts.size()-1; ++i) {
        std::vector<int> v(std::min(counts[i].size(), counts[i+1].size()));
        auto it = std::set_intersection(counts[i].begin(), counts[i].end(),
                                        counts[i+1].begin(), counts[i+1].end(), v.begin());
        v.resize(it-v.begin());
        auto d = (double) v.size();
        auto n = (double) std::sqrt(counts[i].size() * counts[i+1].size());

        to += (d/n);
    }
    to /= ((double)counts.size() - 1.0);
    return to;
}


} // tglib


#endif //TGLIB_TOPOLOGICALOVERLAP_H
/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalReachability.h
 *  @brief Contains function declarations for computing temporal reachability.
 */


#ifndef TGLIB_REACHABILITY_H
#define TGLIB_REACHABILITY_H

#include "../core/BasicTypes.h"
#include "../core/OrderedEdgeList.h"
#include "../core/TRSGraph.h"
#include "../core/IncidentLists.h"
#include "../util/LabelPQ.h"

namespace tglib {

/**
 * @brief Computes the number of reachable nodes with a temporal walk starting at node id in the
 * time interval ti.
 *
 * Note that id reaches itself.
 *
 * @tparam E the edge type
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
template<typename E>
int64_t number_of_reachable_nodes(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrival_time(tgs.getNumberOfNodes(), inf);
    arrival_time[nid] = 0;
    int64_t reachable = 1;

    for (auto &e : tgs.getEdges()) {

    if (e.t < ti.first) continue;
    if (e.t > ti.second) break;

    if (arrival_time[e.u] <= e.t && arrival_time[e.v] > e.t + e.tt) {
        if (arrival_time[e.v] == inf)
            reachable++;
        arrival_time[e.v] = e.t + e.tt;
    }
}

return reachable;
}

} // tglib


#endif //TGLIB_REACHABILITY_H
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
