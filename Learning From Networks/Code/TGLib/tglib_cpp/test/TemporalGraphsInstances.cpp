/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <algorithm>
#include <set>
#include "TemporalGraphsInstances.h"


using namespace tglib;
using namespace std;

void addEdge(vector<TemporalEdge> &edges, NodeId u, NodeId v, Time t, Time tt) {
    edges.push_back(TemporalEdge{u, v, t, tt});
}

OrderedEdgeList<TemporalEdge> getTemporalGraphExample1() {
    std::vector<TemporalEdge> edges;
    addEdge(edges, 0, 1, 1, 1);
    addEdge(edges, 1, 3, 2, 1);
    addEdge(edges, 2, 3, 3, 1);
    addEdge(edges, 4, 3, 15, 1);
    addEdge(edges, 5, 6, 5, 1);
    addEdge(edges, 5, 7, 12, 1);
    addEdge(edges, 7, 8, 13, 1);
    addEdge(edges, 7, 9, 14, 1);
    OrderedEdgeList<TemporalEdge> tgs(10, edges, {0,15});

    return tgs;
}

OrderedEdgeList<TemporalEdge> getTemporalGraphExample2() {
    std::vector<TemporalEdge> edges;

    addEdge(edges, 0, 1, 2, 1);
    addEdge(edges, 1, 2, 2, 1);
    OrderedEdgeList<TemporalEdge> tgs(3, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}

OrderedEdgeList<TemporalEdge> getRandomGraphExample(int nodes, int num_edges) {
    std::set<TemporalEdge> edge_set;
    for (int i = 0; i < num_edges; ++i) {
        NodeId u = rand() % nodes;
        NodeId v = rand() % nodes;
        if (u==v) continue;
        edge_set.insert(TemporalEdge{u,v,1+rand()%num_edges,1+rand()%100});
//        edge_set.insert(TemporalEdge{u,v,1+rand()%edges,1});
    }

    std::vector<TemporalEdge> edges;
    for (auto &e : edge_set){
        edges.push_back(e);
    }

    OrderedEdgeList<TemporalEdge> tgs(nodes, edges, {0,1000000000});

    return tgs;
}


OrderedEdgeList<TemporalEdge> getChain() {
    std::vector<TemporalEdge> edges;

    addEdge(edges, 0, 1, 1, 1);
    addEdge(edges, 1, 2, 2, 1);
    addEdge(edges, 2, 3, 3, 1);
    addEdge(edges, 3, 4, 4, 1);
    addEdge(edges, 4, 5, 5, 1);
    addEdge(edges, 5, 6, 6, 1);
    addEdge(edges, 6, 7, 7, 1);

    OrderedEdgeList<TemporalEdge> tgs(8, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}


OrderedEdgeList<TemporalEdge> getNonBursty(int num_edges) {
    std::vector<TemporalEdge> edges;

    for (int i = 0; i < num_edges; ++i) {
        addEdge(edges, 0, 1, i, 1);
        addEdge(edges, 1, 0, i, 1);
    }

    OrderedEdgeList<TemporalEdge> tgs(2, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}


OrderedEdgeList<TemporalEdge> getBursty(int num_edges) {
    std::vector<TemporalEdge> edges;

    for (int i = 0; i < num_edges; ++i) {
        int x = rand() % 2;
        int t = rand() % 1000;
        int m = 1000000;
        if (x == 0)
            m = 0;
        addEdge(edges, 0, 1, t+m, 1);
        addEdge(edges, 1, 0, t+m, 1);
    }

    std::sort(edges.begin(), edges.end(), [](auto const &e1, auto const &e2){return e1.t < e2.t;});
    OrderedEdgeList<TemporalEdge> tgs(2, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}

tglib::OrderedEdgeList<TemporalEdge> getExampleFromPaper() {
    std::vector<TemporalEdge> edges;

    addEdge(edges, 0, 3, 1, 5);
    addEdge(edges, 0, 1, 2, 1);
    addEdge(edges, 0, 1, 5, 2);
    addEdge(edges, 2, 1, 6, 1);
    addEdge(edges, 3, 2, 6, 2);
    addEdge(edges, 1, 3, 7, 2);
    addEdge(edges, 3, 2, 8, 4);

    OrderedEdgeList<TemporalEdge> tgs(4, edges, {1, 12});

    return tgs;
}


tglib::OrderedEdgeList<TemporalEdge> getExampleFromPaperWithLoopsAndMulitEdges() {
    std::vector<TemporalEdge> edges;

    addEdge(edges, 0, 3, 1, 5);
    addEdge(edges, 0, 3, 1, 5);
    addEdge(edges, 0, 1, 2, 1);
    addEdge(edges, 0, 1, 5, 2);
    addEdge(edges, 0, 1, 5, 2);
    addEdge(edges, 0, 0, 5, 2);
    addEdge(edges, 2, 1, 6, 1);
    addEdge(edges, 3, 2, 6, 2);
    addEdge(edges, 1, 3, 7, 2);
    addEdge(edges, 1, 3, 7, 2);
    addEdge(edges, 3, 2, 8, 4);
    addEdge(edges, 3, 3, 8, 4);

    OrderedEdgeList<TemporalEdge> tgs(4, edges, {1, 12});

    return tgs;
}


tglib::OrderedEdgeList<TemporalEdge> getExample3() {
    std::vector<TemporalEdge> edges;

    addEdge(edges, 0, 1, 1, 1);
    addEdge(edges, 1, 2, 2, 1);
    addEdge(edges, 2, 3, 3, 1);
    addEdge(edges, 1, 3, 4, 1);

    OrderedEdgeList<TemporalEdge> tgs(4, edges, {0, 5});

    return tgs;
}


OrderedEdgeList<TemporalEdge> getBetweennessTest() {
    std::vector<TemporalEdge> edges;

    addEdge(edges, 0, 1, 1, 1);
    addEdge(edges, 1, 2, 2, 1);
    addEdge(edges, 2, 3, 3, 1);
    addEdge(edges, 3, 4, 4, 1);
    addEdge(edges, 3, 5, 4, 1);
    addEdge(edges, 5, 6, 5, 1);

    OrderedEdgeList<TemporalEdge> tgs(7, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}


OrderedEdgeList<TemporalEdge> getNoTopologicalOverlap(int num_edges) {
    std::vector<TemporalEdge> edges;

    for (int i = 0; i < num_edges; i+=2) {
        addEdge(edges, 0, 1, i, 1);
        addEdge(edges, 0, 2, i+1, 1);
    }
    std::sort(edges.begin(), edges.end());
    OrderedEdgeList<TemporalEdge> tgs(3, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}

OrderedEdgeList<TemporalEdge> getFullTopologicalOverlap(int num_edges) {
    std::vector<TemporalEdge> edges;

    for (int i = 0; i < num_edges; ++i) {
        addEdge(edges, 0, 1, i, 1);
    }
    std::sort(edges.begin(), edges.end());

    OrderedEdgeList<TemporalEdge> tgs(2, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}


OrderedEdgeList<TemporalEdge> getCompleteGraph() {
    std::vector<TemporalEdge> edges;

    addEdge(edges, 0, 1, 1, 1);
    addEdge(edges, 0, 2, 1, 1);
    addEdge(edges, 0, 3, 1, 1);
    addEdge(edges, 0, 4, 1, 1);
    addEdge(edges, 1, 0, 1, 1);
    addEdge(edges, 1, 2, 1, 1);
    addEdge(edges, 1, 3, 1, 1);
    addEdge(edges, 1, 4, 1, 1);
    addEdge(edges, 2, 1, 1, 1);
    addEdge(edges, 2, 0, 1, 1);
    addEdge(edges, 2, 3, 1, 1);
    addEdge(edges, 2, 4, 1, 1);
    addEdge(edges, 3, 1, 1, 1);
    addEdge(edges, 3, 2, 1, 1);
    addEdge(edges, 3, 0, 1, 1);
    addEdge(edges, 3, 4, 1, 1);
    addEdge(edges, 4, 1, 1, 1);
    addEdge(edges, 4, 2, 1, 1);
    addEdge(edges, 4, 3, 1, 1);
    addEdge(edges, 4, 0, 1, 1);

    OrderedEdgeList<TemporalEdge> tgs(5, edges, {edges.front().t, edges.back().t + edges.back().tt});

    return tgs;
}

OrderedEdgeList<TemporalEdge> getRandomGraphNonSorted(int nodes, int num_edges) {
    std::set<TemporalEdge> edge_set;
    std::vector<TemporalEdge> edges;

    for (int i = 0; i < num_edges; ++i) {
        NodeId u = rand() % nodes;
        NodeId v = rand() % nodes;
        edges.push_back({u,v,rand()%100, 1+rand()%100});
    }

    for (auto &e : edge_set){
        edges.push_back(e);
    }

    OrderedEdgeList<TemporalEdge> tgs(nodes, edges, {0,0});

    return tgs;
}