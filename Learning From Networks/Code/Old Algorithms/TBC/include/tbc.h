#ifndef BC_COMPUTE_TBC_H
#define BC_COMPUTE_TBC_H
// Libraries:

// Standard C++
#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

// TGLib 
#include "tglib_cpp/src/util/InputOutput.h"
#include "tglib_cpp/src/core/OrderedEdgeList.h"
#include "tglib_cpp/src/core/TRSGraph.h"
#include "tglib_cpp/src/util/UtilFunctions.h"

// Our library
#include "static_graph.h"

template<typename E> class TemporalBetweennessCetrality
{
    public:
    
        /*        
            +++++++++++++++++++++
            | CLASS CONSTRUCTOR |
            +++++++++++++++++++++
        */

        TemporalBetweennessCetrality(tglib::OrderedEdgeList<E> const &TEMPORAL_GRAPH_, int COST_FUNCTION ,int delta = -1) : 
            TEMPORAL_GRAPH  ( TEMPORAL_GRAPH_ ),                            // USELESS
            EDGES           ( TEMPORAL_GRAPH.getEdges() ),
            N               ( TEMPORAL_GRAPH.getNumberOfNodes() ),
            TI_FIRST        ( TEMPORAL_GRAPH.getTimeInterval().first ),
            TI_SECOND       ( TEMPORAL_GRAPH.getTimeInterval().second ),    // USELESS
            T               ( this->TI_SECOND - this->TI_FIRST ),           // USELESS
            COST_FUNCTION   ( COST_FUNCTION),
            delta           ( delta )
        {   
            // Initialize:

            // 1 - Vertices Apperence (or Time-Nodes) set
            initializeVerticesApperenceSet();

            // 2 - Get the time-respecting static graph of the given temporal graph
            transformToStaticGraph();

            // 7 - Betweenness Centrality Scores vector
            betweeness.resize(N, 1);
            //-----------------------------------------------------------------------

            // Compute the Predecessor Graph for each node in V as source node
            for(tglib::NodeId n_id = 0; n_id < N; n_id++)
            {   
                computePredecessorGraph(n_id);
                // Compute the Betweenness Centrality cost for each node of the graph
                generalBetweenness(n_id);
                // Clear the predecessor graph for the next use
                predecessor_graph.clear();
            }

            
        }

        /*        
            ++++++++++++++++++++++++++++++++++++++
            | GET FUNCTION FOR CENTRALITY SCORES |
            ++++++++++++++++++++++++++++++++++++++
        */
        
        std::vector<double> getCentralityBetweennessScores(){
            return betweeness;
        }

    private:

        /*        
            +++++++++++++++++++++++++++++++++++++++++++
            | INITIALIZATION FUNCTION FOR CONSTRUCTOR |
            +++++++++++++++++++++++++++++++++++++++++++
        */

        void initializeVerticesApperenceSet(){
            // Create the vertices apperence set (as in the paper)
            for(const tglib::TemporalEdge &edge : EDGES){
                tglib::NodeId id_v = edge.v;
                auto search = VerticesApperence_SET.find(id_v);
                if(search != VerticesApperence_SET.end()){
                    VerticesApperence_SET.insert({id_v, std::set<tglib::Time>()});
                }
                VerticesApperence_SET[id_v].insert(edge.t);
            }

            // Assigning a timestamp t to the nodes which haven't any predecessor
            // and therefore they are not present yet in the vertices apperence set
            // NOTE: T = {TI_FIRST, TI_FIRST + 1,..., TI_SECOND - 1}, then t = TI_FIRST - 1
            for(tglib::NodeId n_id = 0; n_id < N; n_id++){
                auto search = VerticesApperence_SET.find(n_id);
                //if n_id is not in VerticesApperence_SET
                if(search == VerticesApperence_SET.end()){
                    std::set<tglib::Time> temp;
                    temp.insert(TI_FIRST - 1);
                    VerticesApperence_SET.insert({n_id, temp});
                }
            }

            // Create the bijective function: tglib::TimeNode v_t -> int index
            // to save memory
            // For each v_T is associated an index
            int index = 0;
            for(const auto &[node_id, timestamps] : VerticesApperence_SET){
                for(tglib::Time t : timestamps){
                    tglib::TimeNode tn{node_id, t};
                    TN_TO_IDX_MAP.insert({tn, index});
                    IDX_TO_TN_MAP.insert({index, tn});
                    index++;
                }
            }

            VerticesApperenceSet_SIZE = index;
        }
        
        void transformToStaticGraph(){
            /*
                For each temporal edge (v, w, t) we can have one or more directed edges in a static graph
                such that, each static edge is defined as follow:
                    - head node:  (w,t')
                    - head node:  (v,t), with t <= t' and (v,t) belongs to the VerticesApperence_SET 
            */

            for(const E &edge : EDGES){
                tglib::TimeNode w_t{edge.v, edge.t};
                int wt_idx = getIndex(w_t);
                for(const tglib::Time &t : VerticesApperence_SET[edge.u]){
                    if(t <= edge.t){
                        tglib::TimeNode v_t{edge.u, t};
                        int vt_idx = getIndex(v_t);
                        STATIC_GRAPH.addEdge(vt_idx, wt_idx);
                    }
                }
            }

        }
        
        /*        
            +++++++++++++++++++++++++++++++++
            | PREDECESSOR GRAPH COMPUTATION |
            +++++++++++++++++++++++++++++++++
        */

        void computePredecessorGraph(tglib::NodeId source_node)
        {

            //================
            // INITIALIZATION
            //================

            initializeDijkstra(source_node);

            //==========================
            // BELLMAN-FORD GENERALIZED
            //==========================

            Dijkstra();

            //==================================
            // PREDECESSOR GRAPH RECONSTRUCTION
            //==================================

            for(int node = 0; node < N; node++){
                int best_tn = best_cost_tn[node];
                if(best_tn == -1){
                    continue;
                }
                buildOptimalPredecessorGraph(best_tn, source_node, predecessor_graph);
            }
        }

        /*        
            ++++++++++++
            | DIJSKTRA |
            ++++++++++++
        */

        void initializeDijkstra(tglib::NodeId source_node){
            // Set all costs to infinite
            costs.resize(VerticesApperenceSet_SIZE, INF);
            best_costs.resize(N, INF);
            // Set to "void" the TimeNode with the best cost
            best_cost_tn.resize(N, -1);
            // Initialize the predecessors_list to empty
            predecessors_list.resize(VerticesApperenceSet_SIZE, std::vector<int>());

            // Push all the Time-Nodes into the priority queue
            for(int tn_idx = 0; tn_idx < VerticesApperenceSet_SIZE; tn_idx++){
                // If it is a source time-node
                if(source_node == getTimeNode(tn_idx).first){
                    // Set cost to zero
                    costs[tn_idx] = 0;
                }
                // Push the source node into the priority queue
                priority_queue.push(cost_idx_pair{costs[tn_idx], tn_idx});
            }
            best_costs[source_node] = 0;                      
        }

        void Dijkstra(){
            std::map<int, std::set<int>> static_edges = STATIC_GRAPH.getEdges();

            while(!priority_queue.empty()){
                // Retrieve the node with the smallest cost from the priority queue
                // v_id = ID of node v
                int v_id = priority_queue.top().second;
                // Delete this node from the queue
                priority_queue.pop();

                for(tglib::Time t : VerticesApperence_SET[v_id]){

                    // Candidate previous node v_t and corresponding index vt_idx
                    tglib::TimeNode v_t{v_id, t};                    
                    int vt_idx = getIndex(v_t);

                    // In the correspondig static graph,
                    // for each outgoing edge from v_t, consider the landing node w_t
                    for(int wt_idx : static_edges[vt_idx]){

                        // Corresponding tn of index wt_idx and it's ID w_id
                        tglib::TimeNode w_t = getTimeNode(wt_idx);
                        int w_id = w_t.first;

                        // If the v_t appears later in the time wrt w_t, pass to the next outgoing edge
                        if(w_t.second < t){
                            continue;
                        }
                        
                        if(costs[wt_idx] > costs[vt_idx] + getCost(v_t, w_t)){
                            // Clear the previuous predecessors set of w_t
                            if(!predecessors_list[wt_idx].empty()){
                                predecessors_list[wt_idx].clear();
                            }
                            
                            // Update the costs of node_id
                            costs[wt_idx] = costs[vt_idx] + getCost(v_t, w_t);

                            // Add to the priority_queue the pair (cost, w_id); w_id = ID of node w
                            priority_queue.push(cost_idx_pair{costs[wt_idx], wt_idx});
                            
                        }

                        // Add as predecessor of TimeNode w_t, the TimeNode v_t
                        // that has the same cost 
                        if(costs[wt_idx] == costs[vt_idx] + getCost(v_t, w_t)){                            
                            predecessors_list[wt_idx].push_back(vt_idx);
                        }

                        // Update best_cost and its relative TimeNode
                        if(best_costs[w_id] > costs[wt_idx]){
                            best_costs[w_id] = costs[wt_idx];
                            best_cost_tn[w_id] = wt_idx;
                        }
                    }
                }
            }

            // Cleaning the predecessors lists from duplicates
            for(int tn_idx = 0; tn_idx < predecessors_list.size(); tn_idx++){
                const auto& begin = predecessors_list[tn_idx].begin();
                const auto& end = predecessors_list[tn_idx].end();
                std::sort(begin, end);
                predecessors_list[tn_idx].erase(std::unique(begin, end), end);
            }            
        }
        
        // Version that uses indices as representations of a time nodes
        void buildOptimalPredecessorGraph(int head_idx, tglib::NodeId source_node,StaticGraph<int> &static_graph){

            /*
                DFS approach to create the optimal predecessor 
                static graph wrt to the chosen cost function and 
                a specific source node
            */

            // For each predecessor of head_idx, if any
            while(!predecessors_list[head_idx].empty()){
                // Get a tail from the predecessor list of head_idx
                int tail_idx = predecessors_list[head_idx].back();
                // Erase tail node from the list
                predecessors_list[head_idx].pop_back();
                // Add edge tail->head
                static_graph.addEdge(tail_idx, head_idx);
                // If source node, not deeper researches are needed for this node
                if(source_node == getTimeNode(tail_idx).first){
                    continue;
                }
                // Look for predecessors of tail and link them ot it
                buildOptimalPredecessorGraph(tail_idx, source_node, static_graph);
            }
        }

        /*        
            +++++++++++++++++++++++
            | GENERAL BETWEENNESS |
            +++++++++++++++++++++++
        */

       void generalBetweenness(tglib::NodeId source_node) {
            // Initialize the number of temporal walks from source node to all the other nodes of the graph
            countWalk(source_node);

            // Initialize all the temporal cumulative dependecies to 0
            // Line 5-6 of Algorithm 3 from the paper
            std::vector<double> temporal_cumulative_dependecies(VerticesApperenceSet_SIZE, 0);

            // Sort the transpose map from the node with less successors, to the node with more successors
            std::map<int, std::set<int>> edges = predecessor_graph.getEdges();
            std::vector<std::pair<int, std::set<int>>> temp(edges.begin(), edges.end());
            std::sort(temp.begin(), temp.end(), 
                [](const std::pair<int, std::set<int>>& a, const std::pair<int, std::set<int>>& b){
                    return a.second.size() < b.second.size();
                });
            // Get the predecessors of each node
            std::map<int, std::set<int>> transpose = predecessor_graph.getTranspose();

            // Line 7 of Algorithm 3 from the paper: it has to start the loop from the node with less successors
            // edges has been sorted earlier, so it contains the all the verteces sorted by ascending their successors set size
            for(const auto &[head_idx, NOT_USED_VARIABLE] : edges){
                tglib::NodeId head_id = getTimeNode(head_idx).first;
                // Line 8 of Algorithm 3 from the paper
                // Check if the node is reachable, otherwise leave the cumulative dependecies to zero
                if(temporal_pair_dependecies[head_id][head_idx] != INF){
                    temporal_cumulative_dependecies[head_idx] += temporal_pair_dependecies[head_id][head_idx];
                }
                std::set<int> predecessors_set = transpose[head_idx];
                // Line 9 of Algorithm 3 from the paper
                for(int tail_idx : predecessors_set){
                    tglib::NodeId tail_id = getTimeNode(tail_idx).first;
                    double fraction_sig_vt_by_sig_wt = static_cast<double>(sigma_s_v[tail_id])/sigma_s_v[head_id];
                    // Check if the fraction has a valid value, otherwise set it to zero
                    if(std::isnan(fraction_sig_vt_by_sig_wt) || std::isinf(fraction_sig_vt_by_sig_wt)) fraction_sig_vt_by_sig_wt = 0;
                    // Line 10 of Algorithm 3 from the paper
                    temporal_cumulative_dependecies[tail_idx] += fraction_sig_vt_by_sig_wt * temporal_cumulative_dependecies[head_idx];
                    // Line 11 of Algorithm 3 from the paper
                    betweeness[source_node] += fraction_sig_vt_by_sig_wt * temporal_cumulative_dependecies[head_idx];
                }
            }
            // Line 12 of Algorithm 3 from the paper 
            for(int tn_idx : predecessor_graph.getNodes()){
                tglib::NodeId tn_id = getTimeNode(tn_idx).first;
                if(temporal_pair_dependecies[tn_id][tn_idx] > 0){
                    betweeness[source_node] -= 1;
                }
            }
        }

        void countWalk(tglib::NodeId source_node){
            // Initialize:
            // 1 - Number of temporal walks from node source_node to node v
            sigma_s_v.resize(N, 0);
            // 2 - Number of temporal walks from source_node to vertex apperence v_t
            sigma_s_vt.resize(VerticesApperenceSet_SIZE, 0);
            // 3 - Temporal pair dependecies on source_node and z for each v_t
            temporal_pair_dependecies.resize(N, std::vector<double>(VerticesApperenceSet_SIZE, 0));

            // Compute the SCC by using Kosaraju's algorithm
            std::vector<std::vector<int>> SCCs = Kosaraju(source_node);

            // For each SCC, set the costs of its element to inifity and its sigma
            //CHECK WITH BRUNO THAT THERE ARE NO NEED OF USING BFS
            for(std::vector<int> &SCC: SCCs){
                if(SCC.size() <= 1){
                    continue;
                }
                for(int &tn_idx : SCC){
                    costs[tn_idx] = INF;
                    sigma_s_vt[tn_idx] = INF;
                    predecessor_graph.removeNode(tn_idx);
                }
            }

            // Now, the predecessor graph is a DAG

            std::map<int, std::set<int>> edges = predecessor_graph.getEdges();

            // For each vt, count the number of paths from the source node
            // Iterative BFS approach
            std::queue<int> BFS_queue;
            std::set<int> visited;
            // Push into the queue all the source_node apperences
            for(tglib::Time t : VerticesApperence_SET[source_node]){
                tglib::TimeNode source_tn{source_node, t};
                BFS_queue.push(getIndex(source_tn));
            }
            while(!BFS_queue.empty()){
                int tail = BFS_queue.front();
                BFS_queue.pop();
                auto search = visited.find(tail);
                // If the node tail has been already visited, we pass to the next node
                if(search != visited.end()){
                    continue;
                }
                // Set the node as visited
                visited.insert(tail);
                // For each landing node of each edge starting from tail
                for(int head : edges[tail]){
                    // Increment the number of path from source node to v_t
                    sigma_s_vt[head]++;
                    // Push the v_t node into the queue
                    BFS_queue.push(head);
                }
            }

            // For each node, compute the number of temporal walks that start from
            // source node s and ends on node v
            for(int tn_idx : predecessor_graph.getNodes()){
                int n_id = getTimeNode(tn_idx).first;
                // If the cost to arrive to vt is equals to the best cost find during Dijkstra,
                // it means that vt is one of the best time nodes to reach v
                if(costs[tn_idx] == best_costs[n_id]){
                    sigma_s_v[n_id] += sigma_s_vt[tn_idx]; 
                }
            }

            // Here it is computed the Temporal pair dependecies on source_node and v for each v_t
            for(int tn_idx = 0; tn_idx < VerticesApperenceSet_SIZE; tn_idx++){
                int node_id = getTimeNode(tn_idx).first;
                // Get the number of path from s to v
                int sigma_s_node = sigma_s_v[node_id];
                // If there is no path between s and v
                if(sigma_s_node == 0 || sigma_s_node == INF){
                    // Set the temporal pair dependecies of s,v,(v,t) to zero
                    temporal_pair_dependecies[node_id][tn_idx] = 0;
                }
                else{
                    // Otherwise, set the temporal pair dependecies of s,v,(v,t) to 
                    // ( #paths from s to (v,t) ) / ( #paths from s to v )
                    int sigma_s_timenode = sigma_s_vt[tn_idx];
                    double delta = static_cast<double>(sigma_s_timenode)/sigma_s_node;
                    temporal_pair_dependecies[node_id][tn_idx] = delta;
                }
            }
        }
        
        /*
            ++++++++++++++++
            | Kosaraju SCC |
            ++++++++++++++++
        */

        // Compute the Strongly Connected Components with Kosaraju's Algorithm
        // TO CHECK IF EVERYTHING IS WORKING WELL
        std::vector<std::vector<int>> Kosaraju(tglib::NodeId source_node)
        {
            // Final vector of Strongly Connected Components for all the nodes
            std::vector<std::vector<int>> scc;
            std::stack<int> stack;
            // Mark all the vertices as not visited (For first DFS)
	        std::set<int> visited;
            std::set<int> nodes = predecessor_graph.getNodes();
            std::map<int, std::set<int>> edges = predecessor_graph.getEdges();
            // Fill vertices in stack according to their finishing times
            for(int tn_idx : nodes){
                auto search = visited.find(tn_idx);
                if(search == visited.end()){
                    fillOrder(tn_idx, visited, stack, edges);
                }
            }
            // Create a reversed graph
            std::map<int, std::set<int>> transpose_list = predecessor_graph.getTranspose();
            // Mark all the vertices as not visited (For second DFS)
            visited.clear();
            // Now process all vertices in order defined by Stack
            while (stack.empty() == false)
            {
                // Pop a vertex from stack
                int v = stack.top();
                stack.pop();
		        std::vector<int> cc;
                // Print Strongly Connected Components of the popped vertex
                auto search = visited.find(v);
                if (search == visited.end())
                {
                    DFSUtil(v, visited, transpose_list, cc);
			        // Update scc
			        scc.push_back(cc);
                }
            }
            return scc;
        }

        void fillOrder(int v, std::set<int> &visited, std::stack<int> &stack, std::map<int, std::set<int>> &edges)
        {
            // Mark the current node as visited and print it
            visited.insert(v);
            // Recur for all the vertices adjacent to this vertex
            for(int tn_idx : edges[v]){
                auto search = visited.find(tn_idx);
                if(search == visited.end()){
                    fillOrder(tn_idx, visited, stack, edges);
                }
            }
            // All vertices reachable from v are processed by now, push v
            stack.push(v);
        }

        // A recursive function save Strongly Connected Components
        void DFSUtil(int v, std::set<int> &visited, std::map<int, std::set<int>> &edges, std::vector<int> &scc)
        {
            // Mark the current node as visited and print it
            visited.insert(v);
            scc.push_back(v);
            // Recur for all the vertices adjacent to this vertex
            for(int tn_idx : edges[v]){
                auto search = visited.find(tn_idx);
                if(search == visited.end()){
                    DFSUtil(tn_idx, visited, edges, scc);
                }
            }
        }

        /*        
            ++++++++++++++++++
            | COST FUNCTIONS |
            ++++++++++++++++++
        */

        // Method that implements the chosen cost function
        double getCost(tglib::TimeNode v_t, tglib::TimeNode w_t){
            if(COST_FUNCTION == 0)  return fastestTW(v_t, w_t);
            if(COST_FUNCTION == 1)  return shortestTW(v_t, w_t);
            if(COST_FUNCTION == 2)  return foremostTW(v_t, w_t);
            if(COST_FUNCTION == 3)  return shortestFastestTW(v_t, w_t);
            else  return shortestRestlessTW(v_t, w_t);
        }

        // Implementation of the Fastest Temporal Walk Cost Function
        double fastestTW(tglib::TimeNode v_t, tglib::TimeNode w_t){
            return w_t.second - v_t.second;
        }

        // Implementation of the Shortest Temporal Walk Cost Function
        double shortestTW(tglib::TimeNode v_t, tglib::TimeNode w_t){
            return 1;
        }

        // Implementation of the Foremost Temporal Walk Cost Function
        double foremostTW(tglib::TimeNode v_t, tglib::TimeNode w_t){
            return w_t.second;
        }

        // Implementation of the Shortest Fastest Temporal Walk Cost Function 
        double shortestFastestTW(tglib::TimeNode v_t, tglib::TimeNode w_t)
        {
            return (w_t.second - v_t.second) + 1;
        }

        // Implementation of the Shortest Restless Temporal Walks Cost Function 
        double shortestRestlessTW(tglib::TimeNode v_t, tglib::TimeNode w_t)
        {
            if((w_t.second - v_t.second) < delta)   return 1;
            else                                    return INF;
        }

        /*        
            +++++++++++++++++++++++++++++++++++++++++
            | MAPPING FUNCTION: INDEX <-> TIME NODE |
            +++++++++++++++++++++++++++++++++++++++++
        */

        // MAP: Time Node -> Index
        int getIndex(tglib::TimeNode tn){
            return TN_TO_IDX_MAP[tn];
        }

        // MAP: Index -> Time Node
        tglib::TimeNode getTimeNode(int index){
            return IDX_TO_TN_MAP[index];
        }

        /*        
            +++++++++++++++++
            | CLASS MEMBERS |
            +++++++++++++++++
        */
        
        //=====================
        // TEMPORAL GRAPH DATA
        //=====================

        // Temporal Graph
        tglib::OrderedEdgeList< E > const TEMPORAL_GRAPH;
        // Vector with all the edges of TEMPORAL_GRAPH
        const std::vector<E> EDGES;
        // Number of nodes in TEMPORAL_GRAPH
        const size_t N;
        // Earliest timestamp of TEMPORAL_GRAPH
        const tglib::Time TI_FIRST;
        // Latest timestamp of TEMPORAL_GRAPH + 1
        const tglib::Time TI_SECOND;
        // Period time of TEMPORAL_GRAPH = TI_SECOND - TI_FIRST
        const tglib::Time T;
        // Flag variable to choose the cost function;
        const int COST_FUNCTION;

        //=====================
        //VERTEX APPERENCE SET
        //=====================

        // Nodes apperence set of TEMPORAL_GRAPH: each node has all its apperences
        std::map<tglib::NodeId, std::set<tglib::Time>> VerticesApperence_SET;
        int VerticesApperenceSet_SIZE;

        // Bijective function: TimeNode tn <-> int index
        // Used for the predecessors data sets and the costs array
        std::map<tglib::TimeNode, int> TN_TO_IDX_MAP;
        std::map<int, tglib::TimeNode> IDX_TO_TN_MAP;

        //================================
        // PREDECESSORS STATIC GRAPH DATA
        //================================
        
        // List of predecessors graphs for each node in G as source node
        StaticGraph<int> predecessor_graph;

        //======================================
        // SUPPORT DATA STRUCTURES FOR DIJSKTRA
        //======================================

        // Corresponding static graph of TEMPORAL_GRAPH, where:
        //  - each node is a vertex apperence (v,t)
        //  - (v,t) -> (w,t') iff it exists: 
        //          . edge e=(v,w,t')
        //          . t <= t'
        StaticGraph<int> STATIC_GRAPH;

        // Priority Queue: lower cost has higher priority
        typedef std::pair<double, int> cost_idx_pair;
        std::priority_queue<cost_idx_pair, std::vector<cost_idx_pair>, std::greater<cost_idx_pair>> priority_queue;

        // List of predecessors sets, one set for each time node
        std::vector<std::vector<int>> predecessors_list;

        // COST FUNCTION

        // For each predecessor_graph (then, for each possible source node)

        // Costs array for each node apperence
        std::vector<double> costs;
        // Best cost for each node
        std::vector<double> best_costs;
        // Corresponding index to the TimeNode with the best cost
        std::vector<int> best_cost_tn;
        
        // Delta boundary of the Shortest Restless Temporal Walks Cost
        int delta = -1;

        //============
        // BETWEENESS
        //============

        // Vector of the Betweeness Centrality scores of all the nodes
        std::vector<double> betweeness;
        // Number of temporal walks from node s to node v
        std::vector<int> sigma_s_v;
        // Number of temporal walks from s to vertex apperence v_t
        std::vector<int> sigma_s_vt;
        // Temporal pair dependecies on s and z for each v_t
        std::vector<std::vector<double>> temporal_pair_dependecies;
        
        //=======================
        // SUPPORT CLASS MEMBERS
        //=======================

        // INFINITY set to the max number provided by an int variable
        const int INF = std::numeric_limits<int>::max();
};

#endif // BC_COMPUTETBC_H
