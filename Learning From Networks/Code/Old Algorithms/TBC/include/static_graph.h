#ifndef STATIC_GRAPH_H
#define STATIC_GRAPH_H

// Libraries:
// standard C++
#include <map>
#include <set>
#include <vector>

// TGLib
#include "tglib_cpp/src/core/TRSGraph.h"

template<typename N> class StaticGraph{
    public:
        StaticGraph(){}
        
        void addEdge(N tail, N head){
            // Insert the nodes into the node set
            nodes.insert(tail);
            nodes.insert(head);
            // Check if the tail node is already presented into the
            // adjacency list as source node
            auto search_tail = adj_list.find(tail);
            if(search_tail == adj_list.end()){
                adj_list.insert({tail, std::set<N>()});
            }
            // Do the same thing for the head node
            auto search_head = adj_list.find(head);
            if(search_head == adj_list.end()){
                adj_list.insert({head, std::set<N>()});
            }
            // Add the edge to the graph
            adj_list[tail].insert(head);

            // Fill the transpose list

            // Check if the head node is already presented into the
            // transpose list as source node
            auto search_trans_head = transpose_list.find(head);
            if(search_trans_head == transpose_list.end()){
                transpose_list.insert({head, std::set<N>()});
            }
            // Do the same thing for the tail node
            auto search_trans_tail = transpose_list.find(tail);
            if(search_trans_tail == transpose_list.end()){
                transpose_list.insert({tail, std::set<N>()});
            }
            // Add the edge into the traspose list
            transpose_list[head].insert(tail);
        }

        void removeNode(N node){
            nodes.erase(node);
            
            auto search = adj_list.find(node);
            if(search == adj_list.end()){
                adj_list.erase(node);
            }
            auto search2 = transpose_list.find(node);
            if(search2 == transpose_list.end()){
                transpose_list.erase(node);
            }

        }

        void clear(){
            nodes.clear();
            adj_list.clear();
            transpose_list.clear();
        }

        std::set<N> getNodes(){
            return nodes;
        }

        std::map<N, std::set<N>> getEdges(){
            return adj_list;
        }

        std::map<N, std::set<N>> getTranspose(){
            return transpose_list;
        }

    private:

        std::set<N> nodes;
        std::map<N, std::set<N>> adj_list;

        // It represents the edges fo the graph, but reversed
        std::map<N, std::set<N>> transpose_list;   
};

#endif // STATIC_GRAPH_H