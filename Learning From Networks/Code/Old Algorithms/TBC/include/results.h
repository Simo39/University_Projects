#ifndef RESULTS_H
#define RESULTS_H
// Libraries:
// standard C++
#include <fstream>
#include <iostream>
#include <string>
// TGLib 
#include "tglib_cpp/src/core/OrderedEdgeList.h"
#include "tglib_cpp/src/util/TopkResult.h"

template<typename E, typename T> class StoreResults
{
   public:
      // Create the TKResults vector
      std::vector<std::pair<tglib::NodeId, T>> findTopK(std::vector<T> centralities, std::vector<E> edges, int k)
      {
         tglib::TopkResult<T> res(k);
         for(int i=0; i<edges.size(); i++)
	      {
		      // Create the Top K Results vector
            res.insert(edges[i].u,centralities[edges[i].u]);
	      }
	      // Retrive the Top K Results found at the previous step
	      std::vector<std::pair<tglib::NodeId, T>> top_k_results =  res.getResults();
         return top_k_results;
      }
      // Write the results in a txt file given a path
      void writeResult(std::vector<std::pair<tglib::NodeId, T>> top_k, std::string filename)
      {
	      // Create the File
	      try
	      {
            std::ofstream fout(filename);
            for(int i=0; i<top_k.size() ; i++)
            {
	            // Write Node Id and its Centrality
	            fout << top_k[i].first << '\t' << top_k[i].second << '\n';
            }
	         // Close the File
            fout.close();
	      }catch(std::exception& e)
	      {
		      std::cerr << "error: " << e.what() << std::endl;
	      } 
      }
};

#endif // RESULTS_H