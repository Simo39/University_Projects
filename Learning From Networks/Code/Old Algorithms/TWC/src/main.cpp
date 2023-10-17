/*

    Author : Bruno Principe, Raazia Tariq and Simone Boscolo

*/
// Libraries :
// C++ Libraries :
#include <filesystem>
// Boost :
#include <boost/program_options.hpp>
// Customized :
#include "results.h"
namespace po = boost::program_options;
int main(int argc, char *argv[]) 
{
    try 
    {
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "Example of usage main --path_file ")
            ("path_file", po::value<std::string>(), "File where is stored the Temporal Graph")
            ("alpha", po::value<double>(), "alpha value for TWC")
            ("beta", po::value<double>(), "beta value for TWC")
            ("save_results", po::value<bool>(), "Is true if is prefered to Save the TopKResults in a *.txt otherwise false")
            ("name_file_results", po::value<std::string>(), "Name with the path of a *.txt File where to save the TopKResults");
        po::variables_map vm;        
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
        tglib::OrderedEdgeList<tglib::TemporalEdge> tg_edl;    
        // Controls of the args given in input
        // This will prints the instruction to make run the program 
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        // Check if the file to retrieve the Temporal Path exists otherwise exit
        if (!vm.count("path_file"))
        {
            std::cout << "Insert a valid *.txt File of the Temporal Graph." << std::endl;
            std::cout << "Terminal Commands Example : \n./twc --path_file=\"pathTo/CollegeMsg.txt\" -–alpha=0.1 -–beta=0.2 --save_results=true --name_file_results=\"/pathTo/CollegeMsg_Results.txt\" "<< std::endl;
            return 1;
        }
        // 1- Create the Temporal Graph from the txt file as a Static Direct Graph  
        std::cout << "1- Load the Graph from the file." << std::endl;
        std::string path = vm["path_file"].as<std::string>();
        // Check is has the .txt extension otherwise return 1
        if(std::filesystem::path(path).extension() != ".txt")
        {
            std::cout << "Insert a valid *.txt File of the Temporal Graph." << std::endl;
            return 1;
        }
        try
        {
            // Load the Temporal Graph
            tg_edl = tglib::load_ordered_edge_list<tglib::TemporalEdge>(path);
        }catch(std::exception& e)
        {
            std::cerr << "The File does not exists or is not standard. " << std::endl;
            std::cerr << "error: " << e.what() << std::endl;
            return 1;
        }
        // 2- Compute the Temporal Betweeness Centrality
        std::cout << "2- Compute the Temporal Betweeness Centrality." << std::endl;
        double alpha = vm["alpha"].as<double>();
        double beta = vm["beta"].as<double>();
        if(alpha <= 0 || beta <= 0)
        {
            std::cerr << "alpha and beta must be > 0. " << std::endl;
            return 1;
        }
        std::vector<tglib::TemporalEdge> edges = tg_edl.getEdges();
        std::vector<double> betwenness = tglib::temporal_walk_centrality<tglib::TemporalEdge>(tg_edl,alpha,beta);
        // If save_results is True and the file to save is Valid (*.txt) then proceed to compute the TopkResults and write in the mentioned file
        if (vm.count("save_results") && vm["save_results"].as<bool>() == true && betwenness.empty() != true)
        {
            // For Debug
            //std::cout << "Save the TopKResults : " << vm["save_results"].as<bool>() << std::endl;
            if (vm.count("name_file_results"))
            {
                // For Debug
                //std::cout << "File to save the Results : " << vm["name_file_results"].as<std::string>() << std::endl;
                //std::cout << "extension : "<< std::filesystem::path(vm["name_file_results"].as<std::string>()).extension() << std::endl;
                // Assess if the File to Write the results has the extension *.txt
                if(std::filesystem::path(vm["name_file_results"].as<std::string>()).extension() == ".txt")
                {
                    // 3- Retrive the TopK Results from the previous step
                    std::cout << "3- Retrive the TopK Results." << std::endl;
                    StoreResults<tglib::TemporalEdge,double> topk;
                    // Set the Top K elements to collect
                    int k = 10;
                    // Find the TopK Elements
                    std::vector<std::pair<tglib::NodeId, double>> topk_pairs = topk.findTopK(betwenness, edges, k);
                    // 4- Save the results in a txt file for future Statistics
                    std::cout << "4- Save the results in a txt file." << std::endl;
                    topk.writeResult(topk_pairs, vm["name_file_results"].as<std::string>());
                }else
                {
                    std::cout << "Insert a valid *.txt File to Save the TopKResults." << std::endl;
                }
            }
        }
    }
    // If something went wrong throw an Exception and print it in the Terminal
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << std::endl;
        return 1;
    }catch(...) {
        std::cerr << "Exception of unknown type!" << std::endl;
    }
    return 0;
}