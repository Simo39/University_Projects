// Libraries :

// C++ Libraries :
#include <filesystem>

// Boost :
#include <boost/program_options.hpp>

// Customized :
#include "results.h"
#include "tbc.h"

namespace po = boost::program_options;
int main(int argc, char *argv[]) 
{
    try {
    po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "Example of usage main --path_file ")
            ("path_file", po::value<std::string>(), "File where is stored the Temporal Graph")
            ("save_results", po::value<bool>(), "Is true if is prefered to Save the TopKResults in a *.txt otherwise false")
            ("name_file_results", po::value<std::string>(), "Name with the path of a *.txt File where to save the TopKResults")
            ("cost_function", po::value<std::string>(), "Cost functions allowed:\n"
                                                        " - fastest_TW\n"
                                                        " - shortest_TW\n"
                                                        " - foremost_TW\n"
                                                        " - shortest_fastest_TW\n"
                                                        " - shortest_restless_TW\n");
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
            std::cout << "Terminal Commands Example : \n ./bc --path_file=\"TGraph.txt\" --save_results=true --name_file_results=\"Results.txt\" --cost_function=\"shortest_fastest_TW\" "<< std::endl;
            return 1;
        }
        
        // 1- Create the Temporal Graph from the txt file as a Static Direct Graph  
        std::string path = vm["path_file"].as<std::string>();
        //std::cout << "extension : "<< std::filesystem::path(path).extension() << std::endl;
        // Check is has the .txt extension otherwise return 1
        if(std::filesystem::path(path).extension() != ".txt"){
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
        std::vector<tglib::TemporalEdge> edges = tg_edl.getEdges();
        if (!vm.count("cost_function") || (
            vm["cost_function"].as<std::string>() != "fastest_TW" &&
            vm["cost_function"].as<std::string>() != "shortest_TW" &&
            vm["cost_function"].as<std::string>() != "foremost_TW" &&
            vm["cost_function"].as<std::string>() != "shortest_fastest_TW" &&
            vm["cost_function"].as<std::string>() != "shortest_restless_TW"
            ))
        {
            std::cout << "Insert a valid cost function to use for the Temporal Betweeness." << std::endl;
            std::cout << "Terminal Commands Example : \n ./bc --path_file=\"TGraph.txt\" --save_results=true --name_file_results=\"Results.txt\" --cost_function=\"shortest_fastest_TW\" "<< std::endl;
            return 1;
        }

        int function;

        if(vm["cost_function"].as<std::string>() == "fastest_TW")                   function = 0;
        else if(vm["cost_function"].as<std::string>() == "shortest_TW")             function = 1;
        else if(vm["cost_function"].as<std::string>() == "foremost_TW")             function = 2;
        else if(vm["cost_function"].as<std::string>() == "shortest_fastest_TW")     function = 3;
        else                                                                        function = 4;

        int delta = 2;

        TemporalBetweennessCetrality<tglib::TemporalEdge> tbc(tg_edl, function, delta);

        std::vector<double> betwenness = tbc.getCentralityBetweennessScores();

        // If save_results is True and the file to save is Valid (*.txt) then proceed to compute the TopkResults and write in the mentioned file
        if (vm.count("save_results") && !vm.count("name_file_results")  &&
            std::filesystem::path(vm["name_file_results"].as<std::string>()).extension() != ".txt")
        {
            std::cout << "Insert a valid *.txt File to Save the TopKResults." << std::endl;
            return 0; 
        }else if(!vm.count("save_results") || (vm.count("name_file_results")  &&
            std::filesystem::path(vm["name_file_results"].as<std::string>()).extension() != ".txt"))
        {
            std::cout << "Insert a valid *.txt File to Save the TopKResults 2 and set --save_result=true." << std::endl;
            return 0;
        }
        
        // Assess if the File to Write the results has the extension *.txt
        
        // 3- Retrive the TopK Results from the previous step
        StoreResults<tglib::TemporalEdge,double> topk;
        // Set the Top K elements to collect
        int k = 10;
        // Find the TopK Elements
        std::vector<std::pair<tglib::NodeId, double>> topk_pairs = topk.findTopK(betwenness, edges, k);
        // 4- Save the results in a txt file for future Statistics
        topk.writeResult(topk_pairs, vm["name_file_results"].as<std::string>());
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
