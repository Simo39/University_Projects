# Learning_from_Network_Project : 
## A Survey on the Different Approaches to Temporal Node Centralities

Group's Members :
- Bruno Principe
- Raazia Tariq
- Simone Boscolo

The aim of this Project was to make a Survey of all the existent methods applied in the task of the Temporal Node Centrality in Temporal Network alias Dynamic Graph. Until now there were not a great Literature and with a consequence of an unexplored branch in the Graph Theory, the only few attemps in this Problem Resolution were not enough and suffers of : 
- Lack of scalability both in term of Memory that in term of Time (Node Centrality is a well know problem in the Theory side and also in the Static form is #P-Hard in some cases only on few limited cases is obtained the complexity equal to the Polynomial time)
- Real World Network are Dynamics and does not have a sole definition of the walk. 
- Real World Network are frequently huge and touch millions or more nodes.
- No one provide a worldwide standard implementation of this particular type of Graph, neither Libraries of Graph support it. (Currently there are some sperimental and unknow Libraries just for small and mid-graph Teneto, Dynext just few name, and only 1 for very Large Graph TGLib the most recent).
To wrap up words the entire works is a reimplementation of these few existent methods and to make them running in the current standard of Datasets shared among the Web,  that uses the current version of the languages and makes the statistics on them based on Kendall Rank, Mean Absolute Error and Maximum Deviation. 
Algorithm analyzed : 
- ONBRA Santoro, (2022) Used the official with some additional functions to Retrieve the TopK nodes Centralities (C++)
- Temoral Walk Centrality  Oettershagen, (2022) Used a costum version from the Library TGLib (C++)
- Temporal node centrality in complex networks Kim, (2012) Re-adapted in the current Python 3.10 with Dynext (Python)
- Towards Classifying the Polynomial-Time Solvability of Temporal Betweenness Centrality Rymar, (2021) Created from Scratch throught TGLib (C++) 
- Temporal betweenness centrality in dynamic graphs Tsalouchidou1, (2020) Re-Adapted with the Current Python 3.10, and with added a way to read and process the graph with Dynext

## Run the Experiment : 
In the folder of the Repository are included most of the requirements but not all, for the C++ was used several distro of Linux in general Ubuntu 22.04 LTS or whatever distros that has :
- gcc and g++ greater or equal to 11.02, 
- Cmake greater or equal to 3.22
- Boost Library (Better if installed)
- Eigen 3 Library (Better if Installed)
- TGLib already included in the Repository Folder
In reverse for Python is executable in all the Current Operating System and only needs the installation of the Requirement.txt
Then as next step for the C++ Algorithm just build it with cmake
E.G. 
1. move the TGLib folder inside Old_Algorithm folder then, 
2. cd TBC 
3. mkdir build
4. cd build
5. cmake ..
6. make

And finally run the executable : 
- TBC :  ./tbc --path_file="/pathTo/TGraph.txt" --save_results=true--name_file_results="/pathTo/Results.txt" --cost_function="shortest_fastest_TW”
- ONBRA : ./onbra  f “pathTo\CollegeMsg.txt” -d -s -E 1 -S 1000 -I 10 –name_file_results=”/pathTo/CollegeMsg_Results.txt”
- TWC :  ./twc --path_file=\"pathTo/CollegeMsg.txt\" –alpha=0.1  –beta=0.2  --save_results=true --name_file_results="/pathTo/CollegeMsg_Results.txt"

With the Python counterpart use instead  :
- pip install -r requirements.txt
then :
- TBCDG : python spark_tvbc.py 500 0.3 "pathTo/CollegeMsg.txt" "pathTo/CollegeMsg_Results.txt" 10
- TNCCN : python main.py "pathTo/CollegeMsg.txt" "pathTo/CollegeMsg_Results.txt" 10
## Datasets : 
Retrieved from the Snap Dataset Database : http://snap.stanford.edu/temporal-motifs/
This is the following List used : 
- sx-stackoverflow
- sx-mathoverflow
- sx-superuser
- sx-askubuntu
- wiki-talk-temporal
- email-Eu-core-temporal
- CollegeMsg
- FBWall ( We used a fixed version due to an additional ghosty symbols that makes impossible the correct reading of the Temporal Network)
- SMS-A

And 2 toy dataset from : https://networkrepository.com/dynamic.php
- reptilia-tortoise-network-sl
- insecta-ant-trophallaxis-colony1

Download all of these Datasets from the following links : https://drive.google.com/file/d/1VviBH2hk91MO4H0di0iqm0RgXVUzJ0Wy/view?usp=sharing
### Some Information regarding the whole expreriments
Either for small datasets some algorithms takes from minutes to days, for mid to very Large Datasets all ,but not TWC, are able to compute the Temporal Node Centrality with only 16 GB of Ram meaning that is suggested the usage of a machine of at least 32 GB if 64 GB or more is preferred.
Due the High computation of some of them greater is the number of Cores\Threads of the processor better is.
With the next List we state that : 
- TWC (Stream Approximation) is the fastest algorithm, with some tuning could reach the computation of the largest dataset sx-stackoverflow in just 1 Day or less in a Laptop of 16Gb of RAM and an Intel i7 of 6th generation without any effort.
- ONBRA (Approximation)  quite heavy and without a good setting it takes hours for the smallest but also 12 Hours to Days until reached the mid Dataset, failing to compute it for wiki-talk-temporal, sx-stackoverflow. Heavy means that uses all the memory of the machine cited in the previous line,  making the system kill the process for larg datasets.
- TBC (Exact) inefficient in memory and time, only CollegeMsg was able to be computed in 2 days consuming all the 16GB of Ram of the Laptop, the others datasets executions was killed by the Excess of memory usage also know in the literature as OoM (Out of Memory) Error.
- TBCDG (Distributed Approximation) : Inefficient in Time but perfectly fit in memory, just for run the smallest Network CollegeMsg takes more than 2 weeks or more for the Laptop with the requirements listed above but only 5 days and half for a Desktop with a processor Intel i5 of 13th generation with 14 Cores 5Ghz a good Cooling System is required in the Desktop case there was a mixture of Liquid and Fans to mantain the temperature at 45°C, something that the Laptop instead does not have, thus reaching 85°C during the entire execution, **Warning** these high temperature for so much longer time could ruin the hardware of a machine with a not appropriate Cooling System.
- TNCCN (Exact) : Not scale for Mid to Large Network, is inefficient in Memory. With the current equipment was able to run only CollegeMsg.txt without a OoM Exception.

## Results :
In the Folder Results are visible the TopK Central Nodes found from the different approaches and Datasets.
