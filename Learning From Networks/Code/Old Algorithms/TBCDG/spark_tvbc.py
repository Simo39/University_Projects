import copy
import sys
import math
import brandes
import graph_transformation
import topk
import findspark
import dynetx as dn # Use NetworkX 2.8.8 otherwise throw an exception

from pyspark import SparkConf, SparkContext

findspark.init()
conf = SparkConf()
sc = SparkContext(conf = conf)

def truncate(f, decimals):
    return math.floor(f * 10 ** decimals) / 10 ** decimals

def main():
    decimals = 6
    if len(sys.argv) < 5:
        sys.exit('Usage: python %s #WindowLength #a #filename #resultFilename #k' % sys.argv[0])

    window = int(sys.argv[1])
    print("Window length = ", window)

    a = float(sys.argv[2])
    print("a = ", a)

    filename = sys.argv[3]
    print("filename = ", filename)

    result_file = sys.argv[4]
    print("TopK filename = ", result_file)

    k = int(sys.argv[5])
    print("K = ", k)

    print("1- Open File and Load Dataset")
    tgs = dn.read_snapshots(filename, nodetype=int, timestamptype=int)
    # Retrieve the Timestamps list
    timestamp_l = list(set(tgs.temporal_snapshots_ids()))
    TBC={}
    graph_window = []
    for i in range(window):
        graph_window.append({})
    f = 1  
    for timestamp in timestamp_l:
        #print(" ")
        #print("******************************")
        #print('*******Timestamp =', timestamp, "*********")
        #print("******************************")
        
        dataset = dn.interactions(tgs, t=timestamp)
        # Transform the list of edges into a dictionary of the form {(node_head_id,timestamp) : edges{(node_tail_id,timestamp) : weight}}
        #{(dataset[0][0],timestamp) : {(dataset[0][2],timestamp):0}}
        new_graph_ = dict()
        for i in dataset:
            if i[0] in new_graph_:
                new_graph_[(i[0])][(i[1])] = 0
            else:
                new_graph_[(i[0])] = dict()
                new_graph_[(i[0])][(i[1])] = 0
            if i[1] not in new_graph_:
                new_graph_[(i[1])] = dict()
        new_graph = copy.deepcopy(new_graph_)
        # graph_window is a list of dictionaries
        graph_window = graph_transformation.graph_trsf(graph_window, new_graph, window, timestamp, a)
        # graph is the merged-element graph_window
        graph = {}
        for i in range(window):
            graph.update(copy.deepcopy(graph_window[i]))
        nodes = graph.keys()
        ids = []
        for i in nodes:
            if i[0] not in ids:
                ids.append(i[0])
        betweenness={}
        for i in nodes:
            betweenness[i] = 0.0
        for i in ids:
            if i not in TBC:
                TBC[i] = 0.0
        n = list(nodes)
        g = copy.deepcopy(graph)
        #BEGIN DISTRIBUTED VERSION
        distids = sc.parallelize(ids)
        result1 = distids.map(lambda x:brandes.function4(x, window, n, g, timestamp))
        betweenness = result1.reduce(lambda x, y: brandes.sumation_dictionaries(x,y))
        #END DISTRIBUTED VERSION
        for i in betweenness:
            TBC[i[0]]+= betweenness[i]
        for i in TBC:
            TBC[i] = truncate(TBC[i], decimals)
    print("4- Compute TopK")
    # Compute the Top K Outcomes
    tbc_k = topk.topk_results(TBC,k)
    # Store them in a file
    print("5- Write the TopK Results in a File .txt")
    topk.write_on_file(tbc_k,result_file)
    return TBC

if __name__ == "__main__":
    main()

