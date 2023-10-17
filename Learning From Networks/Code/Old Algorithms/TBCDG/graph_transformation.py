def graph_trsf(graph_window,new_graph, w, t, a):

    for i,k in list(new_graph.items()):#node renaming and edge weight to new value
        for j in list(k.keys()):
            k.pop(j)
            k[(j,t)]= a
        new_graph[(i, t)] = new_graph.pop(i)

    graph_window.pop(0)
    graph_window.append(new_graph)

    #create the transitions between nodes with the same id and different timestamps/
    for i in list(graph_window[w-1].keys()):
        finish = 0
        k = 1
        while not finish and k<=(w-1):

            if (i[0],t-k) in graph_window[(w-1)-k].keys():
                graph_window[(w - 1) - k][i[0], t - k].update({(i[0],t): (1-a)*k})
                finish = 1
            k+=1
    return graph_window