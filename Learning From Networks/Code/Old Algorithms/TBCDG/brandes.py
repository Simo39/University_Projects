import copy
import dijkstra_paths

def brandes_algo(betweenness,S, P, sigma, sigma_prime,s):
    delta={}
    for i in sigma:
        delta[i] = 0.0
    while S:
        w = S.pop()
        coeff = (sigma_prime[w]/sigma[w]+delta[w])  /sigma[w]
        for v in P[w]:
            if v[1]!=-1:
                delta[v] += sigma[v] * coeff
        if w[0] != s:
            betweenness[w] += delta[w]

    return betweenness

def function4(node, window, nodes, graph, timestamp):
    betweenness = {}
    for i in nodes:
        betweenness[i] = 0.0

    # Create the phantom node x'=(id,-1) and connect to the graph window with edges with weight zero
    x_prime = {}
    for j in range(timestamp, timestamp - (window), -1):
        if (node, j) in nodes:  # if the (id,timestamp) exist in the graph window
            x_prime.update({(node, j): 0})
    # G is the merged-element graph_window with the x' node. The node x' is the source node at every timestamp.
    G = copy.deepcopy(graph)
    G.update({(node, -1): x_prime})
    node_prime = (node, -1)

    S, P, sigma, D = dijkstra_paths.dijkstra(G, node_prime)
    D_prime = {}
    S_prime = []
    sigma_prime = {}
    S_copy = copy.deepcopy(S)
    while S:

        x = S.pop(0)
        if x[1] != -1 and ((x[0] not in D_prime) or (D[x] == D_prime[x[0]])):
            D_prime[x[0]] = D[x]
            S_prime.append(x)
            sigma_prime[x] = sigma[x]
        else:
            sigma_prime[x] = 0

    betweenness = brandes_algo(betweenness, S_copy, P, sigma, sigma_prime, node)
    return betweenness

def sumation_dictionaries(dict1, dict2):
    a = dict1.copy()
    b = dict2.copy()
    TVBC = {}
    keys = list(set().union(a.keys(),b.keys()))
    for i in keys:
        TVBC[i] = a.get(i,0)+b.get(i,0)
    return TVBC