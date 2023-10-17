from heapq import heappush, heappop
from itertools import count

def dijkstra(G, s):
    # modified from Eppstein
    S = []
    P = {}

    for v in G:
        P[v] = []
    sigma = {}
    # sigma[v]=0 for v in G
    for i in list(G.keys()):
        sigma[i] = 0.0    
    D = {}
    sigma[s] = 1
    push = heappush
    pop = heappop
    seen = {s: 0}
    c = count()
    Q = []   # use Q as heap with (distance,node id) tuples
    push(Q, (0, next(c), s, s))
    while Q:
        (dist, _, pred, v) = pop(Q)

        if v in D:
            continue  # already searched this node.
        sigma[v] += sigma[pred]  # count paths
        S.append(v)
        D[v] = dist
        for w,edgedata in list(G[v].items()):
            vw_dist = dist + G[v][w]
            if w not in D and (w not in seen or vw_dist < seen[w]):
                seen[w] = vw_dist

                push(Q, (round(vw_dist,15), next(c), v, w))
                sigma[w] = 0.0
                P[w] = [v]
            elif vw_dist == seen[w]:  # handle equal paths
                sigma[w] += sigma[v]
                P[w].append(v)

    #The initialization of the algorithm makes sigma values double. So we can return the exact values dividing by 2.
    for key, value in list(sigma.items()):
        sigma[key] = value / 2
    return S, P, sigma, D