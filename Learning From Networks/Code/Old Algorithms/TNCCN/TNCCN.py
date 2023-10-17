import numpy as np

# compute the temporal betweenness score
def compute_temporal_betweenness(G,end_time,dtgs,timestamp):
    # we start at the end time t = G.t_end, and go backwards in time
    verts = G.nodes()
    n = len(verts)
    # this dictionary stores the distance matrices D_t
    D = {}
    # this dictionary stores the  D_t
    S = {}
    # this stores the cumulative closeness score of G
    betweenness = np.zeros(n)
    timestamp.sort()
    timestamp_asc = timestamp
    timestamp.sort(reverse = True)
    for l,t in list(enumerate(timestamp)): # m time steps
        # this matrix stores the distances at time t
        D[t] = np.ones((n,n))*np.inf
        # this stores the number of shortest paths between two nodes
        S[t] = np.eye(n)
        for v in verts: # n vertices
            if t < end_time:
                for k_t in dtgs.successors_iter(v,t):
                    ki = k_t
                    D[t][v-1,ki-1] = 1
                    S[t][v-1,ki-1] = 1
                for u in verts: # n_vertices
                    # if u is not reachable in one time step
                    if D[t][v-1,u-1] > 1:
                        for k_t in dtgs.successors_iter(v,t): 
                            ki = k_t
                            d = D[timestamp[l-1]][ki-1,u-1] + 1
                            if d < D[t][v-1,u-1]:
                                # there is a shortest path through k!
                                D[t][v-1,u-1] = d
                                S[t][v-1,u-1] = S[timestamp[l-1]][ki-1,u-1]
                            elif d == D[t][v-1,u-1]:
                                # we accumulate the number of shortest paths
                                S[t][v-1,u-1] += S[timestamp[l-1]][ki-1,u-1]
        for si in verts: # n vertices 
            si -= 1
            for di in verts: # n vertices+
                di-=1
                # s and d should be different
                if si == di:
                    continue
                # there should be a shortest path between s and d  
                if S[t][si,di] <= 0:
                    continue
                for vi in verts: # n vertices
                    vi-=1
                    # v should be different from s and d
                    if vi == si or vi == di: 
                        continue
                    # there should exist a path between s and v
                    if S[t][si,vi] <= 0:
                        continue
                    idx_st = l - int(D[t][si,vi])
                    idx_last = len(timestamp_asc)
                    for k in range(idx_st,idx_last):
                        if S[timestamp_asc[k]][vi,di] > 0:
                            d_tk = D[t][si,vi]
                            d_kj = D[timestamp_asc[k]][vi,di]
                            if D[t][si,di] == d_tk + d_kj:
                                if np.isnan(betweenness[vi]):
                                    betweenness[vi] = 0.0
                                if np.isnan(S[t][si,vi]*S[timestamp_asc[k]][vi,di]/S[t][si,di]):
                                    betweenness[vi] += 0.0
                                else :
                                    betweenness[vi] += S[t][si,vi]*S[timestamp_asc[k]][vi,di]/S[t][si,di]
                        else:
                            # no path will exist for t>k
                            break
    return dict(zip(verts,betweenness))