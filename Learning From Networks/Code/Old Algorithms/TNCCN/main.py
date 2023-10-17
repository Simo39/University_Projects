import dynetx as dn
import sys
import topk
import TNCCN

def main():
    print("#############################")
    print("#############################")
    print("#############################")
    # Retrieve the parameters from args
    if len(sys.argv) < 3:
        sys.exit('Usage: python %s #filename #resultFilename #k' % sys.argv[0])

    filename = sys.argv[1]
    print("filename = ", filename)

    result_file = sys.argv[2]
    print("TopK filename = ", result_file)

    k = int(sys.argv[3])
    print("K = ", k)
    print("1- Create the Temporal Network from file")
    # 1- Create the Temporal Network from file
    tgs = dn.read_snapshots(filename, nodetype=int, timestamptype=int)
    # Convert it into a Direct Graph
    print("1.1- Convert it into a Direct Graph")
    dtgs = tgs.to_directed()
    # List of Interactions Periods
    print("1.2- List of Interactions Periods")
    timestamp = sorted(dtgs.temporal_snapshots_ids())
    print(len(timestamp))
    print("1.3- Final Interaction Periods")
    final_t = max(timestamp)
    # 2- Compute the TNCNN centrality
    print("2- Compute the TNCNN centrality")
    centralities = TNCCN.compute_temporal_betweenness(tgs,final_t,dtgs,timestamp)
    # 3- Pick the Topk
    print("3- Pick the Topk")
    tbc_k = topk.topk_results(centralities,k)
    # 4- Save the Topk in a .txt file
    print("4- Save the Topk in a .txt file")
    topk.write_on_file(tbc_k,result_file)

if __name__ == "__main__":
    main()

