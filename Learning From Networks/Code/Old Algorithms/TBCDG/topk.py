import os
import heapq
# Compute the Top K Results from the Centrality
def topk_results(centralities,k):
    topk = heapq.nlargest(k, centralities.items(), key=lambda i: i[1])
    return topk
# Save the Top K Results in a .*txt file
def write_on_file(topk,filename):
    # Split the extension from the path and normalise it to lowercase.
    ext = os.path.splitext(filename)[-1].lower()
    if ext == ".txt":
        print("Writing the TopK Results in The .txt file.")
        # Opening a file
        try:
            file1 = open(filename, 'w')
            for x in topk:
                # Writing a string to file
                s = str(x[0])+" "+str(x[1])+"\n"
                file1.write(s)
        except Exception as e:
            print(e)
    else:
        print("Required file of .txt format to store the TopKResults.")