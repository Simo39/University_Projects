import pytglib as tgl  # tglib
import scipy.stats as ss  # for Kendall's tau correlation

# The example from the paper

tgs = tgl.load_ordered_edge_list("../example_datasets/example_from_paper.tg")
stats = tgl.get_statistics(tgs)
print(stats)
closeness_fastest = tgl.temporal_closeness(tgs, tgl.Distance_Type.Fastest)
closeness_ea = tgl.temporal_closeness(tgs, tgl.Distance_Type.Earliest_Arrival)
tau, p_value = ss.kendalltau(closeness_fastest, closeness_ea)
print(tau, p_value)