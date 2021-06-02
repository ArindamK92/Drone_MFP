TATA_forwardGraph is the initial forward graph(TATA_p7_ws0_wd0) and an edge (0 1 0) is added to it. ****payload 7****
Always remember to add dummy vertex 0 with the depots

TATA_returnGraph is the initial return graph(TATA_p0_ws0_wd0) and an edge (0 custVertex 0) is added to it. ****payload 0****
Always remember to add dummy vertex 0 with the customers
TATA_returnGraph changes for every customer



****create initial SSSP files****
op_seqSSSP.exe TATA_G_forward.txt 146 > SSSP_TATA_G_forward.txt
op_seqSSSP.exe TATA_G_return.txt 146 > SSSP_TATA_G_return.txt

*****
nodes: 146
edges: 389

****How to run****
op_droneSSSP.exe <graph file> <nodes> <edges> <sssp file> <next edgefile> <drone start vertex> <destination> <payload> <output file>

for forward graph:
op_droneSSSP.exe TATA_forwardGraph.txt 146 389 SSSP_TATA_G_forward.txt dummyCE.txt 1 10 7 resultForward.txt


*payload should be 7 for forward graph, 0 for return graph
**for return graph destination is 1(depot)
for return graph:
op_droneSSSP.exe TATA_returnGraph.txt 146 389 SSSP_TATA_return.txt dummyCE.txt 5 1 0 resultReturn.txt
in return graph command destination is always 1, but return graph and its sssp should be changed every time.