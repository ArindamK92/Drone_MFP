op.exe graph.txt 6 6 sssp.txt dummyCE.txt 5

using dummyCE for the first time as we consider the edges will change after first move and initial path is computed using sequential sssp.


op.exe <graph file> <nodes> <edges> <sssp file> <change edges file> <destination vertex>



****keep the set of snapshots of the graphs at different times****
in our example we considered 3 snapshots named below:
cE_delta0
cE_delta1
cE_delta2

depending on the number of snapshots available change in code:
string ceFileName = "cE_delta" + to_string(no_of_movement % 3) + ".txt"; 