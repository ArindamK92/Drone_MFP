TATA_G_forward is the initial forward graph(TATA_p7_ws0_wd0) and an edge (0 1 0) is added to it. ****payload 7****
Always remember to add dummy vertex 0 with the depots

TATA_G_return is the initial return graph(TATA_p0_ws0_wd0) and an edge (0 145 0) is added to it. ****payload 0****
Always remember to add dummy vertex 0 with the customers



****create initial SSSP files****
op_seqSSSP.exe TATA_G_forward.txt 146 > SSSP_TATA_G_forward.txt



****How to run****
op_droneSSSP.exe TATA_G_forward.txt 146 389 SSSP_TATA_G_forward.txt TATA_p0_ws0_wd45.txt 10 7