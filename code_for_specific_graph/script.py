# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 16:39:09 2021

@author: Arindam
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import seaborn as sns
import networkx as nx
import sympy as sp
import mpmath
from geopy.distance import geodesic
from geographiclib.geodesic import Geodesic

os.chdir("C:\\PhD courses\\Adv CPS\\project\\code\\Data_new")


#create forward graph
os.system('copy TATA_p7_ws0_wd0.txt TATA_forwardGraph.txt')
os.system("echo 0 1 0 >> TATA_forwardGraph.txt")
#find initial SSSP of forward graph
os.system('op_seqSSSP.exe TATA_forwardGraph.txt 146 > SSSP_TATA_forward.txt')

greyVertices = [2,3,4,5,6,7,8,10,11,12,13,14,15,16,17,18,19,20,21,22,25,27,28,31,32,35,36,41,42,43,44,45,46,47,48,49,50,51,52,53,54,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,72,73,74,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,120,121,122,123,124,125,126,127,128,129,133,134,136,137,138,139,140,141,142,143]

for x in range(20):
    for i in greyVertices:
        #create return graph
        os.system('copy TATA_p0_ws0_wd0.txt TATA_returnGraph.txt')
        cmnd = "echo 0 "+ str(i)+" 0 >> TATA_returnGraph.txt"
        os.system(cmnd)
        #find initial SSSP of return graph
        os.system('op_seqSSSP.exe TATA_returnGraph.txt 146 > SSSP_TATA_return.txt')
        #compute cost for forward path
        cmnd1 = "op_droneSSSP.exe TATA_forwardGraph.txt 146 389 SSSP_TATA_G_forward.txt dummyCE.txt 1 " + str(i) + " 7 resultForward"+str(x)+".txt"
        os.system(cmnd1)
        #compute cost for return path
        cmnd1 = "op_droneSSSP.exe TATA_returnGraph.txt 146 389 SSSP_TATA_return.txt dummyCE.txt "+str(i)+" 1 0 resultReturn"+str(x)+".txt"
        os.system(cmnd1)

#print output after running OS command
#stream = os.popen('op_droneSSSP.exe TATA_G_forward.txt 146 389 SSSP_TATA_G_forward.txt TATA_p0_ws0_wd45.txt 11 7')
#output = stream.readlines()
#print(output)



