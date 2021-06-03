#include <stdio.h>
#include "all_structure_undir.cuh"
#include "gpuFunctions_undir.cuh"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <iostream>
#include<vector>
#include <chrono>
#include <algorithm>
#include "cuCompactor.cuh"
#include "supportingFunctions.cu"


#define THREADS_PER_BLOCK 1024 //we can change it

using namespace std;
using namespace std::chrono;



/*
1st arg: original graph file name
2nd arg: no. of nodes
3rd arg: no. of edges
4th arg: input SSSP file name
5th arg: change edges file name
6th arg: drone start vertex
7th arg: destination vertex
8th arg: payload (0 or 7)
9th arg: output file name
****main commands to run****
nvcc -o op_main CudaSSSPmain.cu
./op_main original_graph_file_name number_of_nodes number_of_edges input_SSSP_file_name change_edge_file_name
*/
int main(int argc, char* argv[]) {

	int nodes, edges, deviceId, numberOfSMs;
	int no_of_movement = 0;
	cudaError_t cudaStatus;
	char* graphFile = argv[1];
	nodes = atoi(argv[2]);
	edges = atoi(argv[3]);
	char* inputSSSPfile = argv[4];
	char* changeEdgesFile = argv[5];
	char* outFile = argv[9]; //output file

	//Drone related
	int currentLoc = 0; //drone's current location. considering single drone single depot.
	int nextLoc; //drone's current location
	int droneStartLoc = atoi(argv[6]); //drone start vertex
	int destination = atoi(argv[7]); //destination vertex 
	int payload = atoi(argv[8]); //payload
	int cost = 0; //total cost for travel
	int* traversed;
	traversed = (int*)calloc(nodes, sizeof(int));
	int ws[4] = {0,5,10,15}; //keep the choices here//change if taking different dataset
	int wd[5] = {180,135,90,45,0}; //keep the choices here//change if taking different dataset
	
	int oldRand = 0, oldRand2 = 0;
	
	
	
	


	while (currentLoc != destination && no_of_movement < 20) {

		if (no_of_movement > 0)
		{
			graphFile = "nextGraph.txt";
			inputSSSPfile = "nextSSSP.txt";
			changeEdgesFile = "nextEffectiveChangeEdges.txt"; //have to vary this randomly
		}



		int totalInsertion = 0;
		bool zeroDelFlag = false, zeroInsFlag = false;
		vector<ColWtList> AdjList; //stores input graph in 2D adjacency list
		vector<ColWt> AdjListFull; //Row-major implementation of adjacency list (1D)
		ColWt* AdjListFull_device; //1D array in GPU to store Row-major implementation of adjacency list 
		int* AdjListTracker_device; //1D array to track offset for each node's adjacency list
		vector<changeEdge> allChange_Ins, allChange_Del;
		changeEdge* allChange_Ins_device; //stores all change edges marked for insertion in GPU
		changeEdge* allChange_Del_device; //stores all change edges marked for deletion in GPU
		int* counter_del;
		int* affectedNodeList_del;
		int* updatedAffectedNodeList_del;
		int* updated_counter_del;
		vector<ColList> SSSPTreeAdjList;
		int* SSSPTreeAdjListTracker;
		vector<int> SSSPTreeAdjListFull;
		RT_Vertex* SSSP;
		int* SSSPTreeAdjListFull_device;
		int* SSSPTreeAdjListTracker_device;
		vector<int> hop;
		int* d_hop;





		//Get gpu device id and number of SMs
		cudaGetDevice(&deviceId);
		cudaDeviceGetAttribute(&numberOfSMs, cudaDevAttrMultiProcessorCount, deviceId);
		size_t  numberOfBlocks = 32 * numberOfSMs;

		//Read Original input graph
		AdjList.resize(nodes);
		int* AdjListTracker = (int*)malloc((nodes + 1) * sizeof(int));//we take nodes +1 to store the start ptr of the first row
		read_graphEdges(AdjList, graphFile, &nodes);
		
		

		//Read change edges input
		readin_changes(changeEdgesFile, allChange_Ins, allChange_Del, AdjList, totalInsertion);
		int totalChangeEdges_Ins = allChange_Ins.size();
		if (totalChangeEdges_Ins == 0) {
			zeroInsFlag = true;
		}
		int totalChangeEdges_Del = allChange_Del.size();
		if (totalChangeEdges_Del == 0) {
			zeroDelFlag = true;
		}

		//Transfer input graph, changed edges to GPU and set memory advices
		transfer_data_to_GPU(AdjList, AdjListTracker, AdjListFull, AdjListFull_device,
			nodes, edges, totalInsertion, AdjListTracker_device, zeroInsFlag,
			allChange_Ins, allChange_Ins_device, totalChangeEdges_Ins,
			deviceId, totalChangeEdges_Del, zeroDelFlag, allChange_Del_device,
			counter_del, affectedNodeList_del, updatedAffectedNodeList_del, updated_counter_del, allChange_Del, numberOfBlocks);


		//Read input SSSP Tree and storing on unified memory
		read_and_transfer_input_SSSPtree_to_GPU(inputSSSPfile, SSSPTreeAdjList, SSSPTreeAdjListTracker, SSSPTreeAdjListFull,
			SSSP, nodes, edges, SSSPTreeAdjListFull_device, SSSPTreeAdjListTracker_device, hop, deviceId, d_hop);


		//Initialize supporting variables
		int* change = 0;
		cudaStatus = cudaMallocManaged(&change, sizeof(int));
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMalloc failed at change structure");
		}
		int* affectedNodeList;
		cudaStatus = cudaMallocManaged(&affectedNodeList, nodes * sizeof(int));
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMalloc failed at affectedNodeList structure");
		}
		int* counter = 0;
		cudaStatus = cudaMallocManaged(&counter, sizeof(int));
		if (cudaStatus != cudaSuccess) {
			fprintf(stderr, "cudaMalloc failed at counter structure");
		}
		int* updatedAffectedNodeList_all;
		cudaMallocManaged(&updatedAffectedNodeList_all, nodes * sizeof(int));
		int* updated_counter_all = 0;
		cudaMallocManaged(&updated_counter_all, sizeof(int));





		//**process change edges**
		auto startTimeDelEdge = high_resolution_clock::now(); //Time calculation start
		//Process del edges
		if (zeroDelFlag != true) {

			deleteEdge << < numberOfBlocks, THREADS_PER_BLOCK >> > (allChange_Del_device, SSSP, totalChangeEdges_Del, AdjListFull_device, AdjListTracker_device);
			cudaDeviceSynchronize();
		}
		auto stopTimeDelEdge = high_resolution_clock::now();//Time calculation ends
		auto durationDelEdge = duration_cast<microseconds>(stopTimeDelEdge - startTimeDelEdge);// duration calculation
		//cout << "**Time taken for processing deleted edges: "<< float(durationDelEdge.count()) / 1000 << " milliseconds**" << endl;





		//Process ins edges
		auto startTimeinsertEdge = high_resolution_clock::now();
		if (zeroInsFlag != true) {

			insertEdge << < numberOfBlocks, THREADS_PER_BLOCK >> > (allChange_Ins_device, SSSP, totalChangeEdges_Ins, AdjListFull_device, AdjListTracker_device);
			cudaDeviceSynchronize();
		}
		auto stopTimeinsertEdge = high_resolution_clock::now();//Time calculation ends
		auto durationinsertEdge = duration_cast<microseconds>(stopTimeinsertEdge - startTimeinsertEdge);// duration calculation
		//cout << "**Time taken for processing inserted Edges: "<< float(durationinsertEdge.count()) / 1000 << " milliseconds**" << endl;

		//new code
		//int* Q_array = (int*)malloc((nodes) * sizeof(int));
		//vector<int> Del_Affected_array;

		//auto startTime1 = high_resolution_clock::now();
		//*counter_del = cuCompactor::compact<RT_Vertex, int>(SSSP, affectedNodeList_del, nodes, predicate(), THREADS_PER_BLOCK);
		///*for (int i = 0; i < *counter_del; i++) {
		//	printf("%d::", affectedNodeList_del[i]);
		//}*/
		////cudaDeviceSynchronize();
		//for (int i = 0; i < *counter_del; i++) {
		//	int x = affectedNodeList_del[i];
		//	Del_Affected_array.push_back(x);
		//}
		////printf("test 2");
		//int j = 0;
		//while (j < Del_Affected_array.size())
		//{
		//	int x = Del_Affected_array[j];
		//	if (SSSPTreeAdjList.at(x).size() > 0) {
		//		Del_Affected_array.insert(std::end(Del_Affected_array), std::begin(SSSPTreeAdjList.at(x)), std::end(SSSPTreeAdjList.at(x)));
		//	}
		//	
		//	j++;
		//}
		//auto stopTime1 = high_resolution_clock::now();//Time calculation ends
		//auto durationin1 = duration_cast<microseconds>(stopTime1 - startTime1);// duration calculation
		//cout << "**Time taken for creating Del_Affected_array: "
		//	<< float(durationin1.count()) / 1000 << " milliseconds**" << endl;
		//cout << "size of Del_Affected_array:" << j << endl;
		//for (int i = 0; i < Del_Affected_array.size(); i++) {
		//	printf("%d::", Del_Affected_array[i]);
		//}
		//new code 


		//**make the subtree under deletion affected vertices disconnected (make wt = inf)
		auto startTimeupdateNeighbors_del = high_resolution_clock::now();
		if (zeroDelFlag != true) {
			*counter_del = cuCompactor::compact<RT_Vertex, int>(SSSP, affectedNodeList_del, nodes, predicate(), THREADS_PER_BLOCK);
			*change = 1;
			while (*change > 0) {
				*change = 0;
				updateNeighbors_del << <numberOfBlocks, THREADS_PER_BLOCK >> >
					(SSSP, updated_counter_del, updatedAffectedNodeList_del, affectedNodeList_del, counter_del, SSSPTreeAdjListFull_device, SSSPTreeAdjListTracker_device, change);
				*counter_del = cuCompactor::compact<RT_Vertex, int>(SSSP, affectedNodeList_del, nodes, predicate(), THREADS_PER_BLOCK);
				//printf("number of elements in the compacted list: %d\n", *counter_del);
				//cudaDeviceSynchronize();//not required as cudaMalloc/cudaFree perform heavy-weight synchronizations. cuCompactor::compact uses both in it.
			}
		}
		cudaFree(SSSPTreeAdjListFull_device); //we can free memory at the end if we have enough GPU memory. That will decrease some time
		cudaFree(SSSPTreeAdjListTracker);

		auto stopTimeupdateNeighbors_del = high_resolution_clock::now();//Time calculation ends
		auto durationupdateNeighbors_del = duration_cast<microseconds>(stopTimeupdateNeighbors_del - startTimeupdateNeighbors_del);// duration calculation
		//cout << "**Time taken for updateNeighbors_del: "<< float(durationupdateNeighbors_del.count()) / 1000 << " milliseconds**" << endl;



		//**Update neighbors and connect disconnected vertices with main SSSP tree**
		auto startTimeupdateNeighbors = high_resolution_clock::now();

		//collect all vertices where update value > 0
		*counter = cuCompactor::compact<RT_Vertex, int>(SSSP, affectedNodeList, nodes, predicate2(), THREADS_PER_BLOCK);

		*change = 1;
		while (*change == 1) {
			*change = 0;
			updateNeighbors << <(*counter / THREADS_PER_BLOCK) + 1, THREADS_PER_BLOCK >> > (SSSP, counter, affectedNodeList, AdjListFull_device, AdjListTracker_device, change);
			*counter = cuCompactor::compact<RT_Vertex, int>(SSSP, affectedNodeList, nodes, predicate2(), THREADS_PER_BLOCK);
			//cudaDeviceSynchronize(); //not required as cudaMalloc/cudaFree perform heavy-weight synchronizations. cuCompactor::compact uses both in it.
		}

		auto stopTimeupdateNeighbors = high_resolution_clock::now();//Time calculation ends
		auto durationupdateNeighbors = duration_cast<microseconds>(stopTimeupdateNeighbors - startTimeupdateNeighbors);// duration calculation
		//cout << "**Time taken for updateNeighbors: "<< float(durationupdateNeighbors.count()) / 1000 << " milliseconds**" << endl;






		cout << "****Total Time taken for SSSP update: "
			<< (float(durationDelEdge.count()) + float(durationupdateNeighbors_del.count()) + float(durationinsertEdge.count()) + float(durationupdateNeighbors.count())) / 1000 << " milliseconds****" << endl;



		//cout << "Total affected nodes by Delete edge only: " << totalAffectedNodes_del << endl;

		//print node parent distance
		//cout << "from GPU: \n[";
		//printSSSP << <1, 1 >> > (SSSP, nodes);
		//cudaDeviceSynchronize();
		//int x;
		//if (nodes < 40) {
		//	x = nodes;
		//}
		//else {
		//	x = 40;
		//}
		////cout << "from CPU: \n[";
		//for (int i = 0; i < x; i++) {
		//	cout << i << " " << SSSP[i].Parent << " " << SSSP[i].Dist << endl;
		//	//cout << i << ":" << SSSP[i].Dist << " ";
		//}


		//****Print next move****
		traversed[currentLoc] = 1;
		int parent = -1;
		int y = destination;
		cout << "print path" << endl;
		cout << y;
		while (y != 0) {
			parent = SSSP[y].Parent;
			cout << "<- " << parent << "(" << SSSP[y].Dist - SSSP[parent].Dist << ")"; //test

			if (parent == currentLoc)
			{
				cost = cost + SSSP[y].Dist - SSSP[parent].Dist;
				cout << "\nNext move: " << parent << "to" << y << " cost upto this: " << cost << endl;
				nextLoc = y;
				
			}

			y = parent;
			//cout << i << ":" << SSSP[i].Dist << " ";

		}



		//****print sssp tree in file. format: vertex parent distance****
		ofstream myfile("nextSSSP.txt");
		if (myfile.is_open())
		{
			for (int i = 0; i < nodes; i++) {
				string line = to_string(i) + " " + to_string(SSSP[i].Parent) + " " + to_string(SSSP[i].Dist) + "\n";
				myfile << line;
			}
			myfile.close();
		}
		else cout << "Unable to open nextSSSP.txt file";

		//****print current graph in file. format: vertex1 vertex2 weight****
		ofstream myfile2("nextGraph.txt");
		int nextEdges = 0;
		if (myfile2.is_open())
		{
			for (int i = 0; i < nodes; i++) {
				for (int j = AdjListTracker[i]; j < AdjListTracker[i + 1]; j++) {
					int myn = AdjListFull_device[j].col;
					int mywt = AdjListFull_device[j].wt;
					if (mywt < 0) { continue; } //if mywt = -1, that means edge was deleted
					if (myn < i) { continue; } //avoid printing "b a w1" if "a b w1" is already printed
					string line = to_string(i) + " " + to_string(myn) + " " + to_string(mywt) + "\n";
					myfile2 << line;
					nextEdges++;
				}
			}
			myfile2.close();
		}
		else cout << "Unable to open nextGraph.txt file";


		//****choose and print next effective change edges****
		srand(time(NULL));
		int random1 = rand() % 4; //choice for ws
		int random2 = rand() % 5; //choice for wd
		if(oldRand == random1)
		{
			random1 = (random1 + 1)%4;
		}
		if(oldRand2 == random2)
		{
			random2 = (random2 + 1)%5; //adding 1 just to create another number
		}
		oldRand = random1;
		oldRand2 = random2;
		string ceFileName = "TATA_p" + to_string(payload) + "_ws" + to_string(ws[random1]) +"_wd"+to_string(wd[random2]) + ".txt";
		//int filename_length = ceFileName.length();
		cout<<"next changeEdgeFile::"<<ceFileName<<endl;
		const char *char_fileName = ceFileName.c_str();
		
		ofstream myfile3("nextEffectiveChangeEdges.txt");
		if (myfile3.is_open())
		{
			
			//insert new edges
			FILE* delE_file;
			char line[128];
			//delE_file = fopen("nextChangeEdges.txt", "r"); //select the next Del E***implement a random choice fn***
			delE_file = fopen(char_fileName, "r"); //select the next Del E
			while (fgets(line, 128, delE_file) != NULL)
			{
				int n1, n2, wt;
				changeEdge cE;
				sscanf(line, "%d %d %d", &n1, &n2, &wt);
				//Add change edge in effective change edge only when none of the endpoint is traversed
				if (traversed[n1] == 0 && traversed[n2] == 0)
				{
					int flag1 = 0;
					//****delete edge (u,n,wt) when drone moves from u to v****
					for (int j = AdjListTracker[n1]; j < AdjListTracker[n1 + 1]; j++) {
						int myn = AdjListFull_device[j].col;
						int mywt = AdjListFull_device[j].wt;
						if (mywt < 0) { continue; } //if mywt = -1, that means edge was deleted
						if (myn == n2)
						{
							if (mywt != wt) {
								string line1 = to_string(n1) + " " + to_string(myn) + " " + to_string(mywt) + " " + to_string(0) + "\n"; //delete previous edge
								myfile3 << line1;
								//cout << line1 << endl;
								string line2 = to_string(n1) + " " + to_string(n2) + " " + to_string(wt) + " " + to_string(1) + "\n"; //insert new edge
								myfile3 << line2;
								//cout << line2 << endl;
							}
							
							flag1 = 1;
							break;
						}
					}
					if (flag1 == 0)
					{
						string line2 = to_string(n1) + " " + to_string(n2) + " " + to_string(wt) + " " + to_string(1) + "\n"; //insert new edge
						myfile3 << line2;
						//cout << line2 << endl;
					}
				}
			}
			fclose(delE_file);

			//****delete edge (u,n,wt) when drone moves from u to v****
			for (int j = AdjListTracker[currentLoc]; j < AdjListTracker[currentLoc + 1]; j++) {
				int myn = AdjListFull_device[j].col;
				int mywt = AdjListFull_device[j].wt;
				if (mywt < 0) { continue; } //if mywt = -1, that means edge was deleted
				if (myn == nextLoc) { continue; } //skip as nextLoc is v and (u,v) should be 0
				if (traversed[myn] == 1) { continue; }
				string line4 = to_string(currentLoc) + " " + to_string(myn) + " " + to_string(mywt) + " " + to_string(0) + "\n";
				myfile3 << line4;
			}
			//insert edge (u,v,0) when drone moves from u to v
			string line1 = to_string(currentLoc) + " " + to_string(nextLoc) + " " + to_string(0) + " " + to_string(1) + "\n";
			myfile3 << line1;

			myfile3.close();
		}
		else cout << "Unable to open nextEffectiveChangeEdges.txt file";



		//prepare for next iteration
		currentLoc = nextLoc; //prepare current location for next iteration
		edges = nextEdges;
		no_of_movement++;


		if (zeroDelFlag != true) {
			cudaFree(affectedNodeList_del);
			cudaFree(updatedAffectedNodeList_del);
			cudaFree(counter_del);
			cudaFree(updated_counter_del);
			cudaFree(allChange_Del_device);

		}
		if (zeroInsFlag != true) {
			cudaFree(allChange_Ins_device);
		}
		cudaFree(change);
		cudaFree(affectedNodeList);
		cudaFree(counter);
		cudaFree(AdjListFull_device);
		cudaFree(AdjListTracker_device);

		cudaFree(SSSP);
		cudaFree(d_hop); //try to free this at some earlier place


	}

	std::ofstream ofs;
	ofs.open (outFile, std::ofstream::out | std::ofstream::app);
	ofs << droneStartLoc << " " << destination << " " << cost <<"\n";
	ofs.close();

	return 0;
}