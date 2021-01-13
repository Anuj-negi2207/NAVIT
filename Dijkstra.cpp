// C++ program for Dijkstra's single source shortest path algorithm. 
// The program is for adjacency matrix representation of the graph. 


#include <stdio.h> 
#include <limits.h> 
#include <string.h>
#include<vector>
#include <bits/stdc++.h>

//#define PY_SSIZE_T_CLEAN
//#include <Python.h>

using namespace std;




//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




// Number of vertices in the graph 
#define V 45




//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




//classCode
string classCode[V]= {"Entrance", "AB001 - Coding Studio I", "AB002 - Coding Studio II", "AB003 - Coding Studio III", "AB004 - Faculty Cabin - Ground Floor", "AB005 - Computer Lab 1", "AB008 - Music Room - Auditorium Main", "Auditorium Back", "Conference Room", "Leftest Corner", "Stairs at AB003", "Stairs at Auditorium", "Stairs at SDC Office", "Stairs mid of Ground and 1st Floor at AB003", "Stairs mid of Ground and 1st Floor at Auditorium", "Stairs mid of Ground and 1st Floor at SDC Office", 
"AB101 - Math Studio I", "AB102 - Math Studio II", "AB103 - Math Studio II", "AB104 - Faculty Cabin - 1st Floor", "AB105 - Computer Lab 2", "AB108 - Joint - Examination Cabin", "Library ", "Unknown Lab", "Stairs at AB103", "Stairs at Examination Cabin", "Stairs at Library", "Stairs mid of 1st and 2nd Floor at AB103", "Stairs mid of 1st and 2nd Floor at Examination Cabin", "Stairs mid of 1st and 2nd Floor at Library", 
"AB201 - ECE Lab", "AB202 - Classroom", "AB203 - Engineering Drawing", "AB204 - Faculty Cabin - 2nd Floor", "AB205 - English Studio", "AB208 - Joint", "AB209 - Hall", "AB210 - Classroom", "AB211 - EEE Lab", "Stairs at Engineering Studio", "Stairs at AB209", "Stairs at ECE Lab", "Stairs mid of 2nd and 3rd Floor at Engineering Studio", "Stairs mid of 2nd and 3rd Floor at AB209", "Stairs End"};







//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						
//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




// A utility function to find the vertex with minimum distance value, from the set of vertices not yet included in shortest path tree 
int minDistance(double dist[], bool sptSet[]) 
{ 
	
	// Initialize min value 
	double min = INT_MAX, min_index; 

	for (int v = 0; v < V; v++) 
		if (sptSet[v] == false && 
				dist[v] <= min) 
			min = dist[v], min_index = v; 

	return min_index; 
} 




//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




// Function to print shortest path from source to j using parent array 
void printPath(double parent[], int j) 
{ 
	
	// Base Case : If j is source 
	if (parent[j] == - 1) 
		return; 

	printPath(parent, parent[j]); 

	cout<<"\n  -->   "<<classCode[j]; 
} 




//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




// A utility function to print the constructed distance array 
void printSolution(double dist[], double parent[], int src) 
{ 
 
	cout<<"Vertex\t Distance\tPath"; 
	for (int i = 0; i < V; i++) 
	{ 
		if(i==src)
			continue;
			
		cout<<"\n"<<classCode[src]<<" -> "<<classCode[i]<<" \t\t "<<dist[i]<<"\t\t"<<classCode[src]; 
		printPath(parent, i); 
	} 
} 





//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						





//Function to display source to destination shortest distance (IMPORTANT)
void StoD(double dist[], double parent[], int src, int dest)
{
	cout<<"The best path between "<<classCode[src]<<" and "<<classCode[dest]<<" can be - \n";
	printPath(parent, dest);
}






//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						
//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						







// Funtion that implements Dijkstra's single source shortest path algorithm for a graph represented using adjacency matrix representation 




//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




//1st dijsktra function to print all destination
void dijkstra(double graph[V][V], int src) 
{ 
	
	// The output array. dist[i] will hold the shortest distance from src to i 
	double dist[V]; 

	// sptSet[i] will true if vertex i is included / in shortest path tree or shortest distance from src to i is finalized 
	bool sptSet[V]; 

	// Parent array to store shortest path tree 
	double parent[V]; 

	// Initialize all distances as INFINITE and stpSet[] as false 
	for (int i = 0; i < V; i++) 
	{ 
		parent[0] = -1; 
		dist[i] = INT_MAX; 
		sptSet[i] = false; 
	} 

	// Distance of source vertex from itself is always 0 
	dist[src] = 0; 

	// Find shortest path for all vertices 
	for (int count = 0; count < V - 1; count++) 
	{ 
		// Pick the minimum distance 
		// vertex from the set of 
		// vertices not yet processed. 
		// u is always equal to src 
		// in first iteration. 
		int u = minDistance(dist, sptSet); 

		// Mark the picked vertex 
		// as processed 
		sptSet[u] = true; 

		// Update dist value of the 
		// adjacent vertices of the 
		// picked vertex. 
		for (int v = 0; v < V; v++) 

			// Update dist[v] only if is 
			// not in sptSet, there is 
			// an edge from u to v, and 
			// total weight of path from 
			// src to v through u is smaller 
			// than current value of 
			// dist[v] 
			if (!sptSet[v] && graph[u][v] && 
				dist[u] + graph[u][v] < dist[v]) 
			{ 
				parent[v] = u; 
				dist[v] = dist[u] + graph[u][v]; 
			} 
	} 

	// print the constructed 
	// distance array 
	printSolution(dist, parent, src); 
	
} 





//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						





//2nd dijkstra function for source to destination
void dijkstra(double graph[V][V], int src, int dest) 
{ 
	
	// The output array. dist[i] will hold the shortest distance from src to i 
	double dist[V]; 

	// sptSet[i] will true if vertex i is included / in shortest path tree or shortest distance from src to i is finalized 
	bool sptSet[V]; 

	// Parent array to store shortest path tree 
	double parent[V]; 

	// Initialize all distances as INFINITE and stpSet[] as false 
	for (int i = 0; i < V; i++) 
	{ 
		parent[0] = -1; 
		dist[i] = INT_MAX; 
		sptSet[i] = false; 
	} 

	// Distance of source vertex from itself is always 0 
	dist[src] = 0; 

	// Find shortest path for all vertices 
	for (int count = 0; count < V - 1; count++) 
	{ 
		// Pick the minimum distance 
		// vertex from the set of 
		// vertices not yet processed. 
		// u is always equal to src 
		// in first iteration. 
		int u = minDistance(dist, sptSet); 

		// Mark the picked vertex 
		// as processed 
		sptSet[u] = true; 

		// Update dist value of the 
		// adjacent vertices of the 
		// picked vertex. 
		for (int v = 0; v < V; v++) 

			// Update dist[v] only if is 
			// not in sptSet, there is 
			// an edge from u to v, and 
			// total weight of path from 
			// src to v through u is smaller 
			// than current value of 
			// dist[v] 
			if (!sptSet[v] && graph[u][v] && 
				dist[u] + graph[u][v] < dist[v]) 
			{ 
				parent[v] = u; 
				dist[v] = dist[u] + graph[u][v]; 
			} 
	} 

	
	//Function calling to display source to destination shortest distance (IMPORTANT)
	StoD(dist, parent, src, dest);
}




//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						





//Global Graph of VIT academic block upto 2nd floor using adjanceny matrix
double graph[V][V] = {	{0.  , 1.56, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 8.  , 0.  , 0.  , 3.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//01
						
						{1.56, 0.  , 1.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//02
						
						{0., 1., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.}, 																												//03
						
						{0.  , 0.  , 1.  , 0.  , 0.  , 3.04, 0.  , 0.  , 0.  , 0.  , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  },						//04 
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0.5, 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0.}, 																	//05
       
						{0.  , 0.  , 0.  , 3.04, 0.  , 0.  , 3.04, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  }, 					//06
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 3.04, 0.  , 2.08, 0.  , 0.  , 0.  , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//07
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 2.08, 0.  , 1.09, 1.27, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//08
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.09, 0.  , 1.02, 0.  , 0.  , 4.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. },						//09
	    				
						{8.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.27, 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//10
						
						{0.  , 0.  , 0.  , 0.5 , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. },						//11 
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. },						//12
						
						{3.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 4.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//13
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//14
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//15
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//16
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.  , 0.  , 0.  , 0.  , 0.  , 2.33, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. },						//17 
						
						{0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.},																												//18 
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.  , 0.  , 0.  , 3.04, 0.  , 0.  , 0.  , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. },						//19 
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0.5, 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. }, 																	//20
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 3.04, 0.  , 0.  , 3.04, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//21
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 3.04, 0.  , 0.  , 3.2 , 0.  , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//22
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.07, 2.33, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 4.  , 0.  , 0.  , 0.2 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. },						//23 
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 3.2, 4. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. },																	//24 
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.5 , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//25
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. },						//26 
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.2 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 					//27
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0. }, 						//28
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0. }, 						//29
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0. }, 						//30
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.2 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 2.01, 0.  , 0.  , 0. }, 						//31
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 1.2, 0. , 2. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. }, 																	//32
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 2.  , 0.  , 0.  , 3.04, 0.  , 0.  , 0.  , 0.  , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  }, 					//33
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0.5, 0. , 0. , 0. , 0. , 0. }, 																	//34
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 3.04, 0.  , 0.  , 3.04, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0. }, 						//35
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 3.04, 0.  , 0.5 , 0.  , 0.  , 0.  , 0.5 , 0.  , 0.  , 0.  , 0. },						//36
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0.5, 0. , 2. , 0. , 0. , 0.7, 0. , 0. , 0. , 0. }, 																	//37
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 2. , 0. , 0.5, 0. , 0. , 0. , 0. , 0. , 0. }, 																	//38
						
						{0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0. , 0.5, 0. , 0. , 0. , 0. , 0. , 0. , 0. }, 																	//39
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.5 , 0.5 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0. }, 						//40
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.5 , 0.7 , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0. },						//41 
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 2.01, 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02}, 					//42
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0.  , 0. }, 						//43
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0.  , 0. },						//44
						
						{0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 0.  , 1.02, 0.  , 0.  , 0. } 						//45
						
						};



//TO BE EDITED


/*




//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						

// Find the query string at that index in the array of 
// query string is contained in the classCode string   
// Blueprint


bool isInBigString(string bigString, string smallString); 
bool isInBigStringHelper(string bigString, string smallString, int startIdx); 
  
// Function to the multiStringSearch 
vector<bool> multiStringSearch(string bigString, 
                          vector<string> smallStrings) 
{ 
    vector<bool> solution; 
  
    // iterate in the smallString 
    for (string smallString : smallStrings) { 
  
        // calling the isInBigString Function 
        solution.push_back(isInBigString(bigString, smallString)); 
    } 
    return solution; 
} 


//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						



 
// Function to the bigString 
bool isInBigString(string bigString, string smallString) 
{ 
    // iterate in the bigString 
    for (int i = 0; i < bigString.length(); i++) { 
  
        // Check if length of smallString + i is greater than 
        // the length of bigString 
        if (i + smallString.length() > bigString.length()) { 
            break; 
        } 
  
        // call the function isInBigStringHelper 
        if (isInBigStringHelper(bigString, smallString, i)) { 
            return true; 
        } 
    } 
    return false; 
} 
  



//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						






// Helper Function to the Finding bigString 
bool isInBigStringHelper(string bigString, string smallString, int startIdx) 
{ 
    // Initialize leftBigIdx and rightBigIdx and leftSmallIdx variables 
    int leftBigIdx = startIdx; 
    int rightBigIdx = startIdx + smallString.length() - 1; 
    int leftSmallIdx = 0; 
    int rightSmallIdx = smallString.length() - 1; 
  
  
    // Iterate until leftBigIdx variable reaches less  
    // than or equal to rightBigIdx 
    while (leftBigIdx <= rightBigIdx) { 
  
        // Check if bigString[leftBigIdx] is not equal 
        // to smallString[leftSmallIdx] or Check if  
        // bigString[rightBigIdx] is not equal to  
        // smallString[rightSmallIdx] than return false 
        // otherwise increment leftBigIdx and leftSmallIdx 
        // decrement rightBigIdx and rightSmallIdx 
        if (bigString[leftBigIdx] != smallString[leftSmallIdx] ||  
            bigString[rightBigIdx] != smallString[rightSmallIdx]) { 
            return false; 
        } 
  
        leftBigIdx++; 
        rightBigIdx--; 
        leftSmallIdx++; 
        rightSmallIdx--; 
    } 
  
    return true; 
} 






//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




/*
// Find the query string at that index in the array of 
// query string is contained in the classCode string   
// Blueprint of the TrieNode 
class TrieNode { 
public: 
  
    // Declaring children to be of <char, TrieNode> type 
    // key will be of char type and mapped value will 
    // be of TrieNode type 
    unordered_map<char, TrieNode*> children; 
    string word; 
}; 
  
// Blueprint of the Trie 
class Trie { 
public: 
    TrieNode* root; 
    char endSymbol; 
    Trie() 
    { 
        this->root = new TrieNode(); 
        this->endSymbol = '*'; 
    } 
  
    // function to insert string 
    void insert(string str) 
    { 
        TrieNode* current = this->root; 
  
        // iterate in the length of String 
        for (int i = 0; i < str.length(); i++) { 
  
            // initialize char as a letter 
            // put the value of str[i] in letter 
            char letter = str[i]; 
  
            // Check if letter is is equal to endnode or not 
            if (current->children.find(letter) == current->children.end()) { 
                TrieNode* newNode = new TrieNode(); 
                current->children.insert({ letter, newNode }); 
            } 
            current = current->children[letter]; 
        } 
        current->children.insert({ this->endSymbol, NULL }); 
        current->word = str; 
    } 
}; 
  
// define a findSmallStringsIn function 
void findSmallStringsIn(string str, int startIdx, Trie* trie,  
                   unordered_map<string, bool>* containedStrings); 
  
// Function to the multiStringSearch 
vector<bool> multiStringSearch(string bigString, vector<string> smallStrings) 
{ 
    Trie* trie = new Trie(); 
  
    // iterate in the smallString 
    for (string smallString : smallStrings) { 
        trie->insert(smallString); 
    } 
  
    // Declaring containedStrings to be of <string, bool> type 
    // key will be of string type and mapped value will 
    // be of boolean type 
    unordered_map<string, bool> containedStrings; 
  
    // iterate in the bigString 
    for (int i = 0; i < bigString.length(); i++) { 
        findSmallStringsIn(bigString, i, trie, &containedStrings); 
    } 
  
    vector<bool> solution; 
  
    // iterate in the smallString 
    for (string smallString : smallStrings) { 
        solution.push_back(containedStrings.find(smallString)  
                                   != containedStrings.end()); 
    } 
    return solution; 
} 
  
// Function to findSmallStringsIn 
void findSmallStringsIn(string str, int startIdx,  
    Trie* trie, unordered_map<string, bool>* containedStrings) 
{ 
    TrieNode* currentNode = trie->root; 
  
    // iterate the length of the string 
    for (int i = startIdx; i < str.length(); i++) { 
  
        // Check if letter is is equal to endnode or not 
        if (currentNode->children.find(str[i]) == 
                          currentNode->children.end()) { 
            break; 
        } 
        currentNode = currentNode->children[str[i]]; 
  
        if (currentNode->children.find(trie->endSymbol) !=  
                             currentNode->children.end()) { 
            containedStrings->insert({ currentNode->word, true }); 
        } 
    } 
} 

*/







//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						




/*

// FOR LATER


//A utility function to take better input
int unique_input()
{
	string word, query;
	vector<string> words;
	
	//Taking input from the user through this utility function
	cin>>query;
	
	
	
	int i,j=0;
	
	
	//Converting the query entered into substrings 
	for(i=0; i<sizeof(query); i++)
	{	
		char temp;
		temp= query[i];

		if( (temp>=97 && temp<=122) || (temp>=65 && temp<=90) || (temp>=48 && temp<=57) )
			word.push_back(temp);

		else
		{
			//If a word is disturbed whether by any unique character, we'll enter the word in vector
			words.push_back(word);		
			j++;
			word.erase();	//erasing the word so that it can take new values
		}
	}
	
	for(i=0; i<V; i++)
	{
		string S;
		S = classCode[i];	//From classCode, taking values to compare
		
		vector<bool> ans = multiStringSearch(S, words);	//this will check whether those words are present in classCode or not
		
		for(int k=0;k<ans.size();k++)
		{
			if(ans[k]==1)
			continue;		//words are present, and at the end, k= ans.size() and that's why in next step, k-1 is being compared
			
			else
			{
				k++;		//this is done for ease in next step 
				break;		//if any word will be present that time, meaning something wrong is entered as input
			}
		}
		
		if(ans[k-1]==1)		//if all values are true then, it'll return that index value from classCode, else if not, Loop will continue
		{
			return i;
			break;		//index found, exit from code
		}
	}
	
}	

*/


//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						
//___________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________						









//Node Details
/*
0  -> 	O 		->  Entrance 
1  -> 	A0 		->  AB001 - Coding Studio I 
2  -> 	B0 		->  AB002 - Coding Studio II 
3  -> 	C0 		->  AB003 - Coding Studio III 
4  -> 	D0 		->  AB004 - Faculty Cabin - Ground Floor 
5  ->	G0		->  AB005 - Computer Lab 1 
6  ->	J0 		->  AB008 - Music Room - Auditorium Main 
7  -> 	H0 		->  Auditorium Back 
8  -> 	Y0 		->  Conference Room 
9  -> 	L 		->  Leftest Corner 
10 -> 	S11 	->  Stairs at AB003 
11 -> 	S12 	->  Stairs at Auditorium 
12 -> 	S13 	->  Stairs at SDC Office 
13 -> 	M11 	->  Stairs mid of Ground and 1st Floor at AB003 
14 -> 	M12	 	->  Stairs mid of Ground and 1st Floor at Auditorium 
15 -> 	M13 	->  Stairs mid of Ground and 1st Floor at SDC Office 
16 -> 	A1 		->  AB101 - Math Studio I 
17 -> 	B1 		->  AB102 - Math Studio II 
18 -> 	C1 		->  AB103 - Math Studio II 
19 -> 	D1 		->  AB104 - Faculty Cabin - 1st Floor 
20 -> 	G1 		->  AB105 - Computer Lab 2 
21 -> 	J1 		->  AB108 - Joint - Examination Cabin 
22 -> 	LIB		->  Library  
23 -> 	Y1 		->  Unknown Lab 
24 -> 	S21 	->  Stairs at AB103 
25 -> 	S22 	->  Stairs at Examination Cabin 
26 -> 	S23	 	->  Stairs at Library 
27 -> 	M21 	->  Stairs mid of 1st and 2nd Floor at AB103 
28 -> 	M22 	->  Stairs mid of 1st and 2nd Floor at Examination Cabin 
29 -> 	M23 	->  Stairs mid of 1st and 2nd Floor at Library 
30 -> 	LAB 1 	->  AB201 - ECE Lab 
31 -> 	A2 		->  AB202 - Classroom 
32 -> 	C2 		->  AB203 - Engineering Drawing 
33 -> 	D2 		->  AB204 - Faculty Cabin - 2nd Floor 
34 -> 	G2 		->  AB205 - English Studio 
35 -> 	J2 		->  AB208 - Joint 
36 -> 	K2 		->  AB209 - Hall 
37 -> 	N2 		->  AB210 - Classroom 
38 -> 	LAB 2 	->  AB211 - EEE Lab 
39 -> 	S31 	->  Stairs at Engineering Studio 
40 -> 	S32 	->  Stairs at AB209 
41 -> 	S33 	->  Stairs at ECE Lab 
42 -> 	M31 	->  Stairs mid of 2nd and 3rd Floor at Engineering Studio 
43 -> 	M32 	->  Stairs mid of 2nd and 3rd Floor at AB209 
44 -> 	M33 	->  Stairs End 

*/









					
// Driver Code 
int main() 
{ 	 
	int src=0, dest=34;
	
	//cout<<"Please enter your area:";
	//src = unique_input();
	
	//cout<<"Please enter the desired destination:";
	//dest = unique_input();
	
	
	dijkstra(graph, src, dest); 
	return 0; 
} 

