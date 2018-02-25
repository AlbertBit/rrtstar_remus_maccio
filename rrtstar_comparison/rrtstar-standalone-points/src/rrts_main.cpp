#define LIBBOT_PRESENT 0

#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <fstream>
#include <cstdlib>
#include <list>
#include <string>

#include "rrts.hpp"
#include "system_single_integrator.h"


using namespace RRTstar;
using namespace SingleIntegrator;

using namespace std;

#define DATALOGPATH "./../../Datalog"
#define OBSTACLEPATH "./../../Datalog/Planner/Time/Obstacle15"
#define TIMEPATH "./../../Datalog/Planner/Time"
#define N_OBSTACLE 15

#define N_ITERATIONS 20000
#define N_SIMULATIONS 10


typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;

int publishTree (int iterator, planner_t& planner, System& system);
int publishTraj (int iterator, planner_t& planner, System& system);
int randObstacleSizeGenerator ( double *obsSize);
int randObstacleCenterGenerator (int iterator, double *obsSize, double *obsCenter,const double *rootcart,const double *goalcenter,const double *goalsize);




int main () {

	//for (int iter=0;iter<100;iter++){
    int iter = 0;
    planner_t rrts;
    cout << "*****************" << endl;
    cout << "RRTstar is alive: " <<iter<< endl;




    // Create the dynamical system
    System system;

    // Three dimensional configuration space
    system.setNumDimensions (3);

    // Define the operating region
    system.regionOperating.setNumDimensions(3);
    system.regionOperating.center[0] = 0.0;
    system.regionOperating.center[1] = 0.0;
    system.regionOperating.center[2] = 0.0;
    system.regionOperating.size[0] = 20.0;
    system.regionOperating.size[1] = 20.0;
    system.regionOperating.size[2] = 20.0;

    // Define the goal region
    system.regionGoal.setNumDimensions(3);
    system.regionGoal.center[0] = 9.0;
    system.regionGoal.center[1] = 9.0;
    system.regionGoal.center[2] = 9.0;
    system.regionGoal.size[0] = 2.00;
    system.regionGoal.size[1] = 2.00;
    system.regionGoal.size[2] = 2.00;

    // Save Region Data
    //change:
    	// timeEvaluation/Obstacle3
    	//01Time.txt
    	// Number of obstacles
	const char* DataLogPath	=OBSTACLEPATH;
	string DataLogPath2		=OBSTACLEPATH;
	int createDirVal = mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    //cout<<"return value: "<<DataLogPath2<<createDirVal<<std::endl;
	const char* DataLogPathTime	=TIMEPATH;
	string DataLogPath2Time		=TIMEPATH;
	mkdir(DataLogPathTime, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	ofstream Myfile1, Myfile2, Myfile3;
	char stringIter[10];
	sprintf(stringIter, "%d", iter);

    string obstaclePath = DATALOGPATH;
	Myfile3.open ((obstaclePath+"/"+stringIter+"Regions.txt").c_str(),ios::out);
	Myfile3.close();

	Myfile1.open ((DataLogPath2+"/"+stringIter+"Regions.txt").c_str(),ios::trunc);
	Myfile2.open ((DataLogPath2Time+"/15Time.txt").c_str(),ios::app);

	Myfile1 <<system.regionOperating.center[0]<<" "<<system.regionOperating.center[1]<<" "<<system.regionOperating.center[2]<<" ";
	Myfile1 <<system.regionOperating.size[0]<<" "<<system.regionOperating.size[1]<<" "<<system.regionOperating.size[2]<<"\n";
	Myfile1.close();

	//Setup the root vertex and obstacle generators
    //srand(time(NULL));
	double obstacleSize[3];
	double obstacleCenter[3];
	double rootCartPos[3]={-9.0,-9.0,-9.0};

    // Add the system to the planner
    rrts.setSystem (system);

    // Set up the root vertex
    vertex_t &root = rrts.getRootVertex();
    State &rootState = root.getState();
    rootState[0] = rootCartPos[0];
    rootState[1] = rootCartPos[1];
    rootState[2] = rootCartPos[2];


	//	randObstacleSizeGenerator ( obstacleSize);
//	cout<<"Main: "<<obstacleSize[0]<<" "<<obstacleSize[1]<<" "<<obstacleSize[2]<<endl;
//	randObstacleCenterGenerator ( obstacleSize,obstacleCenter,rootCartPos,system.regionGoal.center,system.regionGoal.size);
//	cout<<"Main: "<<obstacleCenter[0]<<" "<<obstacleCenter[1]<<" "<<obstacleCenter[2]<<endl;

    for( int i = 0; i < N_OBSTACLE; i++ ) {
        randObstacleSizeGenerator ( obstacleSize);
    	randObstacleCenterGenerator ( iter,obstacleSize,obstacleCenter,rootCartPos,system.regionGoal.center,system.regionGoal.size);
    	region *obstacle;
        obstacle = new region;
        obstacle->setNumDimensions(3);
        obstacle->center[0] = obstacleCenter[0];
        obstacle->center[1] = obstacleCenter[1];
        obstacle->center[2] = obstacleCenter[2];
        obstacle->size[0] = obstacleSize[0];
        obstacle->size[1] = obstacleSize[1];
        obstacle->size[2] = obstacleSize[2];
        system.obstacles.push_front (obstacle);
    }

    // Initialize the planner
    rrts.initialize ();

    // This parameter should be larger than 1.5 for asymptotic
    //   rather than exploration in the RRT* algorithm. Lower
    //   optimality. Larger values will weigh on optimization
    //   values, such as 0.1, should recover the RRT.
    rrts.setGamma (1.5);



    clock_t start = clock();

    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < N_ITERATIONS; i++)
        rrts.iteration ();

    clock_t finish = clock();
    cout << "Time [s]: " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;
    cout << "Sample State Time [s]: "<<system.sample_state_time/(1000000)<<endl;;
    cout << "Extend To Time [s]: "<<system.extend_to_time/(1000000)<<endl;


    // Kourosh:

	Myfile2 <<iter<<" "<<((double)(finish-start))/CLOCKS_PER_SEC<<"\n" ;
	Myfile2.close();

    publishTree (iter,rrts, system);
    publishTraj (iter,rrts, system);

    /*
	const char* DataLogPath	="/home/nasa/Datalog/Planner";
	string DataLogPath2		="/home/nasa/Datalog/Planner";
	mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	ofstream Myfile1;
	Myfile1.open ((DataLogPath2+"/02path.txt").c_str(),ios::app);


    cout << "numVertices : " << rrts.numVertices << endl;
    cout << "getNumDimensions : " << system.getNumDimensions()<<endl;
    std::list<double*> traj;

    std::cout <<"Trajectory size: "<< rrts.listVertices.size()<< endl;

    rrts.getBestTrajectory(traj);
    int counter=0;
    for (std::list<vertex_t*>::iterator it = rrts.listVertices.begin(); it != rrts.listVertices.end(); it++){
    	//std::cout << (*it)->getState()[0] << ' ';
    	counter++;
    	Myfile1 <<counter<<" "<<(*it)->getState()[0]<<" "<<(*it)->getState()[1]<<" "<<(*it)->getState()[2]<<"\n";
    }

     std::cout << '\n';
     Myfile1.close();
     //std::cout <<"Trajectory size: "<< traj.size()<< endl;

 */
 return 1;
}

int publishTraj ( int iterator,planner_t& planner, System& system) {

	const char* DataLogPath	=OBSTACLEPATH;
	string DataLogPath2		=OBSTACLEPATH;
	mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	char stringIter[10];
	sprintf(stringIter, "%d", iterator);
	ofstream Myfile1;
	Myfile1.open ((DataLogPath2+"/"+stringIter+"optTraj.txt").c_str(),ios::trunc);

    cout << "Publishing trajectory -- start" << endl;

    vertex_t& vertexBest = planner.getBestVertex ();

    //cout<<"planner.getBestVertexCost(): "<<planner.getBestVertexCost()<<endl;

    if (&vertexBest == NULL) {
        cout << "No best vertex" << endl;
        return 0;
    }

    list<double*> stateList;
    planner.getBestTrajectory (stateList);
    cout<<"stateList.size(): "<<stateList.size()<<endl;

    int stateIndex = 0;
    for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
        double* stateRef = *iter;
//      cout<<stateRef[0]<<" "<<stateRef[1];
    	Myfile1 <<stateRef[0]<<" "<<stateRef[1]<<" ";
    	//cout<<"system.getNumDimensions(): "<<system.getNumDimensions()<<endl;
        if (system.getNumDimensions() > 2){
//        	cout<<" "<<stateRef[2]<<"\n";
        	Myfile1 <<stateRef[2]<<"\n";
        }
        else{
//           	cout<<0.0<<"\n";
           	Myfile1 <<0.0<<"\n";
        }
        delete [] stateRef;

        stateIndex++;
    }
    Myfile1.close();
    cout << "Publishing trajectory -- end" << endl;
    return 1;
}

int publishTree (int iterator,planner_t& planner, System& system) {

	const char* DataLogPath	=OBSTACLEPATH;
	string DataLogPath2		=OBSTACLEPATH;
	mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	ofstream Myfile1,Myfile2;
	char stringIter[10];
	sprintf(stringIter, "%d", iterator);

	Myfile1.open ((DataLogPath2+"/"+stringIter+"Vertices.txt").c_str(),ios::trunc);
	Myfile2.open ((DataLogPath2+"/"+stringIter+"Edges.txt").c_str(),ios::trunc);

    cout << "Publishing the tree -- start" << endl;

    bool plot3d = (system.getNumDimensions() > 2);

   int num_vertices = planner.numVertices;

   //PUBLISH VERTICES

    if (num_vertices > 0) {

        int vertexIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {

            vertex_t &vertexCurr = **iter;
            State &stateCurr = vertexCurr.getState ();

//            cout<<vertexIndex<<" "<<stateCurr[0]<<" "<<stateCurr[1];
            Myfile1 <<stateCurr[0]<<" "<<stateCurr[1];
            if (plot3d){
//            	cout<<" "<<stateCurr[2]<<endl;
            	Myfile1 <<" "<<stateCurr[2]<<"\n";
            }
             else{
//            	cout<<" "<<0.0<<endl;
            	Myfile1 <<" "<<0.0<<"\n";
             }
            vertexIndex++;

        }

    }
    else {
    	cout<<"num_vertices <= 0"<<endl;
    }

    //PUBLISH EDGES
    if (num_vertices > 1) {

        int num_edges = num_vertices - 1;

        int edgeIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {

            vertex_t &vertexCurr = **iter;

            vertex_t &vertexParent = vertexCurr.getParent();

            if ( &vertexParent == NULL )
                continue;

            State &stateCurr = vertexCurr.getState ();
            State &stateParent = vertexParent.getState();

//            cout<<edgeIndex<<" "<<stateParent[0]<<" "<<stateParent[1]<<" ";
            Myfile2 <<stateParent[0]<<" "<<stateParent[1]<<" ";
            if (plot3d){
//            	cout<<stateParent[2]<<" ";
            	Myfile2 <<stateParent[2]<<" ";
            }
            else{
//            	cout<<0.0<<" ";
            	Myfile2 <<0.0<<" ";
            }
//            cout<<stateCurr[0]<<" "<<stateCurr[1]<<" ";
            Myfile2 <<stateCurr[0]<<" "<<stateCurr[1]<<" ";

            if (plot3d){
//            	cout<<stateCurr[2]<<endl;
                Myfile2 <<stateCurr[2]<<"\n";
            }
            else{
//            	cout<<0.0<<endl;
            	Myfile2 <<0.0<<"\n";
            }
            edgeIndex++;
        }

    }
    else {
       	cout<<"num_vertices <= 1"<<endl;
    }
    Myfile1.close();
    Myfile2.close();

    cout << "Publishing the tree -- end" << endl;

    return 1;
}

int randObstacleSizeGenerator ( double *obsSize){
	const double MAX_size=10.0;
	obsSize[0]=(double)rand()/(RAND_MAX )*MAX_size;
	obsSize[1]=(double)rand()/(RAND_MAX )*MAX_size;
	obsSize[2]=(double)rand()/(RAND_MAX )*MAX_size;
//	cout<<obsSize[0]<<" "<<obsSize[1]<<" "<<obsSize[2]<<endl;

	return 1;
}

int randObstacleCenterGenerator (int iterator, double *obsSize, double *obsCenter,const double *rootcart,const double *goalcenter,const double *goalsize){


	// 3: working space dimension
	// 10.0: is the working space size in positive direction

	double MAX_centerLim[3]={10.0-obsSize[0]/2.0,10.0-obsSize[1]/2.0,10.0-obsSize[2]/2.0};
	bool obsVerified[3]={false,false,false};
	// Generate random center for obstacle
	// 10.0: is the working space size in positive direction

	do{
			for (int i=0;i<3;i++){ // for each space dimension

				obsCenter[i]=((double)rand()/(RAND_MAX/2.0)-1.0)*MAX_centerLim[i];
				obsVerified[i]=false;

				//check for having overlap between generated obstacle and root and goal regions
				//root overlap check(each axis): min_obstacle<rootCartesian<max_obstacle
				//goal overlap check(each axis):max_obs>min_goal && min_obs<max_goal

				if( ( rootcart[i]<(obsCenter[i]+obsSize[i]/2.0) && rootcart[i]>(obsCenter[i]-obsSize[i]/2.0) )
						||
						((obsCenter[i]+obsSize[i]/2.0)>(goalcenter[i]-goalsize[i]/2.0)	&&
						(obsCenter[i]-obsSize[i]/2.0)<(goalcenter[i]+goalsize[i]/2.0)) ){
						obsVerified[i]=true;
						std::cout<<"-- not verified random obstacle in axis: "<<i<<std::endl;
				}
			}
		}while (obsVerified[0]==true && obsVerified[1]==true && obsVerified[2]==true);



//	obsCenter[1]=((double)rand()/(RAND_MAX/2.0)-1.0)*MAX_centerLim[1];
//	obsCenter[2]=((double)rand()/(RAND_MAX/2.0)-1.0)*MAX_centerLim[2];
//	cout<<obsCenter[0]<<" "<<obsCenter[1]<<" "<<obsCenter[2]<<endl;
	//save obstacle data in a file
	char stringIter[10];
	sprintf(stringIter, "%d", iterator);

	const char* DataLogPath	=DATALOGPATH;
	string DataLogPath2		=DATALOGPATH;
	int createDirVal=mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    //cout<<"return value: "<<DataLogPath2<<createDirVal<<std::endl;
	ofstream Myfile1;
	Myfile1.open ((DataLogPath2+"/"+stringIter+"Regions.txt").c_str(),ios::app);
	Myfile1 <<obsCenter[0]<<" "<<obsCenter[1]<<" "<<obsCenter[2]<<" ";
	Myfile1 <<obsSize[0]<<" "<<obsSize[1]<<" "<<obsSize[2]<<"\n";
	Myfile1.close();



	return 1;
}
