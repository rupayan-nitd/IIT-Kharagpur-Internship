/*
 * Created on 8th June, 2018
 * Experimental setup for P5: QoS-aware Cache Management in SDN
 *
 *
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-loss-model.h"

#include <bits/stdc++.h>
#include <string>
#include <unistd.h>
#include <fstream>
#include <time.h>

#include<math.h>
#include<utility>


using namespace ns3;
using namespace std;

/*********************** Simulation Settings *********************/

const int N=17; // Number of TCAMS
const int M=8;  // Number of SS 8,10,12,14,16
const int Q=N; // Maximum quota size equal to number of TCAMs
const int V=17; // Number of TCAMS
const int F=100; // Number of flows
const int U=256;  // Controller processing capacity (mu) unit??
const double p=0.819677; // probability of table-miss
const int C=1080000000; // speed of light in km/hr
const double Weight[3][1]={{0.3153},{0.6324},{0.0523}}; // a 2D array of weight for matrix multiplication of criteria comparison matrix and this weight matrix
const double MIN_DIST = 5000.00; // Min dist between TCAMs and SS
const double MAX_DIST = 18000.00; // Max dist between TCAMs and SS ~ range from ITZ (km)
const double MIN_LAMBDA = 10000;
const double MAX_LAMBDA = 30000; // TCAM ingress traffic arrival rate (req/sec)
const double MIN_COST_TS = 500;
const double MAX_COST_TS = 2000; // unitless? TCAM cost
const double MIN_COST_SS = 100;
const double MAX_COST_SS = 1600;
const double MAX_LATENCY = 38.00;
const double MIN_LATENCY = 11.00;
const int MIN_SS_QUOTA = 5;
const int MAX_SS_QUOTA = 10; // define quota?

double Run[10]={0}; // for running this prog. with varying #SS
int increment=0; //for index of Run array

typedef pair<int, double> iPair;

NS_LOG_COMPONENT_DEFINE ("Exp");

bool compare(const pair<double,int> &a, const pair<double,int> &b);
/*
 * Get two pairs and return true if key of 1st pair > key of 2nd pair
 * \param const pair a
 * \param const pair b
 * \returns boolean true if key of 1st pair > key of 2nd pair otherwise returns false
*/

bool compare(const pair<double,int> &a, const pair<double,int> &b)
{
    return (a.first>b.first);
}

class Tcam;
class SoftwareSwitch;
class Graph;
class Experiment;

class SoftwareSwitch
{
public:
    int qsize;
    int quota[Q];
    int prefsw[N];
    double distTS[N];
    double delay[N];
    double cost[N];
    map<int,int> mp;
    int lambda[N];
    SoftwareSwitch();
    /*
    * initializes data members
    */

    void display();
    /*
    * displays data members
    */

    friend void build_SoftwareSwitch_Topo(SoftwareSwitch S[]);
    /*
	* \ builds Software Switch Topology
	* \param SoftwareSwitch S array to get set of all Software Switches
    */

    friend void build_Pref_SoftwareSwitch(SoftwareSwitch S[], double Weight[][1]);
    /*
    * builds preference-lists of each software switch
    * \param SoftwareSwitch S array to get set of all software switches
    * \param Weight matrix of the criteria
    */

    friend void build_Tcam_Topo(Tcam T[], SoftwareSwitch S[]);
    /*
	* \ builds TCAM Topology
	* \param Tcam T array to get set of all TCAMs
    */

    friend void build_Pref_Tcam(Tcam T[]);
    /*
    * builds preference-lists of each Tcam
    * \param Tcam T array to get set of all TCAMs
    * \param Weight matrix of the criteria
    */

    friend int isFree(SoftwareSwitch ob);
    /*
    * Checks of SS ob is free or not
    * \param SoftwareSwitch object ob
    * \returns true if that SS is free else false
    */

    friend void stableMatching(SoftwareSwitch S[], Tcam T[]);
    /* does optimal mapping between all set of TCAMs and software switches
    * \param SoftwareSwitch S array to get set of all software switches
    * \param Tcam T array to get set of all TCAMs
    */

    friend void generateCost_StableTransfer(Tcam T[], SoftwareSwitch S[]);
    /*
    * sums up all cost values of each SS for all mapped TCAMs and finds total sum of all SSs
    * \param Tcam T array to get set of all TCAMs
    * \param SoftwareSwitch S array to get set of all software switches
    */

    friend void generateLambda_OneSoftwareSwitch(SoftwareSwitch S[], int one_SS);
    /*
    * sums up all lambda values of selected SS for all N TCAMs
    * \param SoftwareSwitch S array to get set of all software switches
    * \param randomly selected SS one_SS
    */

    friend void generateLambda_StableTransfer(Tcam T[], SoftwareSwitch S[]);
    /*
    * sums up all lambda values of each SS for all mapped TCAMs and finds their maximum
    * \param Tcam T array to get set of all TCAMs
    * \param SoftwareSwitch S array to get set of all software switches
    */

    friend void generateLambda_Random(Tcam T[], SoftwareSwitch S[]);
    /*
    * sums up all lambda values of each SS for all randomly mapped TCAMs and finds their maximum
    * \param Tcam T array to get set of all TCAMs
    * \param SoftwareSwitch S array to get set of all software switches
    */

    friend void transferMapping(Tcam T[], SoftwareSwitch S[]);
    /* applies coalitional game theory approach to apply transfer rule to get more stabilised Nash equilibrium mapping between TS and SS
    * \param SoftwareSwitch S array to get set of all software switches
    * \param Tcam T array to get set of all TCAMs
    */

    friend int oneSoftwareSwitch(Tcam T[], SoftwareSwitch S[]);
    /* does mapping between set of all TCAMs and only SS selected randomly
    * \param Tcam T array to get set of all TCAMs
    */

    friend class Experiment;

    friend void createTopology(Tcam *T, SoftwareSwitch *S, Graph g);
    /*
	 * Deploy the APs  according to the described topology in a grid starting from (9.45,9.45)
	 * The STAs are randomly deployed in a 2D rectangle (0,0),(0,35.45),(35.45,0),(35.45,35.45)
	 *
	*/

    friend void runExperiment(Tcam *T, SoftwareSwitch *S, Graph g);
    /*
	* \ runs the required Experiment
    */
};

SoftwareSwitch :: SoftwareSwitch()
{
	//NS_LOG_DEBUG("SoftwareSwitch constructor...");
	qsize=0;
    for(int i=0;i<N;i++)
        prefsw[i]=-1;
    for(int i=0;i<N;i++)
        distTS[i]=0.0;
    for(int i=0;i<N;i++)
        delay[i]=0.0;
    for(int i=0;i<N;i++)
        cost[i]=0.0;
    for(int i=0;i<Q;i++)
        quota[i]=-1;
    for(int i=0;i<N;i++)
        lambda[i]=0;
}

void SoftwareSwitch :: display()
{
	cout << "Preference-List : ";
	for(int i=0;i<N;i++)
        cout << prefsw[i] << " ";
    cout << endl << "Quota : ";
    for(int i=0;i<qsize;i++)
        cout << quota[i] << " ";
    cout << endl << "Preference indexes" << endl;
    map<int,int> :: iterator it;
    for(it=mp.begin();it!=mp.end();it++)
        cout << it->first << "->" << it->second;
    cout << endl << "Distances for all TCAMs" << endl;
    for(int i=0;i<N;i++)
        cout << distTS[i] << " ";
    cout << endl << "Delays for all TCAMs" << endl;
    for(int i=0;i<N;i++)
        cout << delay[i]*3600*1000 << " ";
    cout << endl << "Costs for all TCAMs" << endl;
    for(int i=0;i<N;i++)
        cout << cost[i] << " ";
    cout << endl << "Lambda values" << endl;
    for(int i=0;i<N;i++)
        cout << lambda[i] << " ";
    cout << endl;
}


class Tcam
{
public:
    int ss; // SS to which this TCAM is mapped
    int preft[M];
    double dist[N]; // distance of this TCAM from all other TS
    double distSS[M]; // distance of this TCAM from all other SS
    double delay[M]; // delay incurred of this TCAM from all other SS
    double cost[M]; // delay incurred of this TCAM from all other SS
    map<int, int> mpt; // map with key as the SS and value as its index in the preft array to get which SS comes first in the preft array 

    Tcam();
    /*
    * initializes data members
    */

    void display();
    /*
    * displays data members
    */

    friend void build_Tcam_Topo(Tcam T[], SoftwareSwitch S[]);

    friend void build_SoftwareSwitch_Topo(SoftwareSwitch S[]);

    friend void build_Pref_Tcam(Tcam T[]);

    friend void build_Pref_SoftwareSwitch(SoftwareSwitch S[]);

    friend void stableMatching(SoftwareSwitch S[], Tcam T[]);

    friend void randomMapping(Tcam T[], SoftwareSwitch S[]);
    /* does random mapping between all set of TCAMs and software switches only for TCAMs
    * \param Tcam T array to get set of all TCAMs
    */

    friend void transferMapping(Tcam T[], SoftwareSwitch S[]);

    friend void generateFlows(Graph g, Tcam T[]);
    /* generates F number of flows between set of all TCAMs by selecting source and destination randomly
    * \param Graph g for all set of TCAMs
    * \param Tcam T array to get set of all TCAMs
    */

    friend void generateFlows2(Graph g, Tcam T[]);
    /* generates F number of flows between set of all TCAMs by selecting source and destination randomly
    * \param Graph g for all set of TCAMs
    * \param Tcam T array to get set of all TCAMs
    */
    
    friend int oneSoftwareSwitch(Tcam T[], SoftwareSwitch S[]);

    friend void generateLambda_OneSoftwareSwitch(SoftwareSwitch S[], int one_SS);

    friend void generateCost_StableTransfer(Tcam T[], SoftwareSwitch S[]);

    friend void generateLambda_StableTransfer(Tcam T[], SoftwareSwitch S[]);

    friend void generateLambda_Random(Tcam T[], SoftwareSwitch S[]);

    friend class Experiment;

    friend void createTopology(Tcam *T, SoftwareSwitch *S, Graph g);

    friend void runExperiment(Tcam *T, SoftwareSwitch *S, Graph g);
};

Tcam :: Tcam()
{
	//NS_LOG_DEBUG("TCAM constructor...");
	ss=-1;
    for(int i=0;i<M;i++)
        preft[i]=-1;
    for(int i=0;i<N;i++)
        dist[i]=0.0;
    for(int i=0;i<M;i++)
        distSS[i]=0.0;
    for(int i=0;i<M;i++)
        delay[i]=0.0;
    for(int i=0;i<M;i++)
        cost[i]=0.0;
}

void Tcam :: display()
{
	cout << endl << "Preference-List : ";
	for(int i=0;i<M;i++)
        cout << preft[i] << " ";
    cout << endl;
    cout << "Mapped with Software Switch " << ss << endl << "Distances for other TCAMS" << endl;
    for(int i=0;i<N;i++)
        cout << dist[i] << " ";
    cout << endl << "Distances for all Software Switches" << endl;
    for(int i=0;i<M;i++)
        cout << distSS[i] << " ";
    cout << endl << "Delays" << endl;
    for(int i=0;i<M;i++)
        cout << delay[i]*60*1000 << " "; // in milli seconds
    cout << endl << "Costs" << endl;
    for(int i=0;i<M;i++)
        cout << cost[i] << " ";
    cout << endl;
}


class Graph
{
	list< pair<int, double> > *adj;// In a weighted graph, we need to store vertex and weight pair for every edge
public:
    Graph();
    /*
    * initializes data members and Allocates memory for adjacency list
    */

    void addEdge(int u, int v, double w);
    /*
    * function to add an edge to graph
    * \param u a node
    * \param v another node
    * \param w weight between u and v
    */

    void shortestPath(int s, Tcam T[]);
    /* prints shortest path from s to all other TS
    * \param s as source
    * \param Tcam T array of set of all Tcams
    */

    friend void generateFlows(Graph g, Tcam T[]);

    friend void generateFlows2(Graph g, Tcam T[]);

    friend class Experiment;

    friend void createTopology(Tcam *T, SoftwareSwitch *S, Graph g);

	friend void runExperiment(Tcam *T, SoftwareSwitch *S, Graph g);
};

Graph :: Graph()
{
    adj = new list<iPair> [V];
}

void Graph :: addEdge(int u, int v, double w)
{
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w));
}

void Graph :: shortestPath(int src, Tcam T[])
{
    // priority queue to store vertices that are being prepropagationed.
    priority_queue< iPair, vector <iPair> , greater<iPair> > pq;
    vector<double> dist(V, DBL_MAX);//vector for distances
    pq.push(make_pair(0, src));
    dist[src] = 0;

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        // 'i' is used to get all adjacent vertices of a vertex
        list< pair<int, double> > :: iterator it;
        for (it = adj[u].begin(); it != adj[u].end(); ++it)
        {
            int v = (*it).first;//Get vertex label and weight of current adjacent of u.
            int weight = (*it).second;
            if (dist[v] > dist[u] + weight)// If there is shorted path to v through u.
            {
                dist[v] = dist[u] + weight;// Updating distance of v
                pq.push(make_pair(dist[v], v));
            }
        }
    }
    for (int i = 0; i < V; ++i)//shortest distances stored in dist[]
        T[src].dist[i]=dist[i];
}

class Experiment
{

public:

	Experiment();
	/*
	* \ default constructor
	*/

	void calCloseness(double arr[][3], double Closeness[], int m, int n);
	/*
	 * Calculate Closeness to get the Preference List
	 * \param array arr to get distance delay and cost aggregated
	 * \param Weight 2D array to get weight matrix
	 * \param Closeness to get closeness value
	 * \param m number of alternatives
	 * \param n number of criteria
	*/

	void getDistance(double distance[], int size);
	/*
	 * Get the distance between TS and SS
	 * \param distance array to store distances between TS and SS
	 * \param size to hold size of diatance
	*/

	void getQueueDelayTCAM(double queueDelay[], int lambda[]);
	/*
	 * Get the Queue Delay between TS and SS
	 * \param queueDelay array to store queue delay
	 * \param i to get ith TCAM TS
	*/

	void getPropagationDelay(double propagationDelay[], double distance[], int size);
	/*
	 * Get the Propagation Delay between TS and SS
	 * \param propagationDelay array to store Propagation delay between TS and SS
	 * \param distance array to store distances between TS and SS
	 * \param size to get the size of the arrays
	*/

	void getDelay(double delay[], double propagationDelay[], double queueDelay[], int size);
	/*
	 * Get the Total Delay between TS and SS for Tcam
	 * \param delay array to store total delay between TS and SS for Tcam
	 * \param propagationDelay array to store Propagation delay between TS and SS
	 * \param queueDelay array to store queue delay
	 * \param size to get the size of the arrays
	*/

	void getCostTCAM(double cost[]);
	/*
	 * Get the cost between TS and SS for TCAM
	 * \param cost array to store TCAM cost
	 * \param i to get ith TCAM TS
	*/

	void getCostSOFTWARESWITCH(double cost[]);
	/*
	 * Get the cost between TS and SS for SS
	 * \param cost array to store SS cost
	 * \param i to get ith SS
	*/

	friend void createTopology(Tcam *T, SoftwareSwitch *S, Graph g);
	
	friend void runExperiment(Tcam *T, SoftwareSwitch *S, Graph g);
	/*
	 * Print all the required statistics
	*/
};

Experiment::Experiment()
{
	//NS_LOG_DEBUG("Experiment constructor...");
}

void Experiment :: getDistance(double distance[], int size)
{
	//srand(time(NULL));
    for(int d=0;d<size;d++)
    {
        double seed = MIN_DIST + ((double)rand()) / ((1+(double)RAND_MAX) / (MAX_DIST - MIN_DIST + 1));
        distance[d]=seed;
    }
}

void Experiment :: getQueueDelayTCAM(double queueDelay[], int lambda[])
{
    double l=0.0,z=0.0;
    //cout << "Enter lambda and meu value respectively associated with every Software Switch for the TCAM " << i << endl;
    //srand(time(NULL));
    for(int d=0;d<M;d++)
    {
    	//u=200000 + rand() / (RAND_MAX / (260000 - 200000 + 1) + 1);
        l=MIN_LAMBDA + rand() / (RAND_MAX / (MAX_LAMBDA - MIN_LAMBDA + 1) + 1);// random numbers for lambda
        lambda[d]=l;
        z=1/(U-l);
        queueDelay[d]=z/3600;
    }
}

void Experiment :: getPropagationDelay(double propagationDelay[], double distance[], int size)
{
    for(int d=0;d<size;d++)
        propagationDelay[d]=distance[d]/C; // hour
}

void Experiment :: getDelay(double delay[], double propagationDelay[], double queueDelay[], int size)
{
    for(int d=0;d<size;d++)
        delay[d]=queueDelay[d]+propagationDelay[d];
}

void Experiment :: getCostTCAM(double cost[])
{
    for(int d=0;d<M;d++)
        cost[d]=MIN_COST_TS + rand() / (RAND_MAX / (MAX_COST_TS - MIN_COST_TS + 1) + 1);
}

void Experiment :: getCostSOFTWARESWITCH(double cost[])
{
    for(int d=0;d<N;d++)
        cost[d]=MIN_COST_SS + rand() / (RAND_MAX / (MAX_COST_SS - MIN_COST_SS + 1) + 1);
}

void Experiment :: calCloseness(double arr[][3], double Closeness[], int m, int n)
{
    int i,j;
    double arr1[m]={0};
    for(i=0;i<n;i++)
    {
        for(j=0;j<m;j++)
            arr1[i]+=arr[j][i];//STORING COLUMN SUMS
    }
    //NORMALIZATION
    for(i=0;i<m;i++)
    {
        for(j=0;j<n;j++)
            arr[i][j]=arr[i][j]/arr1[j];
    }
    for(i=0;i<n;i++)//CALCULATION OF WEIGHT MATRIX
    {
        for(j=0;j<m;j++)
            arr[j][i]=arr[j][i]*Weight[i][0];
    }
    double mini,maxi;
    vector<double> ideal, nideal;
    for(i=0;i<n;i++)//MINIMUM OF EACH COLUMN IS IDEAL AND MAXIMUM IS NIDEAL
    {
        maxi=INT_MIN;
        mini=INT_MAX;
        for(j=0;j<m;j++)
        {
            maxi=max(maxi,arr[j][i]);
            mini=min(mini,arr[j][i]);
        }
        ideal.push_back(mini);
        nideal.push_back(maxi);
    }
    //SEPARATION measure
    double SiPlus[m]={0};
    double SiMinus[m]={0};
    for(i=0;i<m;i++)
    {
        for(j=0;j<n;j++)
        {
            SiPlus[i]+=pow((arr[i][j]-ideal[j]), 2);
            SiMinus[i]+=pow((arr[i][j]-nideal[j]), 2);
        }
        SiPlus[i]=pow(SiPlus[i],0.5);
        SiMinus[i]=pow(SiMinus[i],0.5);
    }
    //CALCULATE CLOSENESS
    double C[m]={0};
    for(i=0;i<m;i++)
        C[i]=SiMinus[i]/(SiPlus[i]+SiMinus[i]);
    for(i=0;i<m;i++)
        Closeness[i]=C[i];
}

void build_Tcam_Topo(Tcam T[], SoftwareSwitch S[])
{
	//NS_LOG_DEBUG("Building TCAM Topology");

	int i,j;
	for(i=0;i<N;i++) // initializing all TCAMs
		T[i].ss=-1;

	for(i=0;i<N;i++)
    {
		double distance[M]={0};
	    double delay[M]={0};
	    double cost[M]={0};
	    double propagationDelay[M]={0};
	    double queueDelay[M]={0};
        int lambda[M]={0};

	    Experiment ex;
	    ex.getDistance(distance, M);
	    ex.getPropagationDelay(propagationDelay, distance, M);
	    ex.getQueueDelayTCAM(queueDelay, lambda);
	    ex.getDelay(delay, propagationDelay, queueDelay, M);
	    ex.getCostTCAM(cost);

	    for(j=0;j<M;j++)
	    {
	    	T[i].distSS[j]=distance[j];
	        T[i].delay[j]=delay[j];
	        T[i].cost[j]=cost[j];
            S[j].lambda[i]=lambda[j];
	    }
	}
}

void build_SoftwareSwitch_Topo(SoftwareSwitch S[])
{
	//NS_LOG_DEBUG("Building SS Topology");
    
	int i,j,h;
	for(i=0;i<M;i++)// initializing all Software Switches
    {
    	for(h=0;h<Q;h++)
    		S[i].quota[h]=-1;
    }

	for(i=0;i<M;i++)
    {
    	S[i].qsize=17; //3 + rand() / (RAND_MAX / (12 - 3 + 1) + 1);
        for(int j=0;j<S[i].qsize;j++)
            S[i].quota[j]=-1;
        double distance[N]={0};
        double propagationDelay[N]={0};
        double cost[N]={0};
        Experiment ex;
        ex.getDistance(distance, N);
        ex.getPropagationDelay(propagationDelay, distance, N);
        ex.getCostSOFTWARESWITCH(cost);
        for(j=0;j<N;j++)
	    {
	    	S[i].distTS[j]=distance[j];
	        S[i].delay[j]=propagationDelay[j];
	        S[i].cost[j]=cost[j];
	    }
    }
}

void build_Pref_Tcam(Tcam T[])
{
	//NS_LOG_DEBUG("Building Tcam Preferences");
    int i,v;
    for(i=0;i<N;i++) // initializing all TCAMs
		T[i].ss=-1;

    for(i=0;i<N;i++)
    {
        double Closeness[M]={0};
        double arr[M][3]={0}; // matrix containing distance, delay and cost value in 0 1 and 2 column resp.        

        for(v=0;v<M;v++)
        {
            arr[v][0]=T[i].distSS[v];
            arr[v][1]=T[i].delay[v];
            arr[v][2]=T[i].cost[v];
        }

        Experiment ex;
        ex.calCloseness(arr,Closeness, M, 3);

        vector<pair<double,int> > vect;
        for(int v=0;v<M;v++)
            vect.push_back(make_pair(Closeness[v],v)); // pair of closeness and associated TCAM
        sort(vect.begin(), vect.end(), compare); // sorting in descending order acc. to closeness

        for(int v=0;v<M;v++)
        {
            T[i].preft[v]=vect[v].second; // keeping the SW acc to closeness value in descending order
            T[i].mpt[T[i].preft[v]]=v; // map with key as the SW and value as its index, This is done to check later the index of SW in the pref list of TCAM in transfer function
        }
    }
}

void build_Pref_SoftwareSwitch(SoftwareSwitch S[])
{
    int i,v,h;
    for(i=0;i<M;i++)// initializing all Software Switches
    {
    	for(h=0;h<Q;h++)
    		S[i].quota[h]=-1;
    }

    for(i=0;i<M;i++)
    {        
        double Closeness[N]={0};
        double arr[N][3]={0};        

        for(v=0;v<N;v++)
        {
            arr[v][0]=S[i].distTS[v];
            arr[v][1]=S[i].delay[v];
            arr[v][2]=S[i].cost[v];
        }

        Experiment ex;
        ex.calCloseness(arr, Closeness, N, 3);

        vector<pair<double,int> > vect;
        for(int v=0;v<N;v++)
            vect.push_back(make_pair(Closeness[v],v)); // pair of closeness and associated TCAM
        sort(vect.begin(), vect.end(), compare); // sorting acc. to score or closeness in descending order

        for(int v=0;v<N;v++)
        {
            S[i].prefsw[v]=vect[v].second; // keeping the TS acc to closeness value in descending order
            S[i].mp[S[i].prefsw[v]]=v;
        }
    }
}

int isFree(SoftwareSwitch ob)
{
    for(int i=0;i<ob.qsize;i++)
        if(ob.quota[i] == -1)// if -1 present
            return 1;
    return 0;
}

void stableMatching(Tcam T[], SoftwareSwitch S[])
{
    int i,x,k;
    bool tFree[N];
    for (int i = 0; i < N; i++)
        tFree[i]=false;
    int freeCount = N;
    while(freeCount > 0) // loops untill all TCAMs gets accepted
    {
        for(x=0;x<M;x++)// from 0 till Mth proposal of all TCAMS
        {
            for (i = 0; i < N; i++)
            {
                if(tFree[i] == true)//ith TCAM gets accepted
                    continue;
                int s = T[i].preft[x];
                if(isFree(S[s]))// if this SW is free, that is, has -1 at the end somewhere
                {
                    int j=0,key=i;
                    while(S[s].quota[j] != -1 && S[s].mp[ S[s].quota[j] ] < S[s].mp[key])// j increases untill this TCAM has less priority than other TCAMs starting from 0th index
                        j++;
                    for(k=S[s].qsize-1;k>j;k--)//shifting all TCAMS one step ahead to make space for this new TCAM tobe inserted
                        S[s].quota[k] = S[s].quota[k-1];

                    S[s].quota[j] = key;
                    tFree[i] = true;
                    T[i].ss=s;// associating this SW with this TCAM
                    freeCount--;// one TCAM gets accepted
                    if(freeCount<=0)
                        break;
                }
                else
                {
                    int key=i,j=0;
                    if(S[s].mp[ S[s].quota[S[s].qsize-1] ] > S[s].mp[key])
                    {
                        tFree[ S[s].quota[ S[s].qsize - 1 ] ] = false;
                        T[S[s].quota[ S[s].qsize - 1]].ss=-1; // last TCAM has no SW associated
                        S[s].quota[ S[s].qsize - 1 ] = -1;
                        while(S[s].quota[j] != -1 && S[s].mp[ S[s].quota[j] ] < S[s].mp[key])
                            j++;
                        for(k=S[s].qsize-1;k>j;k--)// similar sorted insert
                            S[s].quota[k] = S[s].quota[k-1];

                        S[s].quota[j] = key;
                        tFree[key] = true;
                        T[i].ss=s;
                    }
                }
            }
            if(freeCount<=0)
                break;
            if(x==M-1)
                break;
        }
        if(x==M-1)// even if all TCAMs not get accepted the loop will break when all TCAMs proposes to all SW
            break;
    }
    //cout << endl;
    /*cout << "SoftwareSwitch      Tcams In Quota" << endl;
    for (int i = 0; i < M; i++)
    {
        cout << "      " << i << "           \t\t";
        S[i].printQuota();
        cout << endl;
    }*/

}

void transferMapping(Tcam T[], SoftwareSwitch S[])
{
    int i,j,k,l,t, comp, select_sw, key, flag;
    for(i=0;i<M;i++)// looping for each SW
    {
        for(k=S[i].qsize-1;k>=0;k--)// looping for each TCAM in ith SW from back of the quota array
        {
            flag=0;
            t=S[i].quota[k];
            if(t==-1) // if no TCAM
                continue;
            comp=T[t].delay[i]; // taking delay of TCAM t with ith SW
            for(j=0;j<M;j++)
            {
                if(isFree(S[j]) && T[t].delay[j]<comp)// checking if there is any free SW which has less delay associated with TCAM t
                {
                    flag=1;
                    comp=T[t].delay[j];
                    select_sw=j;
                }
            }

            if(flag!=0) // if there is no such SW
            {
                T[t].ss=select_sw;
                for(l=k;l<S[i].qsize-1;l++)// shifting all other TCAMs from last one position before
                    S[i].quota[l]=S[i].quota[l+1];
                S[i].quota[l]=-1;

                j=0;key=t;
                while(S[select_sw].quota[j] != -1 && S[select_sw].mp[ S[select_sw].quota[j] ] < S[select_sw].mp[key])// similarly as in optimal mapping function
                    j++;
                for(l=S[select_sw].qsize-1;l>j;l--) // similar sorted insert in selected SW acc. to the pref list of the selected SW
                    S[select_sw].quota[l] = S[select_sw].quota[l-1];

                S[select_sw].quota[j] = key;
            }
        }
    }
}

void randomMapping(Tcam T[], SoftwareSwitch S[])
{
    int softswitch[M];
    for(int i=0;i<M;i++)
        softswitch[i]=i;
    //srand ( time(NULL) ); //initialize the random seed
    for(int i=0;i<N;i++)
    {
    	int RandIndex = rand() % M;
        int SS=softswitch[RandIndex];

    	T[i].ss=SS; // associating SW
    	int j=0,key=i; // Now associating this TCAM to the Software Switch SS
        while(S[SS].quota[j] != -1)// j increases untill there is space in the quota
            j++;
        S[SS].quota[j] = key;
    }
}

int oneSoftwareSwitch(Tcam T[], SoftwareSwitch S[])
{
	//srand(time(NULL));
    int softswitch[M];
    for(int i=0;i<M;i++)
        softswitch[i]=i;
    int RandIndex = rand() % M;
    int sw=softswitch[RandIndex];
    for(int i=0;i<N;i++)
    {        
        T[i].ss=sw; // associating SW
        int j=0,key=i; // Now associating this TCAM to the Software Switch SS
        while(S[sw].quota[j] != -1)// j increases untill there is space in the quota
            j++;
        S[sw].quota[j] = key;
        //T[i].display();
    }
    return sw;
}

void generateCost_OneSoftwareSwitch(SoftwareSwitch S[], int one_SS)
{
	int i;
    long double sum2=0, allsum2=0;
    for(i=0;i<N;i++)
        sum2 += S[one_SS].cost[i];
    allsum2=sum2;

    //NS_LOG_INFO(allsum2);
    Run[increment++] = allsum2;
}

void generateCost_Random(Tcam T[], SoftwareSwitch S[])
{
	int i,j;
    long double sum1=0.0, maxsum1=0.0;
    for(i=0;i<M;i++)
    {
    	sum1=0;
        for(j=0;j<Q;j++)
        {
            if(S[i].quota[j]==-1)
                break;
            sum1 += S[i].cost[ S[i].quota[j] ];
        }
        maxsum1 = max(maxsum1,sum1);
    }

    //NS_LOG_INFO(maxsum1);
    Run[increment++] = maxsum1;
}

void generateCost_StableTransfer(Tcam T[], SoftwareSwitch S[])
{
	int i,j;
    long double sum1=0.0, maxsum1=0.0;
    for(i=0;i<M;i++)
    {
    	sum1=0;
        for(j=0;j<S[i].qsize;j++)
        {
            if(S[i].quota[j]==-1)
                break;
            sum1 += S[i].cost[ S[i].quota[j] ];
        }
        maxsum1 = max(maxsum1,sum1);
    }

    //NS_LOG_INFO(maxsum1);
    Run[increment++] = maxsum1;
}

void generateLambda_OneSoftwareSwitch(SoftwareSwitch S[], int one_SS)
{
    int i;
    long long int sum2=0, maxsum2=0;
    for(i=0;i<N;i++)
        sum2 += S[one_SS].lambda[i];
    maxsum2=sum2;

    //NS_LOG_INFO(maxsum2/1000);
    Run[increment++] = maxsum2/1000;
}

void generateLambda_StableTransfer(Tcam T[], SoftwareSwitch S[])
{
    int i,j;
    long long int sum1=0, maxsum1=0;
    for(i=0;i<M;i++)
    {
    	sum1=0;
        for(j=0;j<S[i].qsize;j++)
        {
            if(S[i].quota[j]==-1)
                break;
            sum1 += S[i].lambda[ S[i].quota[j] ];
        }
        maxsum1=max(maxsum1, sum1);
    }

    //NS_LOG_INFO(maxsum1/1000);
    Run[increment++] = maxsum1/1000;
}

void generateLambda_Random(Tcam T[], SoftwareSwitch S[])
{
    int i,j;
    long long int sum1=0, maxsum1=0;
    for(i=0;i<M;i++)
    {
    	sum1=0;
        for(j=0;j<Q;j++)
        {
            if(S[i].quota[j]==-1)
                break;
            sum1 += S[i].lambda[ S[i].quota[j] ];
        }
        maxsum1=max(maxsum1, sum1);
    }

    //NS_LOG_INFO(maxsum1/1000);
    Run[increment++] = maxsum1/1000;
}

void generateFlows(Graph g, Tcam T[])
{
    int i;
    double allDelay=0.0,delayHit=0.0,delayMiss=0.0,finalDelay=0.0;
    int arr[V];
    for(i=0;i<V;i++)
        arr[i]=i;
    //srand ( time(NULL) ); //initialize the random seed
    for(i=1;i<=F;i++)
    {
        int RandIndex1 = rand() % V;// generate betw. 0 and (V-1)
        int RandIndex2 = rand() % V;
        int src=arr[RandIndex1];
        int dst=arr[RandIndex2];
        //cout << "Src " << src << " Destn. " << dst << " \n";
        if(src==dst || src>16 || dst >16)
            i--;
        else
        {
            delayHit=T[src].dist[dst]/C; // considering TCAM TCAM distance
            delayMiss=T[src].delay[T[src].ss]; // considering TCAM SW delay
            finalDelay=(1-p)*delayHit + p*delayMiss;
            allDelay+=finalDelay;
        }
    }
    
    //NS_LOG_INFO(allDelay*3600*1000/F);
    Run[increment++] = (double)(allDelay*3600*1000/F);
    //cout << "\n";    
}

void generateFlows2(Graph g, Tcam T[])// after stable transfer
{
    int i,j;
    double allDelay=0.0, delayHit=0.0, delayMiss=0.0, finalDelay=0.0;
    int arr[V];
    for(i=0;i<V;i++)
        arr[i]=i;
    //srand ( time(NULL) ); //initialize the random seed
    for(i=1;i<=F;i++)
    {
        int RandIndex1 = rand() % V;// generate betw. 0 and (V-1)
        int RandIndex2 = rand() % V;
        int src=arr[RandIndex1];
        int dst=arr[RandIndex2];
        //cout << "Src " << src << " Destn. " << dst << " \n";
        if(src==dst || src>16 || dst >16)
            i--;
        else
        {
            delayHit=(T[src].dist[dst]/C)*3600*1000; // considering TCAM TCAM distance and get delay in ms
            double sum=0.0, avg=0.0;
            for(j=0;j<M;j++) // sum of all distances for src Ts of all SS 
	            sum += T[src].distSS[j];
	        avg=sum/M; // taking avg dist and placing the controller there
	        double seed = MIN_LATENCY + ((double)rand()) / ((1+(double)RAND_MAX) / (MAX_LATENCY - MIN_LATENCY + 1)); // in ms
            delayMiss=(avg/C)*3600*1000 + seed; // considering TCAM SW delay in ms
            finalDelay=(1-p)*delayHit + p*delayMiss;
            allDelay+=finalDelay;
        }
    }
    
    //NS_LOG_INFO(allDelay/F);
    Run[increment++] = allDelay/F;
    //cout << "\n";
}


int main (int argc, char *argv[])
{
	int seed;
	
	CommandLine cmd;
	cmd.AddValue("seed", "Seed value for random values", seed);
	cmd.Parse (argc, argv);
	srand(seed);

	LogComponentEnable("Exp", LOG_LEVEL_INFO);

	Experiment exp;
	Tcam *T = new Tcam[N];
	SoftwareSwitch *S = new SoftwareSwitch[M];
	Graph g;
	createTopology(T, S, g);

    increment=0;

    double sum=0.0;


	Simulator::Stop (Seconds (90.0));
	Simulator::Run ();
	Simulator::Destroy ();

    for(int i=0; i<10; i++) // calculating sum of all 10 runs
    {
        sum+=Run[i];
        //NS_LOG_INFO(Run[i]);
    }

    NS_LOG_INFO(sum/10);

	return 0;
}


void runExperiment(Tcam *T, SoftwareSwitch *S, Graph g)
{
	//NS_LOG_DEBUG(" \n Running Expt...\n");
	//srand( (unsigned) time(NULL) * getpid());

    // varying Topology each time of run only for generating Lambda and Cost not in case of delay which is done only once in createTopology fn.
	build_Tcam_Topo(T, S);
    build_SoftwareSwitch_Topo(S);

	build_Pref_Tcam(T);
    build_Pref_SoftwareSwitch(S);

    stableMatching(T, S);
    //generateFlows(g, T);
    //generateFlows2(g, T);
    transferMapping(T, S);
    //generateFlows(g, T);
    generateFlows2(g, T);
    //generateLambda_StableTransfer(T, S);
    //generateCost_StableTransfer(T, S);

    //randomMapping(T, S);
    //generateFlows(g, T);
    //generateFlows2(g, T);
    //generateLambda_Random(T, S);
    //generateCost_Random(T, S);

    //int one_SS=oneSoftwareSwitch(T, S);
    //one_SS++; // it is nothing. Was just giving error that one_SS is not used anywhere so used it here...
    //generateFlows(g, T);
    //generateFlows2(g, T);
    //generateLambda_OneSoftwareSwitch(S, one_SS); // uncomment line 1135
    //generateCost_OneSoftwareSwitch(S, one_SS);
    /*for (int i = 0; i < N; ++i)
    	T[i].display();*/

    //sleepcp(600);
}

void createTopology(Tcam *T, SoftwareSwitch *S, Graph g)
{
	//NS_LOG_DEBUG("Creating topology...");
    ifstream ip("/home/rupayan/ns3/ns-allinone-3.26/ns-3.26/scratch/final.csv");

    if(!ip.is_open())
        cout << "ERROR: File Open" << '\n';
    while(ip.good())
    {
        string source="", destination="", weight="";
        int src=0, dest=0;
        getline(ip,source,',');
        getline(ip,destination,',');
        getline(ip,weight,'\n');
        stringstream geek(source);
        geek >> src;
        stringstream geeks(destination);
        geeks >> dest;
        //cout << src << dest << endl;
        g.addEdge(src, dest, strtod(weight.c_str(), NULL));
    }

    for(int i=0;i<V;i++)
    {
        g.shortestPath(i,T);
        //T[i].display();
    }
    ip.close();

    //build topo only once
    /*build_Tcam_Topo(T, S);
    build_SoftwareSwitch_Topo(S);*/

	for(int i=0; i<100; i = i + 10){
	 	Simulator::Schedule(Seconds(i), &runExperiment, T, S, g);
	}

	
}