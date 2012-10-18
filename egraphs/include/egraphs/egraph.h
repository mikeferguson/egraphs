#ifndef EGRAPH_H
#define EGRAPH_H

#include<ros/ros.h>
#include<vector>
#include<string>

using namespace std;

class EGraph{
  public:

    class EGraphVertex{
      public:
        EGraphVertex(){
          id = -1;
          shortcutIteration = 0;
        };

        int id;
        vector<int> coord;
        //an adjacency list representing the graph (using the egraph ids)
        vector<EGraphVertex*> neighbors;
        vector<int> costs;

        vector<EGraphVertex*> shortcuts;
        vector<int> shortcut_costs;
        int shortcutIteration;
    };

    //constructor takes 4 vectors (min,max,res,names) which tells me the number of dimensions, how many values they can have, and the dimension names
    //load can be called after this in order bring up a stored E-Graph with different parameters than those stored in the file
    EGraph(vector<double>& min, vector<double>& max, vector<double>& resolution, vector<string>& names);

    //another constructor takes an egraph file to load
    //this will load the E-Graph using the parameters (min,max,resolution,names) stored in the file
    EGraph(string filename);

    //add path takes a vector of names, and a vector of vectors of doubles (the waypoints on the path), a vector of costs
    //this add the edges to the e-graph. 
    //no longer computes all-pairs! this simplifies the e-graph data structure and drops computation between queries to almost nothing. 
    //no longer needs to compute components!
    //finally this will call setEGraph on the EGraphable's EGraphHeuristic to prepare it for the next query
    bool addPath(vector<vector<double> >& coords, vector<int>& costs);

    //save egraph
    bool save(string filename);

    //load egraph
    bool load(string filename);

    //collision check
    void collisionCheck();

    void discToCont(vector<int> d, vector<double>& c);
    void contToDisc(vector<double> c, vector<int>& d);

    //an id to coordinate mapping
    vector<EGraphVertex*> id2vertex;
    EGraphVertex* getVertex(vector<int>& coord);
  protected:

    unsigned int inthash(unsigned int key);
    int getHashBin(vector<int>& coord);

    EGraphVertex* createVertex(vector<int>& coord);
    void addEdge(EGraphVertex* v1, EGraphVertex* v2, int cost);

    //a coordinate to id mapping
    //map<vector<double>,int> vertex2id;
    vector<vector<EGraphVertex*> > hashtable;

    vector<double> min_;
    vector<double> max_;
    vector<double> res_;
    vector<string> names_;
};

#endif
