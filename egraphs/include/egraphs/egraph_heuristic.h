#ifndef EGRAPH_HEURISTIC_H
#define EGRAPH_HEURISTIC_H

#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<vector>

using namespace std;

/*
-has setGoal, setEGraph, getHeuristic function
-you can write your own! or use one of the standards I create
-a general N-dimensional dijkstra
--the constructor
---requires a vector of n dimension sizes (so we can allocate a grid), a vector of successors (each of length n), a vector of costs (of the successors)
---a function which returns a pointer to the collision grid (just a 1D array) or null if the grid hasn't changed
---a function which given a coordinate (an egraph vertex) returns a vector of length n (the down-projection)
--setGoal will get a new collision grid, wipe g-values from the last search, and insert the goal into the queue
--getHeuristic is an on-demand dijkstra which uses the given successors and the e-graph edges provided by the setEGraph function until queried cell is filled in
--setEGraph gets a list of edges in the e-graph. it turns these into n-dimensional edges by calling the down-project function on the endpoints of each edge
-a 2D, 8-connected uniform cost dijkstra (an optimized version of the n-dimensional version)
-a 3D, 26-connected uniform cost dijkstra (an optimized version of the n-dimensional version)
-an N-dimensional euclidean distance
*/

class EGraphHeuristic{

  public:
    //if the goal vector is empty it means to reuse the last goal
    virtual void setGoal(vector<double> goal) = 0;
    virtual int getHeuristic(vector<double> coord) = 0;
    virtual void getEGraphVerticesWithSameHeuristic(vector<double> coord, vector<EGraph::EGraphVertex*> vertices) = 0;

    virtual void runPrecomputations(){};

    void initialize(EGraph* eg){
      eg_ = eg;
    };

    void setEpsE(double e){
      epsE_ = e;
      vector<double> dummy_goal;
      setGoal(dummy_goal);
    };

  protected:
    EGraph* eg_;
    double epsE_;
};

#endif
