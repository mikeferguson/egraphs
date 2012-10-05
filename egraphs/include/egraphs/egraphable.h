#ifndef EGRAPHABLE_H
#define EGRAPHABLE_H

#include<vector>
#include<string>

using namespace std;

class EGraphable{
  public:
    //requires save and load environment functions???? (we need this if we want to optimize the e-graph offline)
    virtual bool save(string filename){return false;};
    virtual bool load(string filename){return false;};

    //requires a snap motion function between 2 coordinates. returns true if a transition exists (and then also fills out a cost and state id). 
    //this is used for snap motions as well as reading in demonstrations (so it's important for the environment to attempt to use motiom primitives first and then adaptive motions)
    virtual bool snap(vector<double> from, vector<double> to, int& id, int& cost) = 0;

    //requires a getCoord function which takes a state id (the ids the environment uses) and returns a vector with the coordinate so we can look for shortcuts in the e-graph
    //-we will never call getCoord on the goal (because it is possible we don't know what the goal state looks like)
    virtual bool getCoord(int id, vector<double>& coord) = 0;

    virtual int getStateID(vector<double>& coord) = 0;

    //requires a getHeuristicGoalState function which returns the down-projected goal state which the e-graph heuristic uses
    virtual void getHeuristicGoalState(vector<double>& goal) = 0;

    //requires a downProject function which takes a coordinate in the state space and projects it down into the simpler heuristic space
    virtual void downProject(vector<double> coord, vector<int>& dp) = 0;
    
    //in the future we could ask them to implement features for their environment....we could have some built in ones like 2D patch and 3D patch
};

#endif
