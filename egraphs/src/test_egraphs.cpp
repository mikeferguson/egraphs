#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<egraphs/egraph_2d_grid_heuristic.h>
#include<egraphs/anytime_egraph_planner.h>
#include<sbpl/headers.h>

class myEnv : public EnvironmentNAVXYTHETALAT, public EGraphable{
    //requires a snap motion function between 2 coordinates. returns true if a transition exists (and then also fills out a cost and state id). 
    //this is used for snap motions as well as reading in demonstrations (so it's important for the environment to attempt to use motiom primitives first and then adaptive motions)
    virtual bool snap(vector<double> from, vector<double> to, int& id, int& cost){
      return false;
    };

    //requires a getCoord function which takes a state id (the ids the environment uses) and returns a vector with the coordinate so we can look for shortcuts in the e-graph
    //-we will never call getCoord on the goal (because it is possible we don't know what the goal state looks like)
    virtual bool getCoord(int id, vector<double>& coord){
      return true;
    };

    virtual int getStateID(vector<double>& coord){
      return 0;
    };

    //requires a getHeuristicGoalState function which returns the down-projected goal state which the e-graph heuristic uses
    virtual void getHeuristicGoalState(vector<double>& goal){
      
    };

    //requires a downProject function which takes a coordinate in the state space and projects it down into the simpler heuristic space
    virtual void downProject(vector<double> coord, vector<int>& dp){

    };
};

int main(int argc, char** argv){
  myEnv env;

  vector<string> names;
  vector<double> min;
  vector<double> max;
  vector<double> res;
  EGraph eg(min,max,res,names);
  EGraph2dGridHeuristic heur(100,100,1);

  AnytimeEGraphPlanner aegplanner(&env,true);
  aegplanner.initializeEGraph(&eg,&env,&heur);

  sleep(2.0);
  return 0;
}
