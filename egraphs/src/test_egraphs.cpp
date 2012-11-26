#include<egraphs/egraph.h>
#include<egraphs/egraph_down_project.h>
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

    virtual void getGoalHeuristicCoord(vector<double>& coord){
    };

    virtual bool getGoalCoord(vector<double>& parent, vector<double>& goal){
      return true;
    }

    virtual int getStateID(vector<double>& coord){
      return 0;
    };
};

class myDownProject : public EGraphDownProject{
  void downProject(vector<double> coord, vector<int>& dp){

  };
};

int main(int argc, char** argv){
  myEnv env;
  myDownProject dp;

  vector<string> names;
  vector<double> min;
  vector<double> max;
  vector<double> res;
  EGraph eg(min,max,res,names,0);
  EGraph2dGridHeuristic heur(&dp,100,100,1);

  AnytimeEGraphPlanner aegplanner(&env,true);
  aegplanner.initializeEGraph(&eg,&env,&heur);

  sleep(2.0);
  return 0;
}
