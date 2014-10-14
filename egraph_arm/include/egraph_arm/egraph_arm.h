#pragma once
#include <ros/ros.h>
#include <sbpl_arm_planner/environment_robarm3d.h>
#include <egraphs/egraph.h>
#include <egraphs/egraphable.h>
#include <egraphs/egraph_discretize.h>
#include <vector>
#include <pviz/pviz.h>
#include<egraph_vis/egraph_visualizer.h>

namespace sbpl_arm_planner {
  class EGraphArm : public EnvironmentROBARM3D, 
  public EGraphable<std::vector<int> >, 
  public EGraphDiscretize, public EGraphMarkerMaker {
    public:
      EGraphArm(OccupancyGrid *grid, 
          RobotModel *rmodel, 
          CollisionChecker *cc, 
          ActionSet* as, PlanningParams *pm);
      bool snap(const std::vector<double>& from, const std::vector<double>& to, int& id, int& cost);
      bool getCoord(int id, std::vector<double>& coord);
      void projectGoalToHeuristicSpace(std::vector<int>& coord) const;
      bool isGoal(int id);
      int getStateID(const std::vector<double>& coord);
      bool isValidVertex(const std::vector<double>& coord);
      bool isValidEdge(const std::vector<double>& start, const std::vector<double>& end, bool& changed_cost, int& cost);
      void projectToHeuristicSpace(const std::vector<double>& coord, std::vector<int>& dp) const;
      void discToCont(const std::vector<int>& d, std::vector<double>& c);
      void contToDisc(const std::vector<double>& c, std::vector<int>& d);
      visualization_msgs::MarkerArray stateToVisualizationMarker(std::vector<double> coord);
      visualization_msgs::MarkerArray stateToDetailedVisualizationMarker(std::vector<double> coord);
      visualization_msgs::MarkerArray edgeToVisualizationMarker(std::vector<double> coord, std::vector<double> coord2);
      //void displayPath(const DisplayEdges& edges);
      // remove me!
      int getGoalHeuristic(int state_id);
      //int getGoalHeuristic(int x, int y, int z);
    private:
      //PViz pviz_;
      void printVector(std::vector<int> v);
      void printVector(std::vector<double> v);
  };
}
