#include<egraphs/egraph_euclidean_heuristic.h>
#include<limits>

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, double distance_inflation) : env_(env){
  dist_inflation = distance_inflation;
  inflation.clear();
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, const vector<double>& element_diff_inflation) : env_(env){
  inflation = element_diff_inflation;
}

void EGraphEuclideanHeuristic::setGoal(const vector<double>& goal){
  goal_ = goal;

  if(inflation.empty())
    inflation.resize(goal.size(), dist_inflation);

  ROS_INFO("goal is %f %f",goal[0],goal[1]);
  ROS_INFO("inflation is %f %f",inflation[0],inflation[1]);

  shortcut_cache_.resize(eg_->getNumComponents(),NULL);
  vector<double> comp_dists(eg_->getNumComponents(),std::numeric_limits<double>::max());
  vector<double> c_coord;
  vector<double> h_coord;
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    eg_->discToCont(eg_->id2vertex[i],c_coord);
    env_.projectToHeuristicSpace(c_coord,h_coord);
    double dist = 0;
    for(unsigned int j=0; j<h_coord.size(); j++){
      double d = (h_coord[j] - goal_[j])*inflation[j];
      dist += d*d;
    }
    int c = eg_->id2vertex[i]->component;
    if(dist < comp_dists[c]){
      comp_dists[c] = dist;
      shortcut_cache_[c] = eg_->id2vertex[i];
      ROS_INFO("update shortcut to %f %f (dist to goal %f)",h_coord[0],h_coord[1],dist);
    }
  }

  //search from the goal to all egraph states
  //we might be able to offload a floyd-warshall to the runPrecomputations, allowing a n^2 time setGoal
}

int EGraphEuclideanHeuristic::getHeuristic(const vector<double>& coord){
  double dist = 0;
  for(unsigned int i=0; i<coord.size(); i++){
    double d = (coord[i] - goal_[i])*inflation[i];
    dist += d*d;
  }
  dist = epsE_ * sqrt(dist);
  return int(dist);
}

void EGraphEuclideanHeuristic::getEGraphVerticesWithSameHeuristic(const vector<double>& coord, vector<EGraph::EGraphVertex*>& vertices){
  //there are no snaps for euclidean distance
}

void EGraphEuclideanHeuristic::runPrecomputations(){
}

void EGraphEuclideanHeuristic::getDirectShortcut(int component, vector<EGraph::EGraphVertex*>& shortcuts){
  shortcuts.clear();
  if(shortcut_cache_[component])
    shortcuts.push_back(shortcut_cache_[component]);
}

void EGraphEuclideanHeuristic::resetShortcuts(){
  shortcut_cache_.clear();
  shortcut_cache_.resize(eg_->getNumComponents(),NULL);
}

