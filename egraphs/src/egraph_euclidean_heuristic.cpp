#include<egraphs/egraph_euclidean_heuristic.h>
#include<limits>
#include<angles/angles.h>

using namespace std;

/* WARNING: This heuristic is not very efficient right now. It will be replaced in the next few months with a much more 
 * efficient version. Use at your own risk!
 */

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env) : env_(env){
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, double distance_inflation) : env_(env){
  dist_inflation = distance_inflation;
  inflation.clear();
  continuous_joint.clear();
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, 
    double distance_inflation, const vector<bool>& cont) : env_(env){
  inflation.resize(cont.size(), distance_inflation);
  continuous_joint = cont;
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, const vector<double>& element_diff_inflation) : env_(env){
  inflation = element_diff_inflation;
  continuous_joint.clear();
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, 
    const vector<double>& element_diff_inflation, const vector<bool>& cont) : env_(env){
  inflation = element_diff_inflation;
  continuous_joint = cont;
}

void EGraphEuclideanHeuristic::setGoal(const vector<double>& goal){
  goal_ = goal;

  if(inflation.empty())
    inflation.resize(goal.size(), dist_inflation);
  if(continuous_joint.empty())
    continuous_joint.resize(goal.size(), false);

  //compute shortcuts
  shortcut_cache_.resize(eg_->getNumComponents(),NULL);
  vector<int> comp_dists(eg_->getNumComponents(),std::numeric_limits<int>::max());
  vector<double> c_coord;
  vector<double> h_coord;
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    eg_->discToCont(eg_->id2vertex[i],c_coord);
    env_.projectToHeuristicSpace(c_coord,h_coord);
    int dist = euclideanDistance(h_coord,goal_);
    int c = eg_->id2vertex[i]->component;
    if(dist < comp_dists[c]){
      comp_dists[c] = dist;
      shortcut_cache_[c] = eg_->id2vertex[i];
      //ROS_INFO("update shortcut to %f %f (dist to goal %f)",h_coord[0],h_coord[1],dist);
    }
  }

  heap.makeemptyheap();
  verts.resize(eg_->id2vertex.size()+1);
  CKey key;

  //put goal in heap
  verts.back().coord = goal;
  verts.back().id = verts.size()-1;
  verts.back().heapindex = 0;
  verts.back().g = 0;
  key.key[0] = 0;
  heap.insertheap(&verts.back(),key);

  //initialize the g-value of each egraph vertex to INF
  for(unsigned int i=0; i<verts.size()-1; i++){
    eg_->discToCont(eg_->id2vertex[i],c_coord);
    env_.projectToHeuristicSpace(c_coord,h_coord);
    verts[i].coord = h_coord;

    verts[i].id = i;
    verts[i].heapindex = 0;
    key.key[0] = INFINITECOST;
    verts[i].g = INFINITECOST;
    heap.insertheap(&verts[i],key);
  }

  //run a Dijkstra to compute the distance to each egraph state from the goal
  while(!heap.emptyheap()){
    EGraphEuclideanState* state = (EGraphEuclideanState*)heap.deleteminheap();

    //relax using euclidean heuristic edges
    for(unsigned int i=0; i<verts.size()-1; i++){
      //skip if this state is already closed
      if(state->g >= verts[i].g)
        continue;
      int h_dist = euclideanDistance(state->coord,verts[i].coord);
      int new_g = state->g + h_dist;
      if(new_g < verts[i].g){
        verts[i].g = new_g;
        key.key[0] = new_g;
        heap.updateheap(&verts[i],key);
      }
    }

    //relax using egraph edges
    if(state->id < int(eg_->id2vertex.size())){
      EGraph::EGraphVertex* v = eg_->id2vertex[state->id];
      for(unsigned int i=0; i<v->neighbors.size(); i++){
        int neighbor_id = v->neighbors[i]->id;
        int new_g = state->g + v->costs[i];
        if(new_g < verts[neighbor_id].g){
          verts[neighbor_id].g = new_g;
          key.key[0] = new_g;
          heap.updateheap(&verts[neighbor_id],key);
        }
      }
      //TODO: check for validity of edges
    }
  }

  //TODO:we might be able to offload a floyd-warshall to the runPrecomputations, allowing a n^2 time setGoal
}

int EGraphEuclideanHeuristic::getHeuristic(const vector<double>& coord){
  int best_idx = -1;
  int best_dist = INFINITECOST;
  for(unsigned int i=0; i<verts.size(); i++){
    int dist = euclideanDistance(coord,verts[i].coord) + verts[i].g;
    if(dist < best_dist){
      best_dist = dist;
      best_idx = i;
    }
  }
  /*
  printf("coord: ");
  for(unsigned int i=0; i<coord.size(); i++)
    printf("%f ",coord[i]);
  printf("\n");
  printf("goal:  ");
  for(unsigned int i=0; i<verts[best_idx].coord.size(); i++)
    printf("%f ",verts[best_idx].coord[i]);
  printf("\n");
  printf("dist = %d\n",best_dist);
  */
  //std::cin.get();

  //ROS_INFO("best_idx = %d",best_idx);
  //if(best_idx == verts.size()-1)
    //ROS_WARN("being guided to goal...");
  last_best_idx = best_idx;
  return best_dist;
}

inline int EGraphEuclideanHeuristic::euclideanDistance(const vector<double>& c1, const vector<double>& c2){
  assert(c1.size()==9);
  assert(c2.size()==9);

  float accum = 0;
  for(unsigned int i=0; i<c1.size(); i++)
    accum += eg_dist.accum_dist(float(c1[i]), float(c2[i]), i);
  return int(epsE_ * sqrt(accum));

  /*
  double dist = 0;
  //printf("delta: ");
  for(unsigned int i=0; i<c1.size(); i++){
    double d;
    if(continuous_joint[i])
      d = angles::shortest_angular_distance(c1[i],c2[i])*inflation[i];
    else
      d = (c1[i] - c2[i])*inflation[i];
    //printf("%f ", d);
    dist += d*d;
  }
  //printf("\n");
  dist = epsE_ * sqrt(dist);
  return int(dist);
  */
}

void EGraphEuclideanHeuristic::getEGraphVerticesWithSameHeuristic(const vector<double>& coord, vector<EGraph::EGraphVertex*>& vertices){
  if(dist_to_snap.empty())
    return;
  //there are no snaps for euclidean distance...not! AMP IT UP!
  getHeuristic(coord);
  if(last_best_idx >= int(eg_->id2vertex.size())){
    //ROS_INFO("want to snap to goal?");
    return;
  }
  assert(coord.size()==9);
  vector<double> coord2 = verts[last_best_idx].coord;
  for(unsigned int i=0; i<coord.size(); i++){
    double d;
    //if(continuous_joint[i])
      //d = fabs(angles::shortest_angular_distance(coord[i],coord2[i]));
    //else
      d = fabs(coord[i] - coord2[i]);
    if(d > dist_to_snap[i]){
      ROS_ERROR("not close enough to snap: dimension %d (%f - %f)",i,coord[i],coord2[i]);
      return;
    }
  }

  vertices.push_back(eg_->id2vertex[last_best_idx]);
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

