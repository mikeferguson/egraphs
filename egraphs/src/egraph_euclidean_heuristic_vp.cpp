#include<egraphs/egraph_euclidean_heuristic.h>
#include<limits>
#include<angles/angles.h>

#define C_SIZE 9
#define KD_DEBUG false

#define USE_VP false

using namespace std;

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env) : env_(env){
  epsE_ = HARDCODED_EPS_E;
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, double distance_inflation) : env_(env){
  dist_inflation = distance_inflation;
  inflation.clear();
  continuous_joint.clear();
  epsE_ = HARDCODED_EPS_E;
  exit(1);
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, 
    double distance_inflation, const vector<bool>& cont) : env_(env){
  inflation.resize(cont.size(), distance_inflation);
  continuous_joint = cont;
  epsE_ = HARDCODED_EPS_E;
  exit(1);
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, const vector<double>& element_diff_inflation) : env_(env){
  inflation = element_diff_inflation;
  continuous_joint.clear();
  epsE_ = HARDCODED_EPS_E;
  exit(1);
}

EGraphEuclideanHeuristic::EGraphEuclideanHeuristic(const EGraphable<vector<double> >& env, 
    const vector<double>& element_diff_inflation, const vector<bool>& cont) : env_(env){
  inflation = element_diff_inflation;
  continuous_joint = cont;
  epsE_ = HARDCODED_EPS_E;

  exit(1);
}

EGraphEuclideanHeuristic::~EGraphEuclideanHeuristic(){
}

void EGraphEuclideanHeuristic::setGoal(const vector<double>& goal){
  assert(epsE_==HARDCODED_EPS_E);
  //ROS_INFO("set heuristic goal");
  goal_ = goal;

  coords_.resize(eg_->id2vertex.size()+1);
  coords_.back() = goal;

  /*
  if(inflation.empty())
    inflation.resize(goal.size(), dist_inflation);

  if(inflation.empty())
    inflation.resize(goal.size(), dist_inflation);
  if(continuous_joint.empty())
    continuous_joint.resize(goal.size(), false);
    */

  //ROS_INFO("compute shortcuts");
  //compute shortcuts
  shortcut_cache_.resize(eg_->getNumComponents(),NULL);
  vector<int> comp_dists(eg_->getNumComponents(),std::numeric_limits<int>::max());
  vector<double> c_coord;
  vector<double> h_coord;
  vector<int> goal_to_eg(eg_->id2vertex.size());
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    //eg_->discToCont(eg_->id2vertex[i],c_coord);
    //env_.projectToHeuristicSpace(c_coord,h_coord);
    //int dist = euclideanDistance(h_coord,goal_);
    int dist = euclideanDistance(coords_[i],goal_);
    goal_to_eg[i] = dist;
    int c = eg_->id2vertex[i]->component;
    if(dist < comp_dists[c]){
      comp_dists[c] = dist;
      shortcut_cache_[c] = eg_->id2vertex[i];
      //ROS_INFO("update shortcut to %f %f (dist to goal %f)",h_coord[0],h_coord[1],dist);
    }
  }

  //ROS_INFO("compute best gval for each egraph node");
  for(unsigned int i=0; i<fw_matrix.size(); i++){
    database_[i][C_SIZE] = INFINITECOST;
    for(unsigned int j=0; j<fw_matrix.size(); j++){
      int d = goal_to_eg[j] + fw_matrix[i][j];
      if(database_[i][C_SIZE] > d)
        database_[i][C_SIZE] = d;
    }
  }
  
  assert(goal.size()==C_SIZE);
  for(unsigned int i=0; i<goal.size(); i++)
    database_.back()[i] = goal[i];
  database_.back()[C_SIZE] = 0;
  database_.back()[C_SIZE+1] = float(database_.size()-1);

  //vector<vector<float> > temp_database = database_;

  int leafSize = 10;
  if(USE_VP)
    vptree.create(database_,leafSize);
  else
    ghtree.create(database_,leafSize);


#if KD_DEBUG
  for(unsigned int i=0; i<database_.size(); i++)
    assert(database_[i][C_SIZE+1] == i);

  
  initNaive();

  for(unsigned int i=0; i<database_.size(); i++){
    if(int(database_[i][C_SIZE]) != verts[i].g){
      ROS_ERROR("fw=%d and dij=%d differ at %d",int(database_[i][C_SIZE]),verts[i].g,i);
    }
  }
#endif
  //std::cin.get();
}


void EGraphEuclideanHeuristic::initNaive(){
  //TEMPORARY CODE FOR CHECKING FOR CORRECTNESS
  heap.makeemptyheap();
  verts.resize(eg_->id2vertex.size()+1);
  CKey key;

  //put goal in heap
  verts.back().coord = goal_;
  verts.back().id = verts.size()-1;
  verts.back().heapindex = 0;
  verts.back().g = 0;
  key.key[0] = 0;
  heap.insertheap(&verts.back(),key);

  //initialize the g-value of each egraph vertex to INF
  vector<double> c_coord;
  vector<double> h_coord;
  for(unsigned int i=0; i<verts.size()-1; i++){
    //eg_->discToCont(eg_->id2vertex[i],c_coord);
    //env_.projectToHeuristicSpace(c_coord,h_coord);
    //verts[i].coord = h_coord;
    verts[i].coord = coords_[i];

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
}

int EGraphEuclideanHeuristic::naiveGetHeuristic(const vector<double>& coord, int& best_idx){
  best_idx = -1;
  int best_dist = INFINITECOST;
  for(unsigned int i=0; i<verts.size(); i++){
    int dist = euclideanDistance(coord,verts[i].coord) + verts[i].g;
    if(dist < best_dist){
      best_dist = dist;
      best_idx = i;
    }
  }
  return best_dist;
}

int EGraphEuclideanHeuristic::getHeuristic(const vector<double>& coord){
#if KD_DEBUG
  assert(coord.size()==C_SIZE);
#endif

  double eps = 3.0;

  vector<float> query(C_SIZE+2,0);
  for(unsigned int i=0; i<coord.size(); i++)
    query[i] = float(coord[i]);

  std::vector<float> neighbor(C_SIZE+2,0);
  int dist;
  if(USE_VP)
    vptree.search(query, neighbor, dist, eps);
  else
    ghtree.search(query, neighbor, dist, eps);
  int best_h = int(round(dist));
  int best_idx = int(round(neighbor[C_SIZE+1]));

  /*
  assert( neighbor[C_SIZE] == database_[best_idx][C_SIZE] );

  int eh = FullEGDist(neighbor, query);
  printf("vp=%d (%d, %d)        naive=%d  \n",eh, int(eh-neighbor[C_SIZE]), int(neighbor[C_SIZE]), euclideanDistance(coord,coords_[best_idx]));

  assert(fabs(eh-neighbor[C_SIZE] - euclideanDistance(coord,coords_[best_idx])) <= 1);
  assert(fabs(euclideanDistance(coord,coords_[best_idx]) + database_[best_idx][C_SIZE] - best_h) <= 1);
  */



#if KD_DEBUG
  //BEGIN COMPARE AGAINST NAIVE
  int naive_idx;
  int naive_dist = naiveGetHeuristic(coord, naive_idx);
  ROS_INFO("naive: idx=%d dist=%d",naive_idx,naive_dist);
  ROS_INFO("euclid_dist=%d gval=%d",euclideanDistance(coord,verts[naive_idx].coord), verts[naive_idx].g);
  ROS_INFO("vp: idx=%d dist=%d",best_idx,best_h);
  ROS_INFO("euclid_dist=%d gval=%d",euclideanDistance(coord,coords_[best_idx]), int(round(database_[best_idx][C_SIZE])));
  if(best_h == naive_dist-1)
    best_h = naive_dist;
  if(naive_idx != best_idx)
    ROS_WARN("naive and vp chose different idx!");
  assert(best_h <= eps*naive_dist);
  assert(best_h >= naive_dist);
  static int total_calls = 0;
  total_calls++;
  static int times_same = 0;
  static double avg_bound = 0;
  avg_bound += double(best_h)/naive_dist;
  if(best_h == naive_dist)
    times_same++;
  printf("vp=naive %f percent\n", double(times_same)/total_calls);
  printf("average bound %f\n", avg_bound/total_calls);
  //END COMPARE AGAINST NAIVE
#endif

  last_best_idx = best_idx;
  return best_h;
}

inline int EGraphEuclideanHeuristic::euclideanDistance(const vector<double>& c1, const vector<double>& c2){
#if KD_DEBUG
  assert(c1.size()==C_SIZE);
  assert(c2.size()==C_SIZE);
#endif

  float accum = 0;
  for(unsigned int i=0; i<c1.size(); i++)
    accum += eg_dist.accum_dist(float(c1[i]), float(c2[i]), i);
  return int(epsE_ * sqrt(accum));
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
#if KD_DEBUG
  assert(coord.size()==C_SIZE);
#endif
  vector<double> coord2 = coords_[last_best_idx];
  for(unsigned int i=0; i<coord.size(); i++){
    double d;
    //if(continuous_joint[i])
      //d = fabs(angles::shortest_angular_distance(coord[i],coord2[i]));
    //else
      d = fabs(coord[i] - coord2[i]);
    if(d > dist_to_snap[i]){
#if KD_DEBUG
      ROS_ERROR("not close enough to snap: dimension %d (%f - %f)",i,coord[i],coord2[i]);
#endif
      return;
    }
  }

  vertices.push_back(eg_->id2vertex[last_best_idx]);
}

void EGraphEuclideanHeuristic::runPrecomputations(){
  assert(epsE_ == HARDCODED_EPS_E);
  print = false;
  if(!coords_.empty())
    return;

  //compute the heuristic coordinate for each egraph vertex
  vector<double> c_coord;
  coords_.resize(eg_->id2vertex.size());
  database_.resize(coords_.size()+1);
  for(unsigned int i=0; i<coords_.size(); i++){
    eg_->discToCont(eg_->id2vertex[i],c_coord);
    env_.projectToHeuristicSpace(c_coord,coords_[i]);
    database_[i].resize(coords_[i].size()+2,0);
    for(unsigned int j=0; j<coords_[i].size(); j++)
      database_[i][j] = float(coords_[i][j]);
    database_[i][C_SIZE+1] = float(i);
  }
  database_.back().resize(C_SIZE+2,0);
  ROS_INFO("computed egraph heuristic coords (%lu)",coords_.size());

  //allocate memory for the matrix
  fw_matrix.resize(eg_->id2vertex.size());
  for(unsigned int i=0; i<fw_matrix.size(); i++)
    fw_matrix[i].resize(fw_matrix.size(), -1);
  ROS_INFO("allocated floyd-warshall matrix (%lu by %lu)",fw_matrix.size(),fw_matrix[0].size());

  ROS_INFO("init matrix to fully connected euclidean edges");
  //initialize fully connected graph using penalized euclidean distance heuristic
  for(unsigned int i=0; i<fw_matrix.size(); i++){
    for(unsigned int j=i; j<fw_matrix.size(); j++){
      fw_matrix[i][j] = euclideanDistance(coords_[i],coords_[j]);
      fw_matrix[j][i] = fw_matrix[i][j];
    }
  }

  ROS_INFO("add egraph edges");
  //replace any original heuristic edges with e-graph edges that are cheaper
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    for(unsigned int j=0; j<eg_->id2vertex[i]->neighbors.size(); j++){
      int id = eg_->id2vertex[i]->neighbors[j]->id;
      //assert(id >= 0 && id < int(eg_->id2vertex.size()));
      int c = eg_->id2vertex[i]->costs[j];
      if(fw_matrix[i][id] > c){
        fw_matrix[i][id] = c;
        fw_matrix[id][i] = c;
      }
    }
  }
  //TODO: add check for edge valid!!

  ROS_INFO("run floyd-warshall");
  //run floyd-warshall
  for(unsigned int k=0; k<fw_matrix.size(); k++)
    for(unsigned int i=0; i<fw_matrix.size(); i++)
      for(unsigned int j=0; j<fw_matrix.size(); j++)
        if(fw_matrix[i][j] > fw_matrix[i][k] + fw_matrix[k][j])
          fw_matrix[i][j] = fw_matrix[i][k] + fw_matrix[k][j];

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

