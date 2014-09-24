#include<egraphs/egraph_euclidean_heuristic.h>
#include<limits>
#include<angles/angles.h>

#define HARDCODED_EPS_E 10.0
#define NN 5

#define KD_DEBUG true

using namespace std;

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

  indices = flann::Matrix<int>(new int[NN], 1, NN);
  dists = flann::Matrix<float>(new float[NN], 1, NN);
  query = flann::Matrix<float>(new float[cont.size()], 1, cont.size());
}

EGraphEuclideanHeuristic::~EGraphEuclideanHeuristic(){
  delete[] query.ptr();
  delete[] indices.ptr();
  delete[] dists.ptr();
}

void EGraphEuclideanHeuristic::setGoal(const vector<double>& goal){
  assert(epsE_==HARDCODED_EPS_E);
  ROS_INFO("set heuristic goal");
  goal_ = goal;

  coords_.resize(eg_->id2vertex.size()+1);
  coords_.back() = goal;

  if(inflation.empty())
    inflation.resize(goal.size(), dist_inflation);

  if(inflation.empty())
    inflation.resize(goal.size(), dist_inflation);
  if(continuous_joint.empty())
    continuous_joint.resize(goal.size(), false);

  ROS_INFO("compute shortcuts");
  //compute shortcuts
  shortcut_cache_.resize(eg_->getNumComponents(),NULL);
  vector<int> comp_dists(eg_->getNumComponents(),std::numeric_limits<int>::max());
  vector<double> c_coord;
  vector<double> h_coord;
  vector<int> goal_to_eg(eg_->id2vertex.size());
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    eg_->discToCont(eg_->id2vertex[i],c_coord);
    env_.projectToHeuristicSpace(c_coord,h_coord);
    int dist = euclideanDistance(h_coord,goal_);
    goal_to_eg[i] = dist;
    int c = eg_->id2vertex[i]->component;
    if(dist < comp_dists[c]){
      comp_dists[c] = dist;
      shortcut_cache_[c] = eg_->id2vertex[i];
      //ROS_INFO("update shortcut to %f %f (dist to goal %f)",h_coord[0],h_coord[1],dist);
    }
  }

  ROS_INFO("compute best gval for each egraph node");
  gval.clear();
  gval.resize(fw_matrix.size(),make_pair(INFINITECOST,0));
  for(unsigned int i=0; i<gval.size(); i++){
    gval[i].second = i;
    for(unsigned int j=0; j<fw_matrix.size(); j++){
      int d = goal_to_eg[j] + fw_matrix[i][j];
      //if(d >= INFINITECOST)
        //ROS_INFO("%d + %d",goal_to_eg[j],fw_matrix[i][j]);
      //assert(d < INFINITECOST);
      if(gval[i].first > d)
        gval[i].first = d;
    }
    //assert(gval[i].first < INFINITECOST);
  }
  ROS_INFO("sort gvals %lu",gval.size());
  gval_sorted = gval;
  sort(gval_sorted.begin(), gval_sorted.end()); //Note that this is expensive! 


#if KD_DEBUG
  initNaive();

  for(unsigned int i=0; i<gval.size(); i++){
    if(gval[i].first != verts[i].g){
      ROS_ERROR("fw=%d and dij=%d differ at %d",gval[i].first,verts[i].g,i);
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
  assert(coord.size()==12);
#endif
  //ROS_INFO("get heuristic");
  double epsNN = 1.0;
  bool sort = true;
  for(unsigned int i=0; i<coord.size(); i++)
    query[0][i] = coord[i];
  //ROS_INFO("get NN from KD-tree");
  index->knnSearch(query, indices, dists, NN, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED,/*epsNN-1*/ 0,sort));

#if KD_DEBUG
  //BEGIN ANNOYING ERROR CHECKING TO CONFIRM THEY ARE ACTUALLY THE NEAREST NEIGHBORS
  for(int i=0; i<NN; i++){
    //printf("idx=%d raw_dist=%f dist=%d\n", indices[0][i], dists[0][i], int(epsE_*sqrt(dists[0][i])));
    assert( int(epsE_*sqrt(dists[0][i])) == euclideanDistance(coord, coords_[indices[0][i]]));
  }

  //confirm the nearest neighbor
  {
    assert(eg_dist(query[0], dataMatrix[indices[0][0]], coord.size()) == dists[0][0]);
    for(unsigned int i=0; i<coords_.size()-1; i++){
      float d = eg_dist(query[0], dataMatrix[i], coord.size());
      if(d < dists[0][0]){
        ROS_ERROR("found a better nearest neighbor! %d with raw_dist %f",i,d);
        ROS_ERROR("%f %f %f %f %f %f %f %f %f %f %f %f",
            query[0][0],
            query[0][1],
            query[0][2],
            query[0][3],
            query[0][4],
            query[0][5],
            query[0][6],
            query[0][7],
            query[0][8],
            query[0][9],
            query[0][10],
            query[0][11]);
      }
      assert(dists[0][0] <= d);
    }
  }

  //confirm these are the top 5
  {
    int worst_kd_d = int(epsE_ * sqrt(dists[0][NN-1]));
    //printf("worst_kd_dist=%d\n",worst_kd_d);
    for(unsigned int i=0; i<coords_.size()-1; i++){
      int d = euclideanDistance(coord, coords_[i]);
      if(d < worst_kd_d){
        bool already_chosen = false;
        for(int j=0; j<NN; j++){
          if(indices[0][j] == int(i)){
            already_chosen = true;
            //printf("already chosen idx=%d dist=%d\n",i,d);
          }
        }
        if(!already_chosen)
          ROS_ERROR("idx=%d is better, dist=%d",i,d);
        assert(already_chosen);
      }
    }
  }
  //END ANNOYING ERROR CHECKING TO CONFIRM THEY ARE ACTUALLY THE NEAREST NEIGHBORS
#endif
  
  //suboptimality bound on choosing best index
  int eps = 3.0; //TODO: make parameter

  //initialize best_h and best_idx to the goal
  int g_low = 0.0; //lower bound on g comes from the goal
  int best_h = euclideanDistance(coord,goal_);
  int best_idx = eg_->id2vertex.size();

  //e_low comes from the NN from the kd-tree
  int e_low = 0.0;
  //before pouring over all the egraph nodes, we will first look at the KD-tree NN
  for(int i=0; i<NN; i++){
    int temp_e = epsE_ * sqrt(dists[0][i]);
    if(temp_e > e_low)
      e_low = temp_e;
    int temp_h = temp_e + gval[indices[0][i]].first;
    if(temp_h < best_h){
      best_h = temp_h;
      best_idx = indices[0][i];
    }
  }
  
  int last_update = 0;
  int g_max = best_h - e_low;
  //now iterate over all egraph nodes in increasing gval order
  unsigned int i = 0;
  for(; i<gval_sorted.size(); i++){
    if(best_h < eps*(e_low + g_low))
      break;
    int temp_g = gval_sorted[i].first;
    g_low = temp_g; //g is always increasing, bringing up the lower bound
    if(temp_g >= g_max)
      break;
    int temp_e = euclideanDistance(coord, coords_[gval_sorted[i].second]);
    int temp_h = temp_g + temp_e;
    if(temp_h < best_h){
      best_h = temp_h;
      best_idx = gval_sorted[i].second;
      g_max = best_h - e_low;
      last_update = i;
    }
  }

#if KD_DEBUG
  static int total_calls = 0;
  static int total_checks = 0;
  static int diff_between_best_and_break = 0;
  total_calls++;
  total_checks += i;
  diff_between_best_and_break += i - last_update;
  printf("break early %d/%lu had to be checked\n",i,gval_sorted.size());
  printf("best idx was %d of %d checked\n",last_update,i);
  printf("average difference between break and best %f\n",double(diff_between_best_and_break)/total_calls);
  printf("average checks %f/%lu = %f\n",double(total_checks)/total_calls, gval_sorted.size(), 
      double(total_checks)/total_calls/gval_sorted.size());

  //BEGIN COMPARE AGAINST NAIVE
  int naive_idx;
  int naive_dist = naiveGetHeuristic(coord, naive_idx);
  //ROS_INFO("naive: idx=%d dist=%d",naive_idx,naive_dist);
  //ROS_INFO("euclid_dist=%d gval=%d",euclideanDistance(coord,verts[naive_idx].coord), verts[naive_idx].g);
  for(unsigned int i=0; i<gval_sorted.size(); i++){
    if(gval_sorted[i].second == naive_idx){
      //ROS_INFO("naive was at gval_sorted[%d]",i);
      break;
    }
  }
  //ROS_INFO("kd: idx=%d dist=%d",best_idx,best_h);
  //ROS_INFO("euclid_dist=%d gval=%d",euclideanDistance(coord,coords_[best_idx]), gval[best_idx].first);
  //if(naive_idx != best_idx)
    //ROS_WARN("naive and kd chose different idx!");
  assert(best_h <= eps*naive_dist);
  assert(best_h >= naive_dist);
  static int times_same = 0;
  static double avg_bound = 0;
  avg_bound += double(best_h)/naive_dist;
  if(best_h == naive_dist)
    times_same++;
  printf("kd=naive %f percent\n", double(times_same)/total_calls);
  printf("average bound %f\n", avg_bound/total_calls);
  //END COMPARE AGAINST NAIVE
#endif

  last_best_idx = best_idx;
  return best_h;
}

inline int EGraphEuclideanHeuristic::euclideanDistance(const vector<double>& c1, const vector<double>& c2){
#if KD_DEBUG
  assert(c1.size()==12);
  assert(c2.size()==12);
#endif

  float accum = 0;
  for(unsigned int i=0; i<c1.size(); i++)
    accum += eg_dist.accum_dist(float(c1[i]), float(c2[i]), i);
  return int(epsE_ * sqrt(accum));

  /*
  float dist = 0;
  //printf("delta: ");
  if(print)
    ROS_INFO("ed start");
  for(unsigned int i=0; i<c1.size(); i++){
    float d;
    if(continuous_joint[i])
      d = angles::shortest_angular_distance(float(c2[i]),float(c1[i]))*inflation[i];
    else
      d = (float(c1[i]) - float(c2[i]))*inflation[i];
    if(print)
      ROS_INFO("%f ",d);
    //printf("%f ", d);
    dist += d*d;
    if(print)
      ROS_INFO("accum %f",dist);
  }
  //printf("\n");
  if(print)
    ROS_INFO("raw %f",dist);
  dist = epsE_ * sqrt(dist);
  if(print)
    ROS_INFO("all but cst %f",dist);
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
  assert(coord.size()==12);
  vector<double> coord2 = coords_[last_best_idx];
  for(unsigned int i=0; i<coord.size(); i++){
    double d;
    if(continuous_joint[i])
      d = fabs(angles::shortest_angular_distance(coord[i],coord2[i]));
    else
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
  print = false;
  if(!coords_.empty())
    return;

  //compute the heuristic coordinate for each egraph vertex
  vector<double> c_coord;
  coords_.resize(eg_->id2vertex.size());
  for(unsigned int i=0; i<coords_.size(); i++){
    eg_->discToCont(eg_->id2vertex[i],c_coord);
    env_.projectToHeuristicSpace(c_coord,coords_[i]);
  }
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

  /*
  for(unsigned int i=0; i<fw_matrix.size(); i++)
    for(unsigned int j=0; j<fw_matrix.size(); j++)
      assert(fw_matrix[i][j] >= 0);
      */

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


  dataMatrix = flann::Matrix<float>(new float[(coords_[0].size())*coords_.size()], coords_.size(), coords_[0].size());
  for(unsigned int i=0; i<coords_.size(); i++)
    for(unsigned int j=0; j<coords_[i].size(); j++)
      dataMatrix[i][j] = coords_[i][j];
  ROS_INFO("made FLANN matrix (%lu by %lu)",dataMatrix.rows,dataMatrix.cols);

  ROS_INFO("build KD-tree");
  int leafSize = 10;
  //index = new flann::Index<EG_DIST<float> >(dataMatrix, flann::KDTreeSingleIndexParams(leafSize));
  index = new flann::Index<EG_DIST<float> >(dataMatrix, flann::LinearIndexParams());
  index->buildIndex();

  /*
  {
    flann::Index<EG_DIST<float> >* index2;
    index2 = new flann::Index<EG_DIST<float> >(dataMatrix, flann::LinearIndexParams());
    index2->buildIndex();

    int c_size = 12;
    flann::Matrix<int> indices(new int[NN], 1, NN);
    flann::Matrix<float> dists(new float[NN], 1, NN);
    flann::Matrix<float> query(new float[c_size], 1, c_size);
    query[0][0] = 0.520000;
    query[0][1] = -0.160000;
    query[0][2] = -0.340000;
    query[0][3] = 0.000000;
    query[0][4] = 0.098175;
    query[0][5] = 0.589049;
    query[0][6] = 5.780490;
    query[0][7] = 1.396356;
    query[0][8] = 5.860000;
    query[0][9] = 1.960000;
    query[0][10] = 0.180000;
    query[0][11] = 5.497787;
    index->knnSearch(query, indices, dists, NN, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED, 0, true));

    flann::Matrix<int> indices2(new int[NN], 1, NN);
    flann::Matrix<float> dists2(new float[NN], 1, NN);
    index2->knnSearch(query, indices2, dists2, NN, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED, 0, true));

    printf("kd tree\n");
    for(int i=0; i<NN; i++)
      printf("idx=%d raw_dist=%f dist=%d\n", indices[0][i], dists[0][i], int(epsE_*sqrt(dists[0][i])));

    printf("linear search\n");
    for(int i=0; i<NN; i++)
      printf("idx=%d raw_dist=%f dist=%d\n", indices2[0][i], dists2[0][i], int(epsE_*sqrt(dists2[0][i])));

    assert(dists[0][0] == dists2[0][0]);

    //confirm the nearest neighbor
    for(unsigned int i=0; i<coords_.size()-1; i++){
      float d = eg_dist(query[0], dataMatrix[i], c_size);
      if(d < dists[0][0])
        ROS_ERROR("found a better nearest neighbor! %d with raw_dist %f",i,d);
      assert(dists[0][0] <= d);
    }
  }
  */

#if KD_DEBUG
  for(unsigned int i=0; i<coords_.size(); i++){
    for(unsigned int j=0; j<coords_.size(); j++){
      int kd_dist = int(epsE_ * sqrt(eg_dist(dataMatrix[i], dataMatrix[j], coords_[0].size())));
      float accum = 0;
      for(unsigned int k=0; k<coords_[0].size(); k++)
        accum += eg_dist.accum_dist(dataMatrix[i][k], dataMatrix[j][k], k);
      int accum_dist = int(epsE_ * sqrt(accum));
      assert(accum_dist == kd_dist);
    }
  }

  //print = true;
  for(unsigned int i=0; i<coords_.size(); i++){
    for(unsigned int j=0; j<coords_.size(); j++){
      int kd_dist = int(epsE_ * sqrt(eg_dist(dataMatrix[i], dataMatrix[j], coords_[0].size())));
      int naive_dist = euclideanDistance(coords_[i],coords_[j]);
      if(kd_dist != naive_dist)
        ROS_ERROR("(%d %d) kd_dist=%d naive_dist=%d (kd raw %f) (sqrt(kd)=%f) kd_all_but_cast=%f",i,j,kd_dist,naive_dist, 
            eg_dist(dataMatrix[i], dataMatrix[j], coords_[0].size()),
            sqrt(eg_dist(dataMatrix[i], dataMatrix[j], coords_[0].size())),
            epsE_ * sqrt(eg_dist(dataMatrix[i], dataMatrix[j], coords_[0].size())));
      assert(fabs(kd_dist - naive_dist) <= 1);
    }
  }
#endif
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

