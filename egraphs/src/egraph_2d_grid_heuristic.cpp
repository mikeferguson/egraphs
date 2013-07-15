#include<egraphs/egraph_2d_grid_heuristic.h>

#define HEUR_XY2ID(x,y) ((y + 1) * width_ + (x + 1))

EGraph2dGridHeuristic::EGraph2dGridHeuristic(EGraphDownProject* downProject, int size_x, int size_y, int move_cost){
  downProject_ = downProject;
  sizex_ = size_x;
  sizey_ = size_y;
  cost_1_move_ = move_cost;

  width_ = sizex_ + 2;
  height_ = sizey_ + 2;
  planeSize_ = width_ * height_;
  printf("sizes: x=%d y=%d plane=%d\n",sizex_,sizey_,planeSize_);

  heur.resize(planeSize_);
  sc.resize(planeSize_);
  for (int i = 0; i < planeSize_; i++) {
    int x = i % width_, y = i / width_;
    if (x == 0 || x == width_ - 1 || y == 0 || y == height_ - 1){
      heur[i].cost = -1;
      sc[i].cost = -1;
    }
    else{
      heur[i].cost = INFINITECOST;
      sc[i].cost = INFINITECOST;
    }
  }

}

void EGraph2dGridHeuristic::setGrid(vector<vector<bool> >& grid){
  if(grid.size() != (unsigned int)(sizex_)){
    ROS_ERROR("[EGraph2dGridHeuristic] The dimensions provided in the constructor don't match the given grid.");
    return;
  }
  if(grid.front().size() != (unsigned int)(sizey_) || grid.back().size() != (unsigned int)(sizey_)){
    ROS_ERROR("[EGraph2dGridHeuristic] The dimensions provided in the constructor don't match the given grid.");
    return;
  }

  for(unsigned int x=0; x<grid.size(); x++){
    for(unsigned int y=0; y<grid[x].size(); y++){
      int id = HEUR_XY2ID(x,y);
      if(grid[x][y]){
        heur[id].cost = -1;
        sc[id].cost = -1;
      }
      else{
        heur[id].cost = INFINITECOST;
        sc[id].cost = INFINITECOST;
      }
    }
  }

/*
  FILE* fout = fopen("heur2d_grid.csv","w");
  for(unsigned int x=0; x<grid.size(); x++){
    for(unsigned int y=0; y<grid[x].size(); y++){
      fprintf(fout,"%d ",int(grid[x][y]));
    }
    fprintf(fout,"\n");
  }
  fclose(fout);
*/
}

void EGraph2dGridHeuristic::getEGraphVerticesWithSameHeuristic(vector<double> coord, vector<EGraph::EGraphVertex*>& vertices){
  vector<int> dp;
  downProject_->downProject(coord,dp);
  vertices.clear();
  vertices = heur[HEUR_XY2ID(dp[0],dp[1])].egraph_vertices;
}

void EGraph2dGridHeuristic::runPrecomputations(){
  //ROS_INFO("begin precomputations");
  //refill the cell to egraph vertex mapping
  for(int i=0; i<planeSize_; i++){
    heur[i].egraph_vertices.clear();
    sc[i].egraph_vertices.clear();
  }

  vector<int> dp;
  vector<double> c_coord;
  //ROS_INFO("down project edges...");
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    eg_->discToCont(eg_->id2vertex[i],c_coord);
    //ROS_INFO("size of coord %d",c_coord.size());
    downProject_->downProject(c_coord,dp);
    //ROS_INFO("size of coord %d",dp.size());
    //ROS_INFO("coord %d %d",dp[0],dp[1]);
    heur[HEUR_XY2ID(dp[0],dp[1])].egraph_vertices.push_back(eg_->id2vertex[i]);
    sc[HEUR_XY2ID(dp[0],dp[1])].egraph_vertices.push_back(eg_->id2vertex[i]);
  }
  //ROS_INFO("done precomputations");
}

void EGraph2dGridHeuristic::setGoal(vector<double> goal){
  //ROS_ERROR("begin setGoal");
  //clear the heur data structure
  for(int i=0; i<planeSize_; i++){
    heur[i].id = i;
    heur[i].heapindex = 0;
    heur[i].closed = false;
    if(heur[i].cost!=-1)
      heur[i].cost = INFINITECOST;
    sc[i].id = i;
    sc[i].heapindex = 0;
    sc[i].closed = false;
    if(sc[i].cost!=-1)
      sc[i].cost = INFINITECOST;
  }
  
  vector<int> dp;
  if(goal.empty()){
    dp = goal_dp_;
  }
  else{
    //down project the goal state and add it to the queue
    //downProject_->downProject(goal,dp);

    //assume we are given the goal already down projected
    for(unsigned int i=0; i<goal.size(); i++)
      dp.push_back(int(goal[i]));
    goal_dp_ = dp;
  }
  CKey key;
  key.key[0] = 0;
  heap.makeemptyheap();
  int id = HEUR_XY2ID(dp[0],dp[1]);
  heap.insertheap(&heur[id],key);
  heur[id].cost = 0;

  sc_heap.makeemptyheap();
  id = HEUR_XY2ID(dp[0],dp[1]);
  sc_heap.insertheap(&sc[id],key);
  sc[id].cost = 0;

  inflated_cost_1_move_ = cost_1_move_ * epsE_;
}

#define HEUR_SUCCESSOR(offset){                                                   \
  if(heur[id + (offset)].cost != -1 && (heur[id + (offset)].cost > currentCost)){ \
    if(heur[id + (offset)].heapindex != 0)                                        \
      heap.updateheap(&heur[id + (offset)],key);                                  \
    else                                                                          \
      heap.insertheap(&heur[id + (offset)],key);                                  \
    heur[id + (offset)].cost = currentCost;                                       \
  }                                                                               \
}

int EGraph2dGridHeuristic::getHeuristic(vector<double> coord){
  vector<int> dp;
  downProject_->downProject(coord,dp);
  EGraph2dGridHeuristicCell* cell = &heur[HEUR_XY2ID(dp[0],dp[1])];

  if(cell->cost==-1)
    return INFINITECOST;
  
  CKey key;
  //compute distance from H to all cells and note for each cell, what node in H was the closest
  while(!heap.emptyheap() && !cell->closed){
    EGraph2dGridHeuristicCell* state = (EGraph2dGridHeuristicCell*)heap.deleteminheap();
    int id = state->id;
    state->closed = true;
    int oldCost = state->cost;
    int currentCost = oldCost + inflated_cost_1_move_;
    key.key[0] = currentCost;

    HEUR_SUCCESSOR(-width_);                  //-y
    HEUR_SUCCESSOR(1);                        //+x
    HEUR_SUCCESSOR(width_);                   //+y
    HEUR_SUCCESSOR(-1);                       //-x
    HEUR_SUCCESSOR(-width_-1);                //-y-x
    HEUR_SUCCESSOR(-width_+1);                //-y+x
    HEUR_SUCCESSOR(width_+1);                 //+y+x
    HEUR_SUCCESSOR(width_-1);                 //+y-x

    vector<double> c_coord;
    for(unsigned int i=0; i<state->egraph_vertices.size(); i++){
      for(unsigned int j=0; j<state->egraph_vertices[i]->neighbors.size(); j++){
        if(!state->egraph_vertices[i]->valid[j])
          continue;
        eg_->discToCont(state->egraph_vertices[i]->neighbors[j],c_coord);
        downProject_->downProject(c_coord,dp);
        EGraph2dGridHeuristicCell* cell = &heur[HEUR_XY2ID(dp[0],dp[1])];
        int newCost = oldCost + state->egraph_vertices[i]->costs[j];
        if(cell->cost > newCost){ //if we found a cheaper path to it
          key.key[0] = newCost;
          if(cell->heapindex != 0)
            heap.updateheap(cell,key);
          else
            heap.insertheap(cell,key);
          cell->cost = newCost;
        }
      }
    }
  }
  return cell->cost;
}

// IF IT IS NOT AN OBSTACLE, PUT IN THE HEAP
#define SHORTCUT_SUCCESSOR(offset){               \
  if(sc[id + (offset)].cost != -1 && (sc[id + (offset)].cost > currentCost)){\
    if(sc[id + (offset)].heapindex != 0)          \
      sc_heap.updateheap(&sc[id + (offset)],key); \
    else                                          \
      sc_heap.insertheap(&sc[id + (offset)],key); \
    sc[id + (offset)].cost = currentCost;         \
  }                                               \
}

// 2d breadth first search from goal to all other states until desired component
// 
void EGraph2dGridHeuristic::getDirectShortcut(int component, vector<EGraph::EGraphVertex*>& shortcuts){
  //we can assume that we would not be called if we have already discovered that component
  
  CKey key;
  //compute distance from H to all cells and note for each cell, what node in H was the closest
  while(!sc_heap.emptyheap() && shortcuts.empty()){
    EGraph2dGridHeuristicCell* state = (EGraph2dGridHeuristicCell*)sc_heap.deleteminheap();
    int id = state->id;
    state->closed = true;
    int oldCost = state->cost;
    int currentCost = oldCost + inflated_cost_1_move_;
    key.key[0] = currentCost;

    SHORTCUT_SUCCESSOR(-width_);                  //-y
    SHORTCUT_SUCCESSOR(1);                        //+x
    SHORTCUT_SUCCESSOR(width_);                   //+y
    SHORTCUT_SUCCESSOR(-1);                       //-x
    SHORTCUT_SUCCESSOR(-width_-1);                //-y-x
    SHORTCUT_SUCCESSOR(-width_+1);                //-y+x
    SHORTCUT_SUCCESSOR(width_+2);                 //+y+x
    SHORTCUT_SUCCESSOR(width_-1);                 //+y-x

    for(unsigned int i=0; i<state->egraph_vertices.size(); i++){
      if(state->egraph_vertices[i]->component==component){
        shortcuts.push_back(state->egraph_vertices[i]);
        break;
      }
    }
  }
}



