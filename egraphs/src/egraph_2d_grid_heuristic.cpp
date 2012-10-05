#include<egraphs/egraph_2d_grid_heuristic.h>

#define HEUR_XY2ID(x,y) ((y + 1) * width_ + (x + 1))

EGraph2dGridHeuristic::EGraph2dGridHeuristic(int size_x, int size_y, int move_cost){
  sizex_ = size_x;
  sizey_ = size_y;
  cost_1_move_ = move_cost;

  width_ = sizex_ + 2;
  height_ = sizey_ + 2;
  planeSize_ = width_ * height_;
  printf("sizes: x=%d y=%d plane=%d\n",sizex_,sizey_,planeSize_);

  heur.resize(planeSize_);
  for (int i = 0; i < planeSize_; i++) {
    int x = i % width_, y = i / width_;
    if (x == 0 || x == width_ - 1 || y == 0 || y == height_ - 1)
      heur[i].cost = -1;
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
      if(grid[x][y])
        heur[id].cost = -1;
      else
        heur[id].cost = INFINITECOST;
    }
  }
}

void EGraph2dGridHeuristic::cellToStates(vector<int> dp, vector<EGraph::EGraphVertex*>& states){
  states.clear();
  states = heur[HEUR_XY2ID(dp[0],dp[1])].egraph_vertices;
}

void EGraph2dGridHeuristic::setGoal(vector<double> goal){
  //clear the heur data structure
  for(int i=0; i<planeSize_; i++){
    heur[i].id = i;
    heur[i].heapindex = 0;
    heur[i].closed = false;
    heur[i].egraph_vertices.clear();
  }
  
  //refill the cell to egraph vertex mapping
  vector<int> dp;
  vector<double> c_coord;
  for(unsigned int i=0; eg_->id2vertex.size(); i++){
    eg_->discToCont(eg_->id2vertex[i]->coord,c_coord);
    egraphable_->downProject(c_coord,dp);
    heur[HEUR_XY2ID(dp[0],dp[1])].egraph_vertices.push_back(eg_->id2vertex[i]);
  }

  if(goal.empty()){
    dp = goal_dp_;
  }
  else{
    //down project the goal state and add it to the queue
    egraphable_->downProject(goal,dp);
    goal_dp_ = dp;
  }
  CKey key;
  key.key[0] = 0;
  heap.makeemptyheap();
  int id = HEUR_XY2ID(dp[0],dp[1]);
  heap.insertheap(&heur[id],key);
  heur[id].cost = 0;
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
  egraphable_->downProject(coord,dp);
  EGraph2dGridHeuristicCell* cell = &heur[HEUR_XY2ID(dp[0],dp[1])];
  
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
        eg_->discToCont(state->egraph_vertices[i]->neighbors[j]->coord,c_coord);
        egraphable_->downProject(c_coord,dp);
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

