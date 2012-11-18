#include<egraphs/egraph_3d_grid_heuristic.h>

#define HEUR_XYZ2ID(x,y,z) ((z + 1) * planeSize_ + (y + 1) * width_ + (x + 1))

EGraph3dGridHeuristic::EGraph3dGridHeuristic(EGraphDownProject* downProject, int size_x, int size_y, int size_z, int move_cost){
  downProject_ = downProject;
  sizex_ = size_x;
  sizey_ = size_y;
  sizez_ = size_z;
  cost_1_move_ = move_cost;

  width_ = sizex_ + 2;
  height_ = sizey_ + 2;
  length_ = sizez_ + 2;
  planeSize_ = width_ * height_;
  gridSize_ = planeSize_ * length_;
  printf("sizes: x=%d y=%d z=%d plane=%d grid=%d\n",sizex_,sizey_,sizez_,planeSize_,gridSize_);

  heur.resize(gridSize_);
  for (int i = 0; i < gridSize_; i++) {
    int x = i % width_, y = i / width_ % height_, z = i / planeSize_;
    if (x == 0 || x == width_ - 1 || y == 0 || y == height_ - 1 || z == 0 || z == length_ - 1)
      heur[i].cost = -1;
    else
      heur[i].cost = INFINITECOST;
  }

}

void EGraph3dGridHeuristic::setGrid(vector<vector<vector<bool> > >& grid){
  if(grid.size() != (unsigned int)(sizex_) ||
     grid.front().size() != (unsigned int)(sizey_) ||
     grid.front().front().size() != (unsigned int)(sizez_)){
    ROS_ERROR("[EGraph3dGridHeuristic] The dimensions provided in the constructor don't match the given grid.");
    return;
  }

  for(unsigned int x=0; x<grid.size(); x++){
    for(unsigned int y=0; y<grid[x].size(); y++){
      for(unsigned int z=0; z<grid[x][y].size(); z++){
      int id = HEUR_XYZ2ID(x,y,z);
      if(grid[x][y][z])
        heur[id].cost = -1;
      else
        heur[id].cost = INFINITECOST;
      }
    }
  }

}

void EGraph3dGridHeuristic::getEGraphVerticesWithSameHeuristic(vector<double> coord, vector<EGraph::EGraphVertex*>& vertices){
  vector<int> dp;
  downProject_->downProject(coord,dp);
  vertices.clear();
  //printf("verts in cell: %d\n",heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])].egraph_vertices.size());
  vertices = heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])].egraph_vertices;
}

void EGraph3dGridHeuristic::runPrecomputations(){
  //ROS_INFO("begin precomputations");
  //refill the cell to egraph vertex mapping
  for(int i=0; i<gridSize_; i++)
    heur[i].egraph_vertices.clear();

  vector<int> dp;
  vector<double> c_coord;
  ROS_INFO("down project edges...");
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    eg_->discToCont(eg_->id2vertex[i]->coord,c_coord);
    //ROS_INFO("size of coord %d",c_coord.size());
    downProject_->downProject(c_coord,dp);
    //ROS_INFO("size of coord %d",dp.size());
    //ROS_INFO("coord %d %d %d",dp[0],dp[1],dp[2]);
    heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])].egraph_vertices.push_back(eg_->id2vertex[i]);
    //ROS_INFO("push_back");
  }
  //ROS_INFO("done precomputations");
}

void EGraph3dGridHeuristic::setGoal(vector<double> goal){
  //ROS_ERROR("begin setGoal");
  //clear the heur data structure
  for(int i=0; i<gridSize_; i++){
    heur[i].id = i;
    heur[i].heapindex = 0;
    heur[i].closed = false;
    if(heur[i].cost!=-1)
      heur[i].cost = INFINITECOST;
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
  int id = HEUR_XYZ2ID(dp[0],dp[1],dp[2]);
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

int EGraph3dGridHeuristic::getHeuristic(vector<double> coord){
  vector<int> dp;
  downProject_->downProject(coord,dp);
  EGraph3dGridHeuristicCell* cell = &heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])];

  if(cell->cost==-1)
    return INFINITECOST;
  
  CKey key;
  //compute distance from H to all cells and note for each cell, what node in H was the closest
  while(!heap.emptyheap() && !cell->closed){
    EGraph3dGridHeuristicCell* state = (EGraph3dGridHeuristicCell*)heap.deleteminheap();
    int id = state->id;
    state->closed = true;
    int oldCost = state->cost;
    int currentCost = oldCost + inflated_cost_1_move_;
    key.key[0] = currentCost;

    HEUR_SUCCESSOR(-width_);              //-y
    HEUR_SUCCESSOR(1);                    //+x
    HEUR_SUCCESSOR(width_);               //+y
    HEUR_SUCCESSOR(-1);                   //-x
    HEUR_SUCCESSOR(-width_-1);            //-y-x
    HEUR_SUCCESSOR(-width_+1);            //-y+x
    HEUR_SUCCESSOR(width_+1);             //+y+x
    HEUR_SUCCESSOR(width_-1);             //+y-x
    HEUR_SUCCESSOR(planeSize_);           //+z
    HEUR_SUCCESSOR(-width_+planeSize_);   //+z-y
    HEUR_SUCCESSOR(1+planeSize_);         //+z+x
    HEUR_SUCCESSOR(width_+planeSize_);    //+z+y
    HEUR_SUCCESSOR(-1+planeSize_);        //+z-x
    HEUR_SUCCESSOR(-width_-1+planeSize_); //+z-y-x
    HEUR_SUCCESSOR(-width_+1+planeSize_); //+z-y+x
    HEUR_SUCCESSOR(width_+1+planeSize_);  //+z+y+x
    HEUR_SUCCESSOR(width_-1+planeSize_);  //+z+y-x
    HEUR_SUCCESSOR(-planeSize_);          //-z
    HEUR_SUCCESSOR(-width_-planeSize_);   //-z-y
    HEUR_SUCCESSOR(1-planeSize_);         //-z+x
    HEUR_SUCCESSOR(width_-planeSize_);    //-z+y
    HEUR_SUCCESSOR(-1-planeSize_);        //-z-x
    HEUR_SUCCESSOR(-width_-1-planeSize_); //-z-y-x
    HEUR_SUCCESSOR(-width_+1-planeSize_); //-z-y+x
    HEUR_SUCCESSOR(width_+1-planeSize_);  //-z+y+x
    HEUR_SUCCESSOR(width_-1-planeSize_);  //-z+y-x

    vector<double> c_coord;
    for(unsigned int i=0; i<state->egraph_vertices.size(); i++){
      for(unsigned int j=0; j<state->egraph_vertices[i]->neighbors.size(); j++){
        eg_->discToCont(state->egraph_vertices[i]->neighbors[j]->coord,c_coord);
        downProject_->downProject(c_coord,dp);
        EGraph3dGridHeuristicCell* cell = &heur[HEUR_XYZ2ID(dp[0],dp[1],dp[2])];
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

    /*
    if(cell->closed){
      FILE* fout = fopen("heur3d.csv","w");
      for(unsigned int x=0; x<width_; x++){
        for(unsigned int y=0; y<height_; y++){
          for(unsigned int z=0; z<length_; z++){
            int id = HEUR_XYZ2ID(x,y,z);
            fprintf(fout,"%d %d %d %d\n",x,y,z,heur[id].cost);
          }
        }
      }
      fclose(fout);
    }
    */
  }
  return cell->cost;
}

