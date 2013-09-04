/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <iostream>
using namespace std;

#include <sbpl/headers.h>
#include <egraphs/anytime_egraph_planner.h>




//-----------------------------------------------------------------------------------------------------

AnytimeEGraphPlanner::AnytimeEGraphPlanner(DiscreteSpaceInformation* environment, bool bSearchForward)
{
  bforwardsearch = bSearchForward;

  environment_ = environment;

  bsearchuntilfirstsolution = false;
  finitial_eps = AEG_DEFAULT_INITIAL_EPS;
  final_epsilon = AEG_FINAL_EPS;
  dec_eps = AEG_DECREASE_EPS;
  use_repair_time = false;
  repair_time = INFINITECOST;
  searchexpands = 0;
  MaxMemoryCounter = 0;

#ifndef ROS
  const char* debug = "debug.txt";
#endif
  fDeb = SBPL_FOPEN(debug, "w");
  if(fDeb == NULL){
    SBPL_ERROR("ERROR: could not open planner debug file\n");
    throw new SBPL_Exception();
  }

  pSearchStateSpace_ = new AEGSearchStateSpace_t;


  //create the AEG planner
  if(CreateSearchStateSpace(pSearchStateSpace_) != 1)
  {
    SBPL_ERROR("ERROR: failed to create statespace\n");
    return;
  }

  //set the start and goal states
  if(InitializeSearchStateSpace(pSearchStateSpace_) != 1)
  {
    SBPL_ERROR("ERROR: failed to create statespace\n");
    return;
  }    
  finitial_eps_planning_time = -1.0;
  final_eps_planning_time = -1.0;
  num_of_expands_initial_solution = 0;
  final_eps = -1.0;
}

AnytimeEGraphPlanner::~AnytimeEGraphPlanner()
{
  if(pSearchStateSpace_ != NULL){
    //delete the statespace
    DeleteSearchStateSpace(pSearchStateSpace_);
    delete pSearchStateSpace_;
  }
  SBPL_FCLOSE(fDeb);

  ROS_ERROR("shutting down?");
  boost::unique_lock<boost::mutex> lock(egraph_mutex_);
  planner_ok_ = false;
  egraph_cond_.notify_one();
  lock.unlock();

  //egraph_thread_->interrupt();
  egraph_thread_->join();
}

void AnytimeEGraphPlanner::initializeEGraph(EGraph* egraph, EGraphable* egraph_env, EGraphHeuristic* egraph_heur){
  egraph_ = egraph;
  egraph_env_ = egraph_env;
  egraph_heur_ = egraph_heur;
  egraph_heur_->initialize(egraph_);
  planner_ok_ = true;
  egraph_thread_ = new boost::thread(boost::bind(&AnytimeEGraphPlanner::updateEGraph, this));
}


void AnytimeEGraphPlanner::Initialize_searchinfo(CMDPSTATE* state, AEGSearchStateSpace_t* pSearchStateSpace)
{

  AEGState* searchstateinfo = (AEGState*)state->PlannerSpecificData;

  searchstateinfo->MDPstate = state;
  InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace); 
}


CMDPSTATE* AnytimeEGraphPlanner::CreateState(int stateID, AEGSearchStateSpace_t* pSearchStateSpace)
{	
  CMDPSTATE* state = NULL;

#if DEBUG
  if(environment_->StateID2IndexMapping[stateID][AEGMDP_STATEID2IND] != -1)
  {
    SBPL_ERROR("ERROR in CreateState: state already created\n");
    throw new SBPL_Exception();
  }
#endif

  //adds to the tail a state
  state = pSearchStateSpace->searchMDP.AddState(stateID);

  //remember the index of the state
  environment_->StateID2IndexMapping[stateID][AEGMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;

#if DEBUG
  if(state != pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][AEGMDP_STATEID2IND]])
  {
    SBPL_ERROR("ERROR in CreateState: invalid state index\n");
    throw new SBPL_Exception();
  }
#endif


  //create search specific info
  state->PlannerSpecificData = (AEGState*)malloc(sizeof(AEGState));	
  Initialize_searchinfo(state, pSearchStateSpace);
  MaxMemoryCounter += sizeof(AEGState);

  return state;

}

CMDPSTATE* AnytimeEGraphPlanner::GetState(int stateID, AEGSearchStateSpace_t* pSearchStateSpace)
{	

  if(stateID >= (int)environment_->StateID2IndexMapping.size())
  {
    SBPL_ERROR("ERROR int GetState: stateID %d is invalid\n", stateID);
    throw new SBPL_Exception();
  }

  if(environment_->StateID2IndexMapping[stateID][AEGMDP_STATEID2IND] == -1)
    return CreateState(stateID, pSearchStateSpace);
  else
    return pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][AEGMDP_STATEID2IND]];

}



//-----------------------------------------------------------------------------------------------------


inline int AnytimeEGraphPlanner::getHeuristic(vector<double>& coord){
  double t0 = ros::Time::now().toSec();
  int h = egraph_heur_->getHeuristic(coord);
  double t1 = ros::Time::now().toSec();
  timing_stats.heuristic_time += t1-t0;
  return h;
}


int AnytimeEGraphPlanner::ComputeHeuristic(CMDPSTATE* MDPstate, AEGSearchStateSpace_t* pSearchStateSpace)
{
  //compute heuristic for search

  if(bforwardsearch){
    //forward search: heur = distance from state to searchgoal which is Goal AEGState
    //int retv =  environment_->GetGoalHeuristic(MDPstate->StateID);
    //return retv;
    
    if(pSearchStateSpace->searchgoalstate->StateID==MDPstate->StateID)
      return 0;

    vector<double> coord;
    egraph_env_->getCoord(MDPstate->StateID,coord);
    return getHeuristic(coord);
  }
  else{
    //backward search: heur = distance from searchgoal to state
    //return environment_->GetStartHeuristic(MDPstate->StateID);
    return -1;
  }
}


//initialization of a state
void AnytimeEGraphPlanner::InitializeSearchStateInfo(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace)
{
  state->g = INFINITECOST;
  state->v = INFINITECOST;
  state->iterationclosed = 0;
  state->callnumberaccessed = pSearchStateSpace->callnumber;
  state->bestnextstate = NULL;
  state->costtobestnextstate = INFINITECOST;
  state->heapindex = 0;
  state->listelem[AEG_INCONS_LIST_ID] = 0;
#if DEBUG
  state->numofexpands = 0;
#endif

  state->bestpredstate = NULL;

  //compute heuristics
#if USE_HEUR
  if(pSearchStateSpace->searchgoalstate != NULL)
    state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
  else 
    state->h = 0;
#else
  state->h = 0;
#endif


}



//re-initialization of a state
void AnytimeEGraphPlanner::ReInitializeSearchStateInfo(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace)
{
  state->g = INFINITECOST;
  state->v = INFINITECOST;
  state->iterationclosed = 0;
  state->callnumberaccessed = pSearchStateSpace->callnumber;
  state->bestnextstate = NULL;
  state->costtobestnextstate = INFINITECOST;
  state->heapindex = 0;
  state->listelem[AEG_INCONS_LIST_ID] = 0;
#if DEBUG
  state->numofexpands = 0;
#endif

  state->bestpredstate = NULL;

  //compute heuristics
#if USE_HEUR

  if(pSearchStateSpace->searchgoalstate != NULL)
  {
    state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
  }
  else 
    state->h = 0;

#else

  state->h = 0;

#endif


}



void AnytimeEGraphPlanner::DeleteSearchStateData(AEGState* state)
{
  //no memory was allocated
  MaxMemoryCounter = 0;
  return;
}



//used for backward search
void AnytimeEGraphPlanner::UpdatePreds(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace)
{
  printf("Badness....... we can't do backward search yet!\n");
  exit(1);
}

void AnytimeEGraphPlanner::getDirectShortcutSuccessors(int stateID, vector<int>& SuccIDV, vector<int>& CostV){
  vector<double> coord;
  egraph_env_->getCoord(stateID,coord);
  vector<int> d_coord;
  egraph_->contToDisc(coord,d_coord);
  EGraph::EGraphVertex* v = egraph_->getVertex(d_coord);
  if(!v)
    return;

  //ROS_INFO("get direct shortcut from %d",stateID);
  vector<EGraph::EGraphVertex*> shortcuts;
  if(directShortcutCache_[v->component].empty()){
    double t0 = ros::Time::now().toSec();
    egraph_heur_->getDirectShortcut(v->component,shortcuts);
    double t1 = ros::Time::now().toSec();
    timing_stats.direct_shortcut_time += t1-t0;
    for(unsigned int i=0; i<shortcuts.size(); i++)
      directShortcutCache_[v->component].push_back(shortcuts[i]);
  }
  for(unsigned int i=0; i<directShortcutCache_[v->component].size(); i++){
    egraph_->discToCont(directShortcutCache_[v->component][i],coord);
    int id = egraph_env_->getStateID(coord);
    if(id==stateID)
      continue;
    SuccIDV.push_back(id);
    double t0 = ros::Time::now().toSec();
    CostV.push_back(egraph_->getShortestPath(v,directShortcutCache_[v->component][i]));
    double t1 = ros::Time::now().toSec();
    timing_stats.direct_shortcut_path_time += t1-t0;
    //ROS_INFO("direct shortcut %d->%d (%d)",stateID,id,CostV.back());
  }
}

void AnytimeEGraphPlanner::errorCheckEGraph(EGraph::EGraphVertex* egv){
  vector<double> eg_coord;
  egraph_->discToCont(egv,eg_coord);
  int env_id = egraph_env_->getStateID(eg_coord);
  vector<double> env_coord;
  egraph_env_->getCoord(env_id,env_coord);
  vector<int> env_dcoord;
  egraph_->contToDisc(env_coord,env_dcoord);
  EGraph::EGraphVertex* egv2 = egraph_->getVertex(env_dcoord);

  bool error = false;
  if(egv==NULL){
    error = true;
    ROS_ERROR("[AEG] ErrorCheckEGraph: The vertex is NULL!\n");
  }
  if(egv2 != egv){
    error = true;
    ROS_ERROR("[AEG] ErrorCheckEGraph: We didn't get back the egraph vertex we started with (memory addresses don't match)!\n");
  }
  if(egv2->id != egv->id){
    error = true;
    ROS_ERROR("[AEG] ErrorCheckEGraph: We didn't get back the egraph vertex we started with (egraph ids don't match)!\n");
  }

  if(error){
    printf("eg vertex address %p\n",egv);
    printf("eg vertex id: %d\n",egv->id);

    printf("eg vertex disc coord (");
    for(unsigned int i=0; i<egv->coord.size(); i++)
      printf("%d ", egv->coord[i]);
    printf(") --- constants (");
    for(unsigned int i=0; i<egv->constants.size(); i++)
      printf("%f ", egv->constants[i]);
    printf(")\n");

    printf("eg vertex cont coord (");
    for(unsigned int i=0; i<eg_coord.size(); i++)
      printf("%f ", eg_coord[i]);
    printf(")\n");

    printf("env vertex id: %d\n",env_id);
    printf("env vertex cont coord (");
    for(unsigned int i=0; i<env_coord.size(); i++)
      printf("%f ", env_coord[i]);
    printf(")\n");

    printf("env vertex disc coord (");
    for(unsigned int i=0; i<env_dcoord.size(); i++)
      printf("%d ", env_dcoord[i]);
    printf(")\n");

    printf("eg vertex address (again) %p\n",egv2);
    if(egv2!=NULL)
      printf("eg vertex id (again): %d\n",egv2->id);
    printf("press any key followed by enter to continue...\n");
    char wait;
    cin >> wait;
  }
}

void AnytimeEGraphPlanner::getGradientShortcutSuccessors(int stateID, vector<int>& SuccIDV, vector<int>& CostV){
  vector<double> coord;
  egraph_env_->getCoord(stateID,coord);
  vector<int> d_coord;
  egraph_->contToDisc(coord,d_coord);
  EGraph::EGraphVertex* v = egraph_->getVertex(d_coord);
  if(!v)
    return;

  double t0 = ros::Time::now().toSec();
  //ROS_INFO("get shortcut from %d (egraph %d)",stateID,v->id);
  //ROS_INFO("get shortcut... %d (is %d < %d)",stateID,v->shortcutIteration,pSearchStateSpace_->searchiteration);
  if(v->shortcutIteration < pSearchStateSpace_->searchiteration){
    vector<EGraph::EGraphVertex*> path;
    vector<int> costs;
    path.push_back(v);
    while(1){
      EGraph::EGraphVertex* s = path.back();
      egraph_->discToCont(s,coord);
      int s_cost = getHeuristic(coord);
      //ROS_INFO("%d (%d %d) %d",s->id,s->coord[0],s->coord[1],s_cost);
      EGraph::EGraphVertex* best_neighbor = NULL;
      int best_cost = s_cost;
      int trans_cost = 0;
      for(unsigned int i=0; i<s->neighbors.size(); i++){
        if(!s->valid[i])
          continue;
        //ROS_INFO("check da neighbors");
        EGraph::EGraphVertex* temp = s->neighbors[i];
        egraph_->discToCont(temp,coord);
        int temp_cost = getHeuristic(coord);
        //ROS_INFO("%d (%d %d) %d",temp->id,temp->coord[0],temp->coord[1],temp_cost);
        if(temp_cost < best_cost){
          best_cost = temp_cost;
          best_neighbor = temp;
          trans_cost = s->costs[i];
          //ROS_INFO("trans cost %d",trans_cost);
        }
      }
      //if we've reached a local minima then this state is its own shortcut
      //it's already in the path list from last iteration so it doesn't need to be added again
      if(best_cost == s_cost){
        //ROS_INFO("local minima found");
        s->shortcuts.push_back(s);
        s->shortcutIteration = pSearchStateSpace_->searchiteration;
        s->shortcut_costs.push_back(0);
        break;
      }
      path.push_back(best_neighbor);
      costs.push_back(trans_cost);
      //if we've reach a state that already has its shortcut filled in for this iteration
      //then we can exit early
      if(best_neighbor->shortcutIteration == pSearchStateSpace_->searchiteration){
        //ROS_INFO("cached shortcut");
        break;
      }
    }
    EGraph::EGraphVertex* theShortcut = path.back();
    //ROS_INFO("fill in path with %d (%d %d)",theShortcut->id,theShortcut->coord[0],theShortcut->coord[1]);
    for(int i=path.size()-2; i>=0; i--){
      //ROS_INFO("huh %d %d",path[i]->id,path[i+1]->id);
      path[i]->shortcuts.push_back(theShortcut);
      path[i]->shortcutIteration = pSearchStateSpace_->searchiteration;
      //ROS_INFO("hmm");
      //ROS_INFO("prev cost vec size %d",int(path[i+1]->shortcut_costs.size()));
      //ROS_INFO("prev cost %d",path[i+1]->shortcut_costs.back());
      //ROS_INFO("inc cost %d",costs[i]);
      path[i]->shortcut_costs.push_back(path[i+1]->shortcut_costs.back()+costs[i]);
    }
  }
  //ROS_INFO("done computing shortcut");

  for(unsigned int i=0; i<v->shortcuts.size(); i++){
    egraph_->discToCont(v->shortcuts[i],coord);
    int id = egraph_env_->getStateID(coord);
    if(id == stateID)
      continue;
    SuccIDV.push_back(id);
    CostV.push_back(v->shortcut_costs[i]);
    //ROS_INFO("%d %d",id,v->shortcut_costs[i]);
  }

  //ROS_INFO("ahhhh");
  //for(unsigned int i=0; i<SuccIDV.size(); i++){
    //ROS_INFO("%d %d",SuccIDV[i],CostV[i]);
  //}

  //ROS_INFO("done get shortcut");
  double t1 = ros::Time::now().toSec();
  timing_stats.gradient_shortcut_time += t1-t0;
}

void AnytimeEGraphPlanner::getSnapSuccessors(int stateID, vector<int>& SuccIDV, vector<int>& CostV){
  vector<double> coord;
  egraph_env_->getCoord(stateID,coord);
  vector<EGraph::EGraphVertex*> vertices;
  //ROS_INFO("get verts with same heur...");
  egraph_heur_->getEGraphVerticesWithSameHeuristic(coord,vertices);

  vector<double> new_coord;
  int new_id;
  int new_cost;
  //ROS_INFO("%d possible snaps",vertices.size());
  for(unsigned int i=0; i<vertices.size(); i++){
    egraph_->discToCont(vertices[i],new_coord);
    if(egraph_env_->snap(coord,new_coord,new_id,new_cost)){
      bool duplicate = false;
      for(unsigned int j=0; j<SuccIDV.size(); j++){
        if(new_id==SuccIDV[j]){
          duplicate = true;
          break;
        }
      }
      if(duplicate){
        //printf("duplicate snap...\n");
        continue;
      }
      //printf("snap from %d to %d with cost %d\n",stateID,new_id,new_cost);
      SuccIDV.push_back(new_id);
      CostV.push_back(new_cost);
    }
  }
  //ROS_INFO("done gettting snaps");
}

//used for forward search
void AnytimeEGraphPlanner::UpdateSuccs(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace)
{
  vector<int> SuccIDV;
  vector<int> CostV;
  CKey key;
  AEGState *succstate;

  //ROS_ERROR("expanding g:%d h:%d f:%d",state->v,state->h,int(state->v+state->h*pSearchStateSpace->eps));

  //Get the normal successors from the environment
  environment_->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

  for(unsigned int i=0; i<CostV.size(); i++){
    if(SuccIDV[i]!=pSearchStateSpace->searchgoalstate->StateID){
      vector<double> teh_coord;
      if(!egraph_env_->getCoord(SuccIDV[i], teh_coord)){
        ROS_ERROR("Get coord failed on a freshly generated state! (%d)",SuccIDV[i]);
        exit(1);
      }
      int temp_id = egraph_env_->getStateID(teh_coord);
      if(temp_id != SuccIDV[i]){
        ROS_ERROR("We didn't get a matching id! (%d != %d)",SuccIDV[i],temp_id);
        exit(1);
      }
    }

    if(CostV[i]<=0){
      ROS_ERROR("regular successor with cost %d",CostV[i]);
      exit(1);
    }
  }

  if(useEGraph_){
    int start_idx = SuccIDV.size();
    //if we are on the egraph, get get shortcut successors
    getDirectShortcutSuccessors(state->MDPstate->StateID,SuccIDV,CostV);
    for(unsigned int i=0; i<CostV.size(); i++){
      if(CostV[i]<=0){
        ROS_ERROR("direct shortcut with cost %d (%d->%d)",CostV[i],state->MDPstate->StateID,SuccIDV[i]);
        exit(1);
      }
    }

    getGradientShortcutSuccessors(state->MDPstate->StateID,SuccIDV,CostV);
    for(unsigned int i=0; i<CostV.size(); i++){
      if(CostV[i]<=0){
        ROS_ERROR("gradient shortcut with cost %d (%d->%d)",CostV[i],state->MDPstate->StateID,SuccIDV[i]);
        exit(1);
      }
    }

    //if the downprojected state is on the egraph, get snap successors
    getSnapSuccessors(state->MDPstate->StateID,SuccIDV,CostV);
    for(unsigned int i=0; i<CostV.size(); i++){
      if(CostV[i]<=0){
        ROS_ERROR("snap with cost %d",CostV[i]);
        exit(1);
      }
    }

    for(unsigned int i=start_idx; i<SuccIDV.size(); i++){
      if(egraph_env_->isGoal(SuccIDV[i]))
        SuccIDV[i] = pSearchStateSpace->searchgoalstate->StateID;
    }
  }

  //iterate through predecessors of s
  for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
  {
    CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
    int cost = CostV[sind];

    succstate = (AEGState*)(SuccMDPState->PlannerSpecificData);
    if(succstate->callnumberaccessed != pSearchStateSpace->callnumber)
      ReInitializeSearchStateInfo(succstate, pSearchStateSpace);

    //see if we can improve the value of succstate
    //taking into account the cost of action
    if(succstate->g > state->v + cost)
    {
      succstate->g = state->v + cost;
      succstate->bestpredstate = state->MDPstate; 

      //re-insert into heap if not closed yet
      if(succstate->iterationclosed != pSearchStateSpace->searchiteration)
      {
        //MIKE: get a fresh h-value each time (since it can change between iterations)
        succstate->h = ComputeHeuristic(succstate->MDPstate, pSearchStateSpace); 
        key.key[0] = succstate->g + (int)(pSearchStateSpace->eps*succstate->h);

        //key.key[1] = succstate->h;

        if(succstate->heapindex != 0)
          pSearchStateSpace->heap->updateheap(succstate,key);
        else
          pSearchStateSpace->heap->insertheap(succstate,key);
      }
      //take care of incons list
      else if(succstate->listelem[AEG_INCONS_LIST_ID] == NULL)
      {
        pSearchStateSpace->inconslist->insert(succstate, AEG_INCONS_LIST_ID);
      }
    } //check for cost improvement 

  } //for actions
}

//TODO-debugmax - add obsthresh and other thresholds to other environments in 3dkin
int AnytimeEGraphPlanner::GetGVal(int StateID, AEGSearchStateSpace_t* pSearchStateSpace)
{
  CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
  AEGState* state = (AEGState*)cmdp_state->PlannerSpecificData;
  return state->g;
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int AnytimeEGraphPlanner::ImprovePath(AEGSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs)
{
  int expands;
  AEGState *state, *searchgoalstate;
  CKey key, minkey;
  CKey goalkey;

  expands = 0;


  if(pSearchStateSpace->searchgoalstate == NULL)
  {
    SBPL_ERROR("ERROR searching: no goal state is set\n");
    throw new SBPL_Exception();
  }

  //goal state
  searchgoalstate = (AEGState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
  if(searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber)
    ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace);

  //set goal key
  goalkey.key[0] = searchgoalstate->g;
  //goalkey.key[1] = searchgoalstate->h;

  //expand states until done
  minkey = pSearchStateSpace->heap->getminkeyheap();
  CKey oldkey = minkey;
  while(!pSearchStateSpace->heap->emptyheap() && minkey.key[0] < INFINITECOST && goalkey > minkey &&
      (clock()-TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC && 
      (pSearchStateSpace->eps_satisfied == INFINITECOST || (clock()- TimeStarted) < repair_time*(double)CLOCKS_PER_SEC))
  {

    //get the state		
    state = (AEGState*)pSearchStateSpace->heap->deleteminheap();


#if DEBUG
    //SBPL_FPRINTF(fDeb, "expanding state(%d): h=%d g=%u key=%u v=%u iterc=%d callnuma=%d expands=%d (g(goal)=%u)\n",
    //	state->MDPstate->StateID, state->h, state->g, state->g+(int)(pSearchStateSpace->eps*state->h), state->v, 
    //	state->iterationclosed, state->callnumberaccessed, state->numofexpands, searchgoalstate->g);
    //SBPL_FPRINTF(fDeb, "expanding: ");
    //PrintSearchState(state, fDeb);
    if(state->listelem[AEG_INCONS_LIST_ID]  != NULL)
    {
      SBPL_FPRINTF(fDeb, "ERROR: expanding a state from inconslist\n");
      SBPL_ERROR("ERROR: expanding a state from inconslist\n");
      throw new SBPL_Exception();
    }
    //SBPL_FFLUSH(fDeb);
#endif

#if DEBUG
    if(minkey.key[0] < oldkey.key[0] && fabs(this->finitial_eps - 1.0) < ERR_EPS)
    {
      //SBPL_PRINTF("WARN in search: the sequence of keys decreases\n");
      //throw new SBPL_Exception();
    }
    oldkey = minkey;
#endif

    if(state->v == state->g)
    {
      SBPL_ERROR("ERROR: consistent state is being expanded\n");
#if DEBUG
      SBPL_FPRINTF(fDeb, "ERROR: consistent state is being expanded\n");
      throw new SBPL_Exception();
#endif
    }

    //recompute state value      
    state->v = state->g;
    state->iterationclosed = pSearchStateSpace->searchiteration;

    //new expand      
    expands++;
#if DEBUG
    state->numofexpands++;
#endif


    if(bforwardsearch == false)
      UpdatePreds(state, pSearchStateSpace);
    else
      UpdateSuccs(state, pSearchStateSpace);

    //recompute minkey
    minkey = pSearchStateSpace->heap->getminkeyheap();

    //recompute goalkey if necessary
    if(goalkey.key[0] != (int)searchgoalstate->g)
    {
      //SBPL_PRINTF("re-computing goal key\n");
      //recompute the goal key (heuristics should be zero)
      goalkey.key[0] = searchgoalstate->g;
      //goalkey.key[1] = searchgoalstate->h;
    }

    if(expands%100000 == 0 && expands > 0)
    {
      SBPL_PRINTF("expands so far=%u\n", expands);
    }

  }

  int retv = 1;
  if(searchgoalstate->g == INFINITECOST && pSearchStateSpace->heap->emptyheap())
  {
    SBPL_PRINTF("solution does not exist: search exited because heap is empty\n");
    retv = 0;
  }
  else if(!pSearchStateSpace->heap->emptyheap() && goalkey > minkey)
  {
    SBPL_PRINTF("search exited because it ran out of time\n");
    retv = 2;
  }
  else if(searchgoalstate->g == INFINITECOST && !pSearchStateSpace->heap->emptyheap())
  {
    SBPL_PRINTF("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
    retv = 0;
  }
  else
  {
    SBPL_PRINTF("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps*pSearchStateSpace->epsE);
    retv = 1;
  }

  //SBPL_FPRINTF(fDeb, "expanded=%d\n", expands);

  searchexpands += expands;

  return retv;		
}


void AnytimeEGraphPlanner::BuildNewOPENList(AEGSearchStateSpace_t* pSearchStateSpace)
{
  AEGState *state;
  CKey key;
  CHeap* pheap = pSearchStateSpace->heap;
  CList* pinconslist = pSearchStateSpace->inconslist; 

  //move incons into open
  while(pinconslist->firstelement != NULL)
  {
    state = (AEGState*)pinconslist->firstelement->liststate;

    //compute f-value
    state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
    key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h);
    //key.key[1] = state->h;

    //insert into OPEN
    pheap->insertheap(state, key);
    //remove from INCONS
    pinconslist->remove(state, AEG_INCONS_LIST_ID);
  }
}


void AnytimeEGraphPlanner::Reevaluatefvals(AEGSearchStateSpace_t* pSearchStateSpace)
{
  CKey key;
  int i;
  CHeap* pheap = pSearchStateSpace->heap;

  //recompute priorities for states in OPEN and reorder it
  for (i = 1; i <= pheap->currentsize; ++i)
  {
    AEGState* state = (AEGState*)pheap->heap[i].heapstate;
    state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace); 
    pheap->heap[i].key.key[0] = state->g + 
      (int)(pSearchStateSpace->eps*state->h); 
    //pheap->heap[i].key.key[1] = state->h; 
  }
  pheap->makeheap();

  pSearchStateSpace->bReevaluatefvals = false;
}




//creates (allocates memory) search state space
//does not initialize search statespace
int AnytimeEGraphPlanner::CreateSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace)
{

  //create a heap
  pSearchStateSpace->heap = new CHeap;
  pSearchStateSpace->inconslist = new CList;
  MaxMemoryCounter += sizeof(CHeap);
  MaxMemoryCounter += sizeof(CList);

  pSearchStateSpace->searchgoalstate = NULL;
  pSearchStateSpace->searchstartstate = NULL;

  searchexpands = 0;


  pSearchStateSpace->bReinitializeSearchStateSpace = false;

  return 1;
}

//deallocates memory used by SearchStateSpace
void AnytimeEGraphPlanner::DeleteSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace)
{
  if(pSearchStateSpace->heap != NULL)
  {
    pSearchStateSpace->heap->makeemptyheap();
    delete pSearchStateSpace->heap;
    pSearchStateSpace->heap = NULL;
  }

  if(pSearchStateSpace->inconslist != NULL)
  {
    pSearchStateSpace->inconslist->makeemptylist(AEG_INCONS_LIST_ID);
    delete pSearchStateSpace->inconslist;
    pSearchStateSpace->inconslist = NULL;
  }

  //delete the states themselves
  int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
  for(int i=0; i < iend; i++)
  {
    CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
    if(state != NULL && state->PlannerSpecificData != NULL){
      DeleteSearchStateData((AEGState*)state->PlannerSpecificData);
      free((AEGState*)state->PlannerSpecificData);
      state->PlannerSpecificData = NULL;
    }
  }
  pSearchStateSpace->searchMDP.Delete();
}



//reset properly search state space
//needs to be done before deleting states
int AnytimeEGraphPlanner::ResetSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace)
{
  pSearchStateSpace->heap->makeemptyheap();
  pSearchStateSpace->inconslist->makeemptylist(AEG_INCONS_LIST_ID);

  return 1;
}

//initialization before each search
void AnytimeEGraphPlanner::ReInitializeSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace)
{
  CKey key;

  //increase callnumber
  pSearchStateSpace->callnumber++;

  //reset iteration
  pSearchStateSpace->searchiteration = 0;
  pSearchStateSpace->bNewSearchIteration = true;

#if DEBUG
  SBPL_FPRINTF(fDeb, "reinitializing search state-space (new call number=%d search iter=%d)\n", 
      pSearchStateSpace->callnumber,pSearchStateSpace->searchiteration );
#endif



  pSearchStateSpace->heap->makeemptyheap();
  pSearchStateSpace->inconslist->makeemptylist(AEG_INCONS_LIST_ID);

  //reset 
  pSearchStateSpace->eps = this->finitial_eps;
  pSearchStateSpace->eps_satisfied = INFINITECOST;

  //initialize start state
  AEGState* startstateinfo = (AEGState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
  if(startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber)
    ReInitializeSearchStateInfo(startstateinfo, pSearchStateSpace);

  startstateinfo->g = 0;

  //insert start state into the heap
  startstateinfo->h = ComputeHeuristic(startstateinfo->MDPstate, pSearchStateSpace); 
  key.key[0] = (long int)(pSearchStateSpace->eps*startstateinfo->h);
  //key.key[1] = startstateinfo->h;
  pSearchStateSpace->heap->insertheap(startstateinfo, key);

  pSearchStateSpace->bReinitializeSearchStateSpace = false;
  pSearchStateSpace->bReevaluatefvals = false;
}

//very first initialization
int AnytimeEGraphPlanner::InitializeSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace)
{

  if(pSearchStateSpace->heap->currentsize != 0 || 
      pSearchStateSpace->inconslist->currentsize != 0)
  {
    SBPL_ERROR("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
    throw new SBPL_Exception();
  }

  pSearchStateSpace->eps = this->finitial_eps;
  pSearchStateSpace->eps_satisfied = INFINITECOST;
  pSearchStateSpace->searchiteration = 0;
  pSearchStateSpace->bNewSearchIteration = true;
  pSearchStateSpace->callnumber = 0;
  pSearchStateSpace->bReevaluatefvals = false;


  //create and set the search start state
  pSearchStateSpace->searchgoalstate = NULL;
  //pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);
  pSearchStateSpace->searchstartstate = NULL;


  pSearchStateSpace->bReinitializeSearchStateSpace = true;

  return 1;

}


int AnytimeEGraphPlanner::SetSearchGoalState(int SearchGoalStateID, AEGSearchStateSpace_t* pSearchStateSpace)
{
  if(pSearchStateSpace->searchgoalstate == NULL || 
      pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
  {
    pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);

    //should be new search iteration
    pSearchStateSpace->eps_satisfied = INFINITECOST;
    pSearchStateSpace->bNewSearchIteration = true;
    pSearchStateSpace_->eps = this->finitial_eps;


    //recompute heuristic for the heap if heuristics is used
#if USE_HEUR
/*
    for(int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++)
    {
      CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
      AEGState* state = (AEGState*)MDPstate->PlannerSpecificData;
      state->h = ComputeHeuristic(MDPstate, pSearchStateSpace);
    }

    pSearchStateSpace->bReevaluatefvals = true;
    */
#endif
  }


  return 1;

}


int AnytimeEGraphPlanner::SetSearchStartState(int SearchStartStateID, AEGSearchStateSpace_t* pSearchStateSpace)
{

  CMDPSTATE* MDPstate = GetState(SearchStartStateID, pSearchStateSpace);

  if(MDPstate !=  pSearchStateSpace->searchstartstate)
  {	
    pSearchStateSpace->searchstartstate = MDPstate;
    pSearchStateSpace->bReinitializeSearchStateSpace = true;
  }

  return 1;

}



int AnytimeEGraphPlanner::ReconstructPath(AEGSearchStateSpace_t* pSearchStateSpace)
{	


  if(bforwardsearch) //nothing to do, if search is backward
  {
    CMDPSTATE* MDPstate = pSearchStateSpace->searchgoalstate;
    CMDPSTATE* PredMDPstate;
    AEGState *predstateinfo, *stateinfo;



#if DEBUG
    SBPL_FPRINTF(fDeb, "reconstructing a path:\n");
#endif

    while(MDPstate != pSearchStateSpace->searchstartstate)
    {
      stateinfo = (AEGState*)MDPstate->PlannerSpecificData;

#if DEBUG
      PrintSearchState(stateinfo, fDeb);
#endif
      if(stateinfo->g == INFINITECOST)
      {	
        //SBPL_ERROR("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
        //throw new SBPL_Exception();
        return -1;
      }

      if(stateinfo->bestpredstate == NULL)
      {
        SBPL_ERROR("ERROR in ReconstructPath: bestpred is NULL\n");
        throw new SBPL_Exception();
      }

      //get the parent state
      PredMDPstate = stateinfo->bestpredstate;
      predstateinfo = (AEGState*)PredMDPstate->PlannerSpecificData;

      //set its best next info
      predstateinfo->bestnextstate = MDPstate;

      //check the decrease of g-values along the path
      if(predstateinfo->v >= stateinfo->g)
      {
        SBPL_ERROR("ERROR in ReconstructPath: g-values are non-decreasing\n");			
        PrintSearchState(predstateinfo, fDeb);
        throw new SBPL_Exception();
      }

      //transition back
      MDPstate = PredMDPstate;
    }
  }

  return 1;
}



void AnytimeEGraphPlanner::PrintSearchPath(AEGSearchStateSpace_t* pSearchStateSpace, FILE* fOut)
{
  AEGState* searchstateinfo;
  CMDPSTATE* state;
  int goalID;
  int PathCost;

  if(bforwardsearch)
  {
    state  = pSearchStateSpace->searchstartstate;
    goalID = pSearchStateSpace->searchgoalstate->StateID;
  }
  else
  {
    state = pSearchStateSpace->searchgoalstate;
    goalID = pSearchStateSpace->searchstartstate->StateID;
  }
  if(fOut == NULL)
    fOut = stdout;

  PathCost = ((AEGState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;

  SBPL_FPRINTF(fOut, "Printing a path from state %d to the goal state %d\n", 
      state->StateID, pSearchStateSpace->searchgoalstate->StateID);
  SBPL_FPRINTF(fOut, "Path cost = %d:\n", PathCost);


  environment_->PrintState(state->StateID, false, fOut);

  int costFromStart = 0;
  while(state->StateID != goalID)
  {
    SBPL_FPRINTF(fOut, "state %d ", state->StateID);

    if(state->PlannerSpecificData == NULL)
    {
      SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
      break;
    }

    searchstateinfo = (AEGState*)state->PlannerSpecificData;

    if(searchstateinfo->bestnextstate == NULL)
    {
      SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
      break;
    }
    if(searchstateinfo->g == INFINITECOST)
    {
      SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
      break;
    }

    int costToGoal = PathCost - costFromStart;
    int transcost = searchstateinfo->g - ((AEGState*)(searchstateinfo->bestnextstate->PlannerSpecificData))->v;
    if(bforwardsearch)
      transcost = -transcost;

    costFromStart += transcost;

    SBPL_FPRINTF(fOut, "g=%d-->state %d, h = %d ctg = %d  ", searchstateinfo->g, 			
        searchstateinfo->bestnextstate->StateID, searchstateinfo->h, costToGoal);

    state = searchstateinfo->bestnextstate;

    environment_->PrintState(state->StateID, false, fOut);



  }
}

void AnytimeEGraphPlanner::PrintSearchState(AEGState* state, FILE* fOut)
{
#if DEBUG
  SBPL_FPRINTF(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d expands=%d heapind=%d inconslist=%d\n",
      state->MDPstate->StateID, state->h, state->g, state->v, 
      state->iterationclosed, state->callnumberaccessed, state->numofexpands, state->heapindex, state->listelem[AEG_INCONS_LIST_ID]?1:0);
#else
  SBPL_FPRINTF(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d heapind=%d inconslist=%d\n",
      state->MDPstate->StateID, state->h, state->g, state->v, 
      state->iterationclosed, state->callnumberaccessed, state->heapindex, state->listelem[AEG_INCONS_LIST_ID]?1:0);
#endif
  environment_->PrintState(state->MDPstate->StateID, true, fOut);

}



int AnytimeEGraphPlanner::getHeurValue(AEGSearchStateSpace_t* pSearchStateSpace, int StateID)
{
  CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
  AEGState* searchstateinfo = (AEGState*)MDPstate->PlannerSpecificData;
  return searchstateinfo->h;
}

void AnytimeEGraphPlanner::getDirectShortcutPath(int fromID, int toID, vector<int>& ids){
  vector<double> coord;
  vector<int> d_coord;

  egraph_env_->getCoord(fromID,coord);
  egraph_->contToDisc(coord,d_coord);
  EGraph::EGraphVertex* v1 = egraph_->getVertex(d_coord);

  egraph_env_->getCoord(toID,coord);
  egraph_->contToDisc(coord,d_coord);
  EGraph::EGraphVertex* v2 = egraph_->getVertex(d_coord);

  vector<EGraph::EGraphVertex*> path;
  vector<int> costs;
  double t0 = ros::Time::now().toSec();
  egraph_->getShortestPath(v1,v2,&path,&costs);
  double t1 = ros::Time::now().toSec();
  timing_stats.direct_shortcut_path_time += t1-t0;

  for(unsigned int i=1; i<path.size(); i++){
    EGraph::EGraphVertex* v = path[i];
    egraph_->discToCont(v,coord);
    ids.push_back(egraph_env_->getStateID(coord));
    if(i<path.size()-1)
      egraph_path_.push_back(coord);
  }
  for(unsigned int i=0; i<costs.size(); i++)
    egraph_path_costs_.push_back(costs[i]);
}

//the id path (fromID,toID]
void AnytimeEGraphPlanner::getGradientShortcutPath(int fromID, int toID, int cost, vector<int>& ids){
  //ROS_INFO("[AEGPlanner] begin getGradientShortcutPath");
  //printf("%d %d %d\n",fromID,toID,cost);
  double t0 = ros::Time::now().toSec();
  vector<double> coord;
  vector<int> d_coord;
  egraph_env_->getCoord(fromID,coord);
  egraph_->contToDisc(coord,d_coord);
  EGraph::EGraphVertex* v1 = egraph_->getVertex(d_coord);
  egraph_env_->getCoord(toID,coord);
  //printf("the coord goes (%f %f %f)\n",coord[0],coord[1],coord[2]);
  //printf("%f/%f=%f\n",coord[2],2*M_PI/16,coord[2]/(2*M_PI/16));
  egraph_->contToDisc(coord,d_coord);
  //printf("the d_coord goes (%d %d %d)\n",d_coord[0],d_coord[1],d_coord[2]);
  EGraph::EGraphVertex* v2 = egraph_->getVertex(d_coord);

  EGraph::EGraphVertex* v = v1;

  ids.clear();
  //ROS_INFO("looking for vertex %d",v2->id);
  while(v!=v2){
    //ROS_INFO("at vertex %d",v->id);
    bool found = false;
    for(unsigned int i=0; i<v->neighbors.size(); i++){
      if(!v->valid[i])
        continue;
      //ROS_INFO("  neighbor %d %d",v->neighbors[i]->id,v->costs[i]);
      for(unsigned int j=0; j<v->neighbors[i]->shortcuts.size(); j++){
        //ROS_INFO("    shortcut %d %d",v->neighbors[i]->shortcuts[j]->id,v->neighbors[i]->shortcut_costs[j]);
        if(v->neighbors[i]->shortcuts[j]==v2 && v->neighbors[i]->shortcut_costs[j]+v->costs[i] == cost){
          //ROS_INFO("found!");
          found = true;
          int c = v->costs[i];
          cost -= c;
          v = v->neighbors[i];
          egraph_->discToCont(v,coord);
          ids.push_back(egraph_env_->getStateID(coord));
          
          //ROS_INFO("wooo pushin the costs back %d",c);
          egraph_path_costs_.push_back(c);
          if(v!=v2){
            //ROS_INFO("....and the vert %d",v->id);
            egraph_path_.push_back(coord);
          }

          break;
        }
      }
      if(found)
        break;
    }
  }
  //ROS_INFO("[AEGPlanner] end getGradientShortcutPath");
  double t1 = ros::Time::now().toSec();
  timing_stats.gradient_shortcut_path_time += t1-t0;
}


vector<int> AnytimeEGraphPlanner::GetSearchPath(AEGSearchStateSpace_t* pSearchStateSpace, int& solcost)
{
  int shortcutCount = 0;
  vector<int> SuccIDV;
  vector<int> CostV;
  vector<int> wholePathIds;
  AEGState* searchstateinfo;
  CMDPSTATE* state = NULL; 
  CMDPSTATE* goalstate = NULL;
  CMDPSTATE* startstate=NULL;

  egraph_path_.clear();
  egraph_path_costs_.clear();

  if(bforwardsearch)
  {	
    startstate = pSearchStateSpace->searchstartstate;
    goalstate = pSearchStateSpace->searchgoalstate;

    //reconstruct the path by setting bestnextstate pointers appropriately
    ReconstructPath(pSearchStateSpace);
  }
  else
  {
    startstate = pSearchStateSpace->searchgoalstate;
    goalstate = pSearchStateSpace->searchstartstate;
  }


  state = startstate;

  wholePathIds.push_back(state->StateID);
  solcost = 0;

  vector<double> coord;
  //ROS_INFO("calling getCoord!");
  egraph_env_->getCoord(state->StateID,coord);
  //ROS_INFO("done getCoord!");
  egraph_path_.push_back(coord);

  FILE* fOut = stdout;
  if(fOut == NULL){
    SBPL_ERROR("ERROR: could not open file\n");
    throw new SBPL_Exception();
  }
  while(state->StateID != goalstate->StateID)
  {
    if(state->PlannerSpecificData == NULL)
    {
      SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
      break;
    }

    searchstateinfo = (AEGState*)state->PlannerSpecificData;

    if(searchstateinfo->bestnextstate == NULL)
    {
      SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
      break;
    }
    if(searchstateinfo->g == INFINITECOST)
    {
      SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
      break;
    }

    ///////////////////////////////////
    // Begin regular edge check
    ///////////////////////////////////
    environment_->GetSuccs(state->StateID, &SuccIDV, &CostV);
    int actioncost = INFINITECOST;
    for(int i = 0; i < (int)SuccIDV.size(); i++){   
      if(SuccIDV[i] == searchstateinfo->bestnextstate->StateID && CostV[i]<actioncost)
        actioncost = CostV[i];
    }
    int shortcut_id = -1;
    int snap_id = -1;
    if(useEGraph_ && actioncost == INFINITECOST){
      ///////////////////////////////////
      // Begin direct shortcut check
      ///////////////////////////////////
      SuccIDV.clear();
      CostV.clear();
      getDirectShortcutSuccessors(state->StateID,SuccIDV,CostV);
      unsigned int i;
      //ROS_INFO("direct shortcut maybe? from %d",state->StateID);
      for(i=0; i<SuccIDV.size(); i++){
        //ROS_INFO("checking %d %d",searchstateinfo->bestnextstate->StateID,CostV[i]);
        if(SuccIDV[i] == searchstateinfo->bestnextstate->StateID && CostV[i]<actioncost){
          actioncost = CostV[i];
          shortcut_id = searchstateinfo->bestnextstate->StateID;
        }
        else if(egraph_env_->isGoal(SuccIDV[i]) && searchstateinfo->bestnextstate->StateID == goalstate->StateID && CostV[i]<actioncost){
          actioncost = CostV[i];
          shortcut_id = SuccIDV[i];
          //ROS_INFO("the direct shortcut goes to the goal %d", shortcut_id);
        }
      }
      if(actioncost < INFINITECOST){
        //we used a direct shortcut, fill in the sub-path
        vector<int> shortcut_path;
        //ROS_INFO("get the direct shortcut path %d %d %d",state->StateID,shortcut_id,actioncost);
        getDirectShortcutPath(state->StateID,shortcut_id,shortcut_path);
        for(unsigned int j=0; j<shortcut_path.size()-1; j++)
          wholePathIds.push_back(shortcut_path[j]);
        shortcutCount += shortcut_path.size();
      }
      else{
        ///////////////////////////////////
        // Begin gradient shortcut check
        ///////////////////////////////////
        getGradientShortcutSuccessors(state->StateID,SuccIDV,CostV);
        unsigned int i;
        //ROS_INFO("gradient shortcut maybe? from %d",state->StateID);
        for(i=0; i<SuccIDV.size(); i++){
          //ROS_INFO("checking %d %d",searchstateinfo->bestnextstate->StateID,CostV[i]);
          if(SuccIDV[i] == searchstateinfo->bestnextstate->StateID && CostV[i]<actioncost){
            actioncost = CostV[i];
            shortcut_id = searchstateinfo->bestnextstate->StateID;
          }
          else if(egraph_env_->isGoal(SuccIDV[i]) && searchstateinfo->bestnextstate->StateID == goalstate->StateID && CostV[i]<actioncost){
            actioncost = CostV[i];
            shortcut_id = SuccIDV[i];
          }
        }
        if(actioncost < INFINITECOST){
          //we used a shortcut, fill in the sub-path
          vector<int> shortcut_path;
          getGradientShortcutPath(state->StateID,shortcut_id,actioncost,shortcut_path);
          for(unsigned int j=0; j<shortcut_path.size()-1; j++)
            wholePathIds.push_back(shortcut_path[j]);
          shortcutCount += shortcut_path.size();
        }
        else{
          ///////////////////////////////////
          // Begin snap check
          ///////////////////////////////////
          //ROS_INFO("nope...it's a snap");
          SuccIDV.clear();
          CostV.clear();
          getSnapSuccessors(state->StateID,SuccIDV,CostV);
          for(unsigned int i=0; i<SuccIDV.size(); i++){
            if((SuccIDV[i] == searchstateinfo->bestnextstate->StateID || (egraph_env_->isGoal(SuccIDV[i]) && searchstateinfo->bestnextstate->StateID == goalstate->StateID)) && CostV[i]<actioncost){
              actioncost = CostV[i];
              snap_id = SuccIDV[i];
            }
          }
        }
      }
    }
    if(actioncost == INFINITECOST)
      SBPL_PRINTF("WARNING: actioncost = %d\n", actioncost);
    environment_->PrintState(state->StateID, true);
    ROS_INFO("%d->%d for %d please.",state->StateID,searchstateinfo->bestnextstate->StateID,actioncost);
    environment_->PrintState(searchstateinfo->bestnextstate->StateID, true);

    solcost += actioncost;

    state = searchstateinfo->bestnextstate;

    wholePathIds.push_back(state->StateID);

    //if we used a shortcut...in this case we already added
    if(egraph_path_costs_.size() < egraph_path_.size())
      egraph_path_costs_.push_back(actioncost);

    if(state->StateID == goalstate->StateID){
      //printf("***");
      if(shortcut_id>=0){
        egraph_env_->getCoord(shortcut_id,coord);
      }
      else if(snap_id>0){
        egraph_env_->getCoord(snap_id,coord);
      }
      else
        egraph_env_->getGoalCoord(egraph_path_.back(),coord);
    }
    else{
      //printf(".");
      //ROS_INFO("calling getCoord!");
      egraph_env_->getCoord(state->StateID,coord);
      //ROS_INFO("done getCoord!");
    }
    egraph_path_.push_back(coord);
  }

  //TODO: confirm that we can get the state ids back from the coordinates
  for(unsigned int i=0; i<egraph_path_.size(); i++){
    int id = egraph_env_->getStateID(egraph_path_[i]);
    if(id != wholePathIds[i])
      ROS_WARN("uh oh. %d is not %d",id,wholePathIds[i]);
  }


  ROS_INFO("[AEGPlanner] %f percent of the path came from shortcuts (%d)\n",((double)shortcutCount)/(wholePathIds.size()-1),shortcutCount);
  return wholePathIds;
}



bool AnytimeEGraphPlanner::Search(AEGSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs)
{
  CKey key;
  TimeStarted = clock();
  searchexpands = 0;
  double old_repair_time = repair_time;
  if(!use_repair_time)
    repair_time = MaxNumofSecs;

#if DEBUG
  SBPL_FPRINTF(fDeb, "new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif

  if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
    //re-initialize state space 
    ReInitializeSearchStateSpace(pSearchStateSpace);
  }


  if(bOptimalSolution)
  {
    pSearchStateSpace->eps = 1;
    MaxNumofSecs = INFINITECOST;
    repair_time = INFINITECOST;
  }
  else if(bFirstSolution)
  {
    MaxNumofSecs = INFINITECOST;
    repair_time = INFINITECOST;
  }

  //ensure heuristics are up-to-date
  environment_->EnsureHeuristicsUpdated((bforwardsearch==true));

  //the main loop of AEG*
  stats.clear();
  int prevexpands = 0;
  clock_t loop_time;
  printf("eps_sat=%f final_eps=%f epsE=%f\n",pSearchStateSpace->eps_satisfied,final_epsilon,pSearchStateSpace->epsE);
  while(pSearchStateSpace->eps_satisfied > final_epsilon*final_epsE && 
      (clock()- TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC &&
      (pSearchStateSpace->eps_satisfied == INFINITECOST || (clock()- TimeStarted) < repair_time*(double)CLOCKS_PER_SEC))
  {
    printf("eps_sat=%f final_eps=%f epsE=%f\n",pSearchStateSpace->eps_satisfied,final_epsilon,pSearchStateSpace->epsE);
    loop_time = clock();
    //decrease eps for all subsequent iterations
    if(fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps*pSearchStateSpace->epsE) < ERR_EPS && !bFirstSolution)
    {
      if(pSearchStateSpace->epsE == final_epsE){
        pSearchStateSpace->eps = pSearchStateSpace->eps - dec_eps;
        if(pSearchStateSpace->eps < final_epsilon)
          pSearchStateSpace->eps = final_epsilon;
      }
      else{
        pSearchStateSpace->epsE -= dec_epsE;
        if(pSearchStateSpace->epsE < final_epsE)
          pSearchStateSpace->epsE = final_epsE;
        egraph_heur_->setEpsE(pSearchStateSpace_->epsE);
      }

      //the priorities need to be updated
      pSearchStateSpace->bReevaluatefvals = true; 

      //it will be a new search
      pSearchStateSpace->bNewSearchIteration = true;

      //build a new open list by merging it with incons one
      BuildNewOPENList(pSearchStateSpace); 

    }

    if(pSearchStateSpace->bNewSearchIteration)
    {
      pSearchStateSpace->searchiteration++;
      pSearchStateSpace->bNewSearchIteration = false;
    }

    //re-compute f-values if necessary and reorder the heap
    if(pSearchStateSpace->bReevaluatefvals) 
      Reevaluatefvals(pSearchStateSpace);

    //improve or compute path
    if(ImprovePath(pSearchStateSpace, MaxNumofSecs) == 1){
      pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps*pSearchStateSpace->epsE;
      printf("eps_sat=%f eps=%f epsE=%f\n",pSearchStateSpace->eps_satisfied,pSearchStateSpace->eps,pSearchStateSpace->epsE);
    }

    //print the solution cost and eps bound
    SBPL_PRINTF("eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,
        ((AEGState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);

    if(pSearchStateSpace->eps_satisfied == finitial_eps*initial_epsE)
    {
      finitial_eps_planning_time = double(clock()-loop_time)/CLOCKS_PER_SEC;
      num_of_expands_initial_solution = searchexpands - prevexpands;
    }

    if(stats.empty() || pSearchStateSpace->eps_satisfied != stats.back().eps){
      EGraphPlannerStats tempStat;
      tempStat.bound = pSearchStateSpace->eps_satisfied;
      tempStat.eps = pSearchStateSpace->eps;
      tempStat.epsE = pSearchStateSpace->epsE;
      tempStat.expands = searchexpands-prevexpands;
      tempStat.time = double(clock()-loop_time)/CLOCKS_PER_SEC;
      tempStat.cost = ((AEGState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
      stats.push_back(tempStat);
    }

#if DEBUG
    SBPL_FPRINTF(fDeb, "eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands,
        ((AEGState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock()-loop_time)/CLOCKS_PER_SEC);
    PrintSearchState((AEGState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData, fDeb);
#endif
    prevexpands = searchexpands;


    //if just the first solution then we are done
    if(bFirstSolution)
      break;

    //no solution exists
    if(((AEGState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST)
      break;

  }
  repair_time = old_repair_time;
  ROS_ERROR("Main loop done");


#if DEBUG
  SBPL_FFLUSH(fDeb);
#endif

  PathCost = ((AEGState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
  MaxMemoryCounter += environment_->StateID2IndexMapping.size()*sizeof(int);

  SBPL_PRINTF("MaxMemoryCounter = %d\n", MaxMemoryCounter);

  int solcost = INFINITECOST;
  bool ret = false;
  //if(PathCost == INFINITECOST)
  if(pSearchStateSpace->eps_satisfied == INFINITECOST)
  {
    SBPL_PRINTF("could not find a solution\n");
    ret = false;
  }
  else
  {
    SBPL_PRINTF("solution is found\n");      
    pathIds = GetSearchPath(pSearchStateSpace, solcost);
    ret = true;

    //TODO:Mike pick up here!!! wake up the maintain egraph thread!
    ROS_ERROR("wake up the thread...");
    boost::unique_lock<boost::mutex> lock(egraph_mutex_);
    egraph_cond_.notify_one();
    lock.unlock();
    while(!egraph_path_costs_.empty()){sleep(0.1);}
  }

  SBPL_PRINTF("total expands this call = %d, planning time = %.3f secs, solution cost=%d\n", 
      searchexpands, (clock()-TimeStarted)/((double)CLOCKS_PER_SEC), solcost);
  final_eps_planning_time = (clock()-TimeStarted)/((double)CLOCKS_PER_SEC);
  final_eps = pSearchStateSpace->eps_satisfied;
  //SBPL_FPRINTF(fStat, "%d %d\n", searchexpands, solcost);

  return ret;

}

void AnytimeEGraphPlanner::updateEGraph(){
  boost::unique_lock<boost::mutex> lock(egraph_mutex_);

  while(1){
    //the mutex is locked
    ROS_INFO("[AEGPlanner] Maintain EGraph thread suspending...\n");
    egraph_cond_.wait(lock);

    if(!planner_ok_)
      break;

    double t0 = ros::Time::now().toSec();
    ROS_INFO("[AEGPlanner] Maintain EGraph thread awake!\n");
    //lock.unlock();

    for(unsigned int i=0; i<egraph_->id2vertex.size(); i++){
      EGraph::EGraphVertex* v = egraph_->id2vertex[i];
      v->shortcuts.clear();
      v->shortcut_costs.clear();
      v->shortcutIteration = 0;
      v->search_iteration = 0;
    }
    egraph_->search_iteration_ = 0;

    if(update_stats)
      egraph_->recordStats(egraph_path_);

    if(feedback_path){
      //adds the new path to the e-graph and runs heuristic precomputations (like down projections of th e-graph)
      egraph_->addPath(egraph_path_,egraph_path_costs_);
      double t2 = ros::Time::now().toSec();
      for(unsigned int i=0; i<egraph_->id2vertex.size(); i++){
        EGraph::EGraphVertex* v = egraph_->id2vertex[i];
        vector<double> coord;
        egraph_->discToCont(v,coord);
        //ROS_INFO("egraph2env ids: %d",egraph_env_->getStateID(coord));
        errorCheckEGraph(v);
      }
      double t3 = ros::Time::now().toSec();
      timing_stats.errorCheckEGraphTime += t3-t2;
      egraph_heur_->runPrecomputations();
    }
    egraph_path_.clear();
    egraph_path_costs_.clear();
    double t1 = ros::Time::now().toSec();
    timing_stats.updateEGraphTime += t1-t0;
  }
}


//-----------------------------Interface function-----------------------------------------------------

int AnytimeEGraphPlanner::replan(vector<int>* solution_stateIDs_V, EGraphReplanParams params){
  int solcost;
  return replan(solution_stateIDs_V, params, &solcost);
}
int AnytimeEGraphPlanner::replan(vector<int>* solution_stateIDs_V, EGraphReplanParams params, int* solcost){
  useEGraph_ = params.use_egraph;
  if(useEGraph_){
    pSearchStateSpace_->epsE = params.epsE;
    initial_epsE = params.epsE;
    final_epsE = params.final_epsE;
  }
  else{
    pSearchStateSpace_->epsE = 1.0;
    initial_epsE = 1.0;
    final_epsE = 1.0;
  }
  dec_epsE = params.dec_epsE;
  egraph_heur_->setEpsE(pSearchStateSpace_->epsE);
  finitial_eps = params.initial_eps;
  final_epsilon = params.final_eps;
  dec_eps = params.dec_eps;
  bsearchuntilfirstsolution = params.return_first_solution;
  use_repair_time = params.repair_time > 0;
  repair_time = params.repair_time;
  feedback_path = params.feedback_path;
  update_stats = params.update_stats;
  int ret = replan(params.max_time, solution_stateIDs_V, solcost);
  return ret;
}

//returns 1 if found a solution, and 0 otherwise
int AnytimeEGraphPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
  int solcost;

  return replan(allocated_time_secs, solution_stateIDs_V, &solcost);

}

//returns 1 if found a solution, and 0 otherwise
int AnytimeEGraphPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost)
{
  vector<int> pathIds; 
  bool bFound = false;
  int PathCost;
  bool bFirstSolution = this->bsearchuntilfirstsolution;
  bool bOptimalSolution = false;
  *psolcost = 0;

  SBPL_PRINTF("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);

  timing_stats.heuristic_time = 0;
  timing_stats.direct_shortcut_time = 0;
  timing_stats.direct_shortcut_path_time = 0;
  timing_stats.gradient_shortcut_time = 0;
  timing_stats.gradient_shortcut_path_time = 0;
  timing_stats.errorCheckEGraphTime = 0;
  timing_stats.updateEGraphTime = 0;

  //plan
  if((bFound = Search(pSearchStateSpace_, pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false) 
  {
    SBPL_PRINTF("failed to find a solution\n");
  }

  //copy the solution
  *solution_stateIDs_V = pathIds;
  *psolcost = PathCost;

  return (int)bFound;

}


int AnytimeEGraphPlanner::set_goal(int goal_stateID)
{

  SBPL_PRINTF("planner: setting goal to %d\n", goal_stateID);
  environment_->PrintState(goal_stateID, true, stdout);
  vector<double> coord;
  //egraph_env_->getCoord(goal_stateID,coord);

  double t0 = ros::Time::now().toSec();
  egraph_env_->getGoalHeuristicCoord(coord);
  egraph_heur_->setGoal(coord);
  directShortcutCache_.clear();
  directShortcutCache_.resize(egraph_->getNumComponents());
  double t1 = ros::Time::now().toSec();
  timing_stats.initial_heuristic_time = t1-t0;

  if(bforwardsearch)
  {	
    if(SetSearchGoalState(goal_stateID, pSearchStateSpace_) != 1)
    {
      SBPL_ERROR("ERROR: failed to set search goal state\n");
      return 0;
    }
  }
  else
  {
    if(SetSearchStartState(goal_stateID, pSearchStateSpace_) != 1)
    {
      SBPL_ERROR("ERROR: failed to set search start state\n");
      return 0;
    }
  }

  //ROS_ERROR("done setting goal");
  return 1;
}


int AnytimeEGraphPlanner::set_start(int start_stateID)
{

  SBPL_PRINTF("planner: setting start to %d\n", start_stateID);
  environment_->PrintState(start_stateID, true, stdout);

  if(bforwardsearch)
  {	

    if(SetSearchStartState(start_stateID, pSearchStateSpace_) != 1)
    {
      SBPL_ERROR("ERROR: failed to set search start state\n");
      return 0;
    }
  }
  else
  {
    if(SetSearchGoalState(start_stateID, pSearchStateSpace_) != 1)
    {
      SBPL_ERROR("ERROR: failed to set search goal state\n");
      return 0;
    }
  }

  return 1;

}



void AnytimeEGraphPlanner::costs_changed(StateChangeQuery const & stateChange)
{


  pSearchStateSpace_->bReinitializeSearchStateSpace = true;


}

void AnytimeEGraphPlanner::costs_changed()
{

  pSearchStateSpace_->bReinitializeSearchStateSpace = true;

}



int AnytimeEGraphPlanner::force_planning_from_scratch()
{
  SBPL_PRINTF("planner: forceplanfromscratch set\n");

  pSearchStateSpace_->bReinitializeSearchStateSpace = true;

  return 1;
}

int AnytimeEGraphPlanner::force_planning_from_scratch_and_free_memory()
{
  SBPL_PRINTF("planner: forceplanfromscratch set\n");
  int start_id = -1;
  int goal_id = -1;
  if(pSearchStateSpace_->searchstartstate)
    start_id = pSearchStateSpace_->searchstartstate->StateID;
  if(pSearchStateSpace_->searchgoalstate)
    goal_id = pSearchStateSpace_->searchgoalstate->StateID;

  if(!bforwardsearch){
    int temp = start_id;
    start_id = goal_id;
    goal_id = temp;
  }

  DeleteSearchStateSpace(pSearchStateSpace_);
  CreateSearchStateSpace(pSearchStateSpace_);
  InitializeSearchStateSpace(pSearchStateSpace_);
  for(unsigned int i=0; i<environment_->StateID2IndexMapping.size(); i++)
    for(int j=0; j<NUMOFINDICES_STATEID2IND; j++)
      environment_->StateID2IndexMapping[i][j] = -1;

  if(start_id>=0)
    set_start(start_id);
  if(goal_id>=0)
    set_goal(goal_id);
  return 1;
}


int AnytimeEGraphPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{

  SBPL_PRINTF("planner: search mode set to %d\n", bSearchUntilFirstSolution);

  bsearchuntilfirstsolution = bSearchUntilFirstSolution;

  return 1;
}


void AnytimeEGraphPlanner::print_searchpath(FILE* fOut)
{
  PrintSearchPath(pSearchStateSpace_, fOut);
}

void AnytimeEGraphPlanner::collisionCheck(){
  for(unsigned int i=0; i<egraph_->id2vertex.size(); i++){
    EGraph::EGraphVertex* v = egraph_->id2vertex[i];
    vector<double> coord;
    egraph_->discToCont(v,coord);
    for(unsigned int j=0; j<v->neighbors.size(); j++){
      EGraph::EGraphVertex* u = v->neighbors[j];
      if(v->id<u->id){
        vector<double> coord2;
        egraph_->discToCont(u,coord2);
        int cost = v->costs[j];
        bool valid = egraph_env_->isValidEdge(coord,coord2,cost);
        egraph_->updateEdge(v,u,valid,cost);
      }
    }
  }
  egraph_->computeComponents();
}


//---------------------------------------------------------------------------------------------------------


void AnytimeEGraphPlanner::get_search_stats(vector<EGraphPlannerStats>* s){
  s->clear();
  s->reserve(stats.size());
  for(unsigned int i=0; i<stats.size(); i++){
    s->push_back(stats[i]);
  }
}

