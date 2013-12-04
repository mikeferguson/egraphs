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
#ifndef __SIMPLE_ANYTIME_EGRAPH_PLANNER_H_
#define __SIMPLE_ANYTIME_EGRAPH_PLANNER_H_

#define AEGMDP_STATEID2IND STATEID2IND_SLOT0

#include<sbpl/headers.h>
#include<egraphs/egraph.h>
#include<egraphs/egraphable.h>
#include<egraphs/egraph_heuristic.h>

#include<boost/thread.hpp>

//---configuration----

//control of EPS
//initial suboptimality bound (cost solution <= cost(eps*cost optimal solution)
#define AEG_DEFAULT_INITIAL_EPS	    5.0
//as planning time exist, AEG* decreases epsilon bound
#define AEG_DECREASE_EPS    0.2
//final epsilon bound
#define AEG_FINAL_EPS	    1.0


//---------------------

#define AEG_INCONS_LIST_ID 0

class CMDP;
class CMDPSTATE;
class CMDPACTION;
class CHeap;
class CList;

class EGraphReplanParams : public ReplanParams{
  public:
    EGraphReplanParams(double time):ReplanParams(time) {
      epsE = 10.0;
      final_epsE = 1.0;
      dec_epsE = 1.0;
      feedback_path = true;
      use_egraph = true;
    };
    double epsE;
    double final_epsE;
    double dec_epsE;
    bool feedback_path;
    bool use_egraph;
    bool update_stats;
};

class EGraphPlannerStats : public PlannerStats{
  public:
    double bound;
    double epsE;
};

class EGraphTimingStats{
  public:
    EGraphTimingStats(){
      initial_heuristic_time = 0;
      heuristic_time = 0;
      direct_shortcut_time = 0;
      direct_shortcut_path_time = 0;
      gradient_shortcut_time = 0;
      gradient_shortcut_path_time = 0;
      errorCheckEGraphTime = 0;
      updateEGraphTime = 0;
    }

    double initial_heuristic_time;
    double heuristic_time;
    double direct_shortcut_time;
    double direct_shortcut_path_time;
    double gradient_shortcut_time;
    double gradient_shortcut_path_time;
    double errorCheckEGraphTime;
    double updateEGraphTime;
};

//-------------------------------------------------------------


/** \brief state structure used in AEG* search tree
*/
typedef class AEGSEARCHSTATEDATA : public AbstractSearchState
{
  public:
    /** \brief the MDP state itself
    */
    CMDPSTATE* MDPstate; 
    /** \brief AEG* relevant data
    */
    unsigned int v;
    /** \brief AEG* relevant data
    */
    unsigned int g;
    /** \brief AEG* relevant data
    */
    short unsigned int iterationclosed;
    /** \brief AEG* relevant data
    */
    short unsigned int callnumberaccessed;

#if DEBUG
    /** \brief AEG* relevant data
    */
    short unsigned int numofexpands;
#endif

    /** \brief best predecessor and the action from it, used only in forward searches
    */
    CMDPSTATE *bestpredstate;
    /** \brief the next state if executing best action
    */
    CMDPSTATE  *bestnextstate;
    unsigned int costtobestnextstate;
    int h;


  public:
    AEGSEARCHSTATEDATA() {};	
    ~AEGSEARCHSTATEDATA() {};
} AEGState;



/** \brief the statespace of AEG*
*/
typedef struct AEGSEARCHSTATESPACE
{
  double eps;
  double epsE;
  double eps_satisfied;
  CHeap* heap;
  CList* inconslist;
  short unsigned int searchiteration;
  short unsigned int callnumber;
  CMDPSTATE* searchgoalstate;
  CMDPSTATE* searchstartstate;

  CMDP searchMDP;

  bool bReevaluatefvals;
  bool bReinitializeSearchStateSpace;
  bool bNewSearchIteration;

} AEGSearchStateSpace_t;



/** \brief AEG* planner
*/
class AnytimeEGraphPlanner : public SBPLPlanner
{

  public:

    void initializeEGraph(EGraph* egraph, EGraphable* egraph_env, EGraphHeuristic* egraph_heur);

    /** \brief replan a path within the allocated time, return the solution in the vector
    */
    virtual int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);
    /** \brief replan a path within the allocated time, return the solution in the vector, also returns solution cost
    */
    virtual int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost);

    /** \brief works same as replan function with time and solution states, but it let's you fill out all the parameters for the search
    */
    virtual int replan(std::vector<int>* solution_stateIDs_V, EGraphReplanParams params);

    /** \brief works same as replan function with time, solution states, and cost, but it let's you fill out all the parameters for the search
    */
    virtual int replan(std::vector<int>* solution_stateIDs_V, EGraphReplanParams params, int* solcost);


    /** \brief set the goal state
    */
    virtual int set_goal(int goal_stateID);
    /** \brief set the start state
    */
    virtual int set_start(int start_stateID);

    /** \brief inform the search about the new edge costs
    */
    virtual void costs_changed(StateChangeQuery const & stateChange);

    /** \brief inform the search about the new edge costs - 
      \note since AEG* is non-incremental, it is sufficient (and more efficient) to just inform AEG* of the fact that some costs changed
      */
    virtual void costs_changed();


    /** \brief set a flag to get rid of the previous search efforts, and re-initialize the search, when the next replan is called
    */
    virtual int force_planning_from_scratch(); 

    /** \brief Gets rid of the previous search efforts, release the memory and re-initialize the search. 
    */
    virtual int force_planning_from_scratch_and_free_memory();

    /** \brief you can either search forwards or backwards
    */
    virtual int set_search_mode(bool bSearchUntilFirstSolution);

    void useEGraph(bool use){useEGraph_ = use;};

    /** \brief returns the suboptimality bound on the currently found solution
    */
    virtual double get_solution_eps() const {return pSearchStateSpace_->eps_satisfied;};

    /** \brief returns the number of states expanded so far
    */
    virtual int get_n_expands() const { return searchexpands; }

    /** \brief sets the value of the initial epsilon (suboptimality bound) used
    */
    virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps;};

    /** \brief prints out the search path into a file
    */
    virtual void print_searchpath(FILE* fOut);


    /** \brief constructor 
    */
    AnytimeEGraphPlanner(DiscreteSpaceInformation* environment, bool bforwardsearch);
    /** \brief destructor
    */
    ~AnytimeEGraphPlanner();

    /** \brief returns the initial epsilon
    */
    virtual double get_initial_eps(){return finitial_eps;};

    /** \brief returns the time taken to find the first solution
    */
    virtual double get_initial_eps_planning_time(){return finitial_eps_planning_time;}

    /** \brief returns the time taken to get the final solution
    */
    virtual double get_final_eps_planning_time(){return final_eps_planning_time;};

    /** \brief returns the number of expands to find the first solution
    */
    virtual int get_n_expands_init_solution(){return num_of_expands_initial_solution;};

    /** \brief returns the final epsilon achieved during the search
    */
    virtual double get_final_epsilon(){return final_eps;};

    /** \brief fills out a vector of stats from the search
    */
    virtual void get_search_stats(vector<EGraphPlannerStats>* s);

    virtual EGraphTimingStats get_timing_stats(){return timing_stats;};


    void collisionCheck();

  protected:

    //member variables
    double finitial_eps, finitial_eps_planning_time, final_eps_planning_time, final_eps, dec_eps, final_epsilon;
    double initial_epsE, final_epsE, dec_epsE;
    double repair_time;
    bool use_repair_time;
    bool feedback_path;
    bool update_stats;

    vector<EGraphPlannerStats> stats;
    EGraphTimingStats timing_stats;

    int num_of_expands_initial_solution;

    DiscreteSpaceInformation* environment_;
    EGraph* egraph_;
    EGraphable* egraph_env_;
    EGraphHeuristic* egraph_heur_;
    bool useEGraph_;
    vector<vector<EGraph::EGraphVertex*> > directShortcutCache_;

    bool planner_ok_;
    boost::thread* egraph_thread_;
    boost::mutex egraph_mutex_;
    boost::condition_variable egraph_cond_;

    vector<vector<double> > egraph_path_;
    vector<int> egraph_path_costs_;
    MDPConfig* MDPCfg_;

    bool bforwardsearch; //if true, then search proceeds forward, otherwise backward

    bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

    AEGSearchStateSpace_t* pSearchStateSpace_;

    unsigned int searchexpands;
    int MaxMemoryCounter;
    clock_t TimeStarted;
    FILE *fDeb;


    //member functions
    void errorCheckEGraph(EGraph::EGraphVertex* egv);

    virtual void Initialize_searchinfo(CMDPSTATE* state, AEGSearchStateSpace_t* pSearchStateSpace);

    virtual CMDPSTATE* CreateState(int stateID, AEGSearchStateSpace_t* pSearchStateSpace);

    virtual CMDPSTATE* GetState(int stateID, AEGSearchStateSpace_t* pSearchStateSpace);

    inline int getHeuristic(vector<double>& coord);
    virtual int ComputeHeuristic(CMDPSTATE* MDPstate, AEGSearchStateSpace_t* pSearchStateSpace);

    //initialization of a state
    virtual void InitializeSearchStateInfo(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace);

    //re-initialization of a state
    virtual void ReInitializeSearchStateInfo(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace);

    virtual void DeleteSearchStateData(AEGState* state);

    //used for backward search
    virtual void UpdatePreds(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace);


    //used for forward search
    virtual void UpdateSuccs(AEGState* state, AEGSearchStateSpace_t* pSearchStateSpace);

    virtual void getDirectShortcutSuccessors(int stateID, vector<int>& SuccIDV, vector<int>& CostV);
    virtual void getGradientShortcutSuccessors(int stateID, vector<int>& SuccIDV, vector<int>& CostV);
    virtual void getSnapSuccessors(int stateID, vector<int>& SuccIDV, vector<int>& CostV);
    virtual void getDirectShortcutPath(int fromID, int toID, vector<int>& ids);
    virtual void getGradientShortcutPath(int fromID, int toID, int cost, vector<int>& ids);
    virtual void updateEGraph();

    virtual int GetGVal(int StateID, AEGSearchStateSpace_t* pSearchStateSpace);

    //returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
    virtual int ImprovePath(AEGSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);

    virtual void BuildNewOPENList(AEGSearchStateSpace_t* pSearchStateSpace);

    virtual void Reevaluatefvals(AEGSearchStateSpace_t* pSearchStateSpace);

    //creates (allocates memory) search state space
    //does not initialize search statespace
    virtual int CreateSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace);

    //deallocates memory used by SearchStateSpace
    virtual void DeleteSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace);

    //debugging 
    virtual void PrintSearchState(AEGState* state, FILE* fOut);


    //reset properly search state space
    //needs to be done before deleting states
    virtual int ResetSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace);

    //initialization before each search
    virtual void ReInitializeSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace);

    //very first initialization
    virtual int InitializeSearchStateSpace(AEGSearchStateSpace_t* pSearchStateSpace);

    virtual int SetSearchGoalState(int SearchGoalStateID, AEGSearchStateSpace_t* pSearchStateSpace);


    virtual int SetSearchStartState(int SearchStartStateID, AEGSearchStateSpace_t* pSearchStateSpace);

    //reconstruct path functions are only relevant for forward search
    virtual int ReconstructPath(AEGSearchStateSpace_t* pSearchStateSpace);


    virtual void PrintSearchPath(AEGSearchStateSpace_t* pSearchStateSpace, FILE* fOut);

    virtual int getHeurValue(AEGSearchStateSpace_t* pSearchStateSpace, int StateID);

    //get path 
    virtual vector<int> GetSearchPath(AEGSearchStateSpace_t* pSearchStateSpace, int& solcost);


    virtual bool Search(AEGSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs);


};


#endif



