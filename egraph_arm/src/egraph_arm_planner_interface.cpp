/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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
/** \author Mike Phillips, Benjamin Cohen */

#include <egraph_arm/egraph_arm_planner_interface.h>
#include <visualization_msgs/Marker.h>

clock_t starttime;

using namespace sbpl_arm_planner;

EGraphSBPLArmPlannerInterface::EGraphSBPLArmPlannerInterface(RobotModel *rm, CollisionChecker *cc, ActionSet* as, distance_field::PropagationDistanceField* df) : 
   SBPLArmPlannerInterface(rm, cc, as, df), eplanner_(NULL), egraph_arm_env_(NULL), replan_params_(0.0)
{
}

EGraphSBPLArmPlannerInterface::~EGraphSBPLArmPlannerInterface()
{
  if(heur_ != NULL)
    delete heur_;
  if(egraph_vis_ != NULL)
    delete egraph_vis_;
}

bool EGraphSBPLArmPlannerInterface::initializePlannerAndEnvironment(std::string ns)
{
  prm_ = new sbpl_arm_planner::PlanningParams();
  if(!prm_->init(ns))
    return false;

  grid_ = new sbpl_arm_planner::OccupancyGrid(df_);
  egraph_arm_env_ = new sbpl_arm_planner::EGraphArm(grid_, rm_, cc_, as_, prm_);  
  sbpl_arm_env_ = egraph_arm_env_;

  if(!sbpl_arm_env_)
    return false;

  if(!as_->init(sbpl_arm_env_))
  {
    ROS_ERROR("Failed to initialize the action set.");
    return false;
  } 
  //as_->print();

  //initialize arm planner environment
  if(!sbpl_arm_env_->initEnvironment())
  {
    ROS_ERROR("ERROR: initEnvironment failed");
    return false;
  }

  //initialize MDP 
  if(!sbpl_arm_env_->InitializeMDPCfg(&mdp_cfg_))
  {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
    return false;
  }

  string egraph_filename;
  nh_.param<string>("egraph_filename", egraph_filename, "");
  if(egraph_filename.empty())
    egraph_ = new EGraph(egraph_arm_env_,7, 0);
  else
    egraph_ = new EGraph(egraph_arm_env_,egraph_filename);

  int dimX,dimY,dimZ;
  grid_->getGridSize(dimX, dimY, dimZ);
  heur_ = new EGraph3dGridHeuristic(*egraph_arm_env_,dimX,dimY,dimZ,prm_->cost_per_cell_);
  egraph_mgr_ = new EGraphManager<vector<int> >(egraph_, egraph_arm_env_, heur_);
  eplanner_ = new LazyAEGPlanner<vector<int> >(egraph_arm_env_, true, egraph_mgr_);
  planner_ = eplanner_;
  egraph_vis_ = new EGraphVisualizer(egraph_, egraph_arm_env_);
  
  ROS_INFO("Initialized egraph sbpl arm planning environment.");
  return true;
}

bool EGraphSBPLArmPlannerInterface::solve(const arm_navigation_msgs::PlanningSceneConstPtr& planning_scene,
                                    EGraphReplanParams params,
                                    const arm_navigation_msgs::GetMotionPlan::Request &req,
                                    arm_navigation_msgs::GetMotionPlan::Response &res) 
{
  replan_params_ = params;
  return SBPLArmPlannerInterface::solve(planning_scene, req, res);
}

bool EGraphSBPLArmPlannerInterface::solve(EGraphReplanParams params,
                                    const arm_navigation_msgs::GetMotionPlan::Request &req,
                                    arm_navigation_msgs::GetMotionPlan::Response &res) 
{
  replan_params_ = params;
  return SBPLArmPlannerInterface::solve(req, res);
}

bool EGraphSBPLArmPlannerInterface::solve(const arm_navigation_msgs::GetMotionPlan::Request &req,
                                          arm_navigation_msgs::GetMotionPlan::Response &res) 
{
  replan_params_.max_time = req.motion_plan_request.allowed_planning_time.toSec();
  return SBPLArmPlannerInterface::solve(req, res);
}

bool EGraphSBPLArmPlannerInterface::plan(trajectory_msgs::JointTrajectory &traj)
{
  bool b_ret = false;
  std::vector<int> solution_state_ids;

  //reinitialize the search space
  planner_->force_planning_from_scratch();

  //plan
  b_ret = eplanner_->replan(&solution_state_ids, replan_params_, &solution_cost_);

  //check if an empty plan was received.
  if(b_ret && solution_state_ids.size() <= 0)
  {
    ROS_WARN("Path returned by the planner is empty?");
    b_ret = false;
  }

  // if a path is returned, then pack it into msg form
  if(b_ret && (solution_state_ids.size() > 0))
  {
    egraph_stat_map_ = eplanner_->getStats();
    
    if(!egraph_arm_env_->convertStateIDPathToJointTrajectory(solution_state_ids, traj))
      return false;
  }
  return b_ret;
}

void EGraphSBPLArmPlannerInterface::clearEGraph()
{
  egraph_->clearEGraph();
}

void EGraphSBPLArmPlannerInterface::validateEGraph()
{
  egraph_mgr_->validateEGraph(true);
}

bool EGraphSBPLArmPlannerInterface::loadEGraphFromFile(std::string filename)
{
  return egraph_->load(filename);
}

bool EGraphSBPLArmPlannerInterface::saveEGraphToFile(std::string filename)
{
  return egraph_->save(filename);
}

visualization_msgs::MarkerArray EGraphSBPLArmPlannerInterface::getVisualization(std::string type)
{
  if(type.compare("egraph") == 0)
    return egraph_vis_->getVisualization("egraph");
  else
    return SBPLArmPlannerInterface::getVisualization(type);
}

void EGraphSBPLArmPlannerInterface::setPlanningScene(const arm_navigation_msgs::PlanningSceneConstPtr& planning_scene)
{
  cc_->setPlanningScene(*planning_scene);
  egraph_mgr_->validateEGraph(true);
}

bool EGraphSBPLArmPlannerInterface::addDemonstration(egraph_arm::AddPathsToEGraph::Request& req, egraph_arm::AddPathsToEGraph::Response& res)
{
  if(req.clearEGraph)
    egraph_->clearEGraph();
  if(!req.paths.empty()){
    vector<vector<vector<double> > > paths;
    vector<vector<int> > costs;
    paths.resize(req.paths.size());
    costs.resize(req.paths.size());
    for(unsigned int i=0; i<req.paths.size(); i++){
      paths[i].resize(req.paths[i].points.size());
      costs[i].resize(req.paths[i].points.size()-1);
      for(unsigned int j=0; j<req.paths[i].points.size(); j++){
        paths[i][j].resize(req.paths[i].points[j].positions.size());
        int cost = 0;
        for(unsigned int k=0; k<req.paths[i].points[j].positions.size(); k++){
          paths[i][j][k] = req.paths[i].points[j].positions[k];
          if(j>0){
            double dang = fabs(angles::shortest_angular_distance(paths[i][j][k],paths[i][j-1][k]));
            cost += dang/(4*M_PI/180.0) * prm_->cost_multiplier_;
          }
        }
        if(j>0)
          costs[i][j-1] = cost;
      }
    }
    egraph_->addPaths(paths,costs);
  }
  return true;
}

