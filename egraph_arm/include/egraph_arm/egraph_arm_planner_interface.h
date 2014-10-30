/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef _EGRAPH_ARM_PLANNER_INTERFACE_H_
#define _EGRAPH_ARM_PLANNER_INTERFACE_H_

#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <egraph_arm/egraph_arm.h>
#include <egraph_arm/AddPathsToEGraph.h>
#include <egraphs/egraph_planner.h>
#include <egraphs/egraph_3d_grid_heuristic.h>

namespace sbpl_arm_planner{

class EGraphSBPLArmPlannerInterface : public SBPLArmPlannerInterface
{
  public:
    
    EGraphSBPLArmPlannerInterface(RobotModel *rmodel, CollisionChecker *cc, ActionSet* as, distance_field::PropagationDistanceField* df);

    ~EGraphSBPLArmPlannerInterface();
    
    bool solve(const arm_navigation_msgs::PlanningSceneConstPtr& planning_scene, EGraphReplanParams params, const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res);
    
    bool solve(EGraphReplanParams params, const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res);

    bool solve(const arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res);
    
    void interrupt(){eplanner_->interrupt();};

    map<string,double> getEGraphStats(){return egraph_stat_map_;};

    void clearEGraph();

    void validateEGraph();

    bool addDemonstration(egraph_arm::AddPathsToEGraph::Request& req, egraph_arm::AddPathsToEGraph::Response& res);

    bool loadEGraphFromFile(std::string filename);

    bool saveEGraphToFile(std::string filename);

    void setPlanningScene(const arm_navigation_msgs::PlanningSceneConstPtr& planning_scene);

    visualization_msgs::MarkerArray getVisualization(std::string type);

    void visualizeEGraphInteractiveMarkers(){egraph_vis_->visualize();};

  private:

    EGraph* egraph_;

    map<string,double> egraph_stat_map_;

    /* planner & environment */
    LazyAEGPlanner<vector<int> > *eplanner_;
    sbpl_arm_planner::EGraphArm *egraph_arm_env_;
    EGraph3dGridHeuristic* heur_;
    EGraphManager<vector<int> >* egraph_mgr_;
    EGraphVisualizer* egraph_vis_;
    EGraphReplanParams replan_params_;

    /** \brief Initialize the SBPL planner and the sbpl_arm_planner environment */
    bool initializePlannerAndEnvironment(std::string ns="~");

    /** \brief Retrieve plan from sbpl */
    bool plan(trajectory_msgs::JointTrajectory &traj);
};

}

#endif
