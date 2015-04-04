#include <egraph_arm/egraph_arm.h>
#include <assert.h>
#include <angles/angles.h>

using namespace std;
namespace sbpl_arm_planner {

EGraphArm::EGraphArm(OccupancyGrid *grid, 
                                     RobotModel *rmodel, 
                                     CollisionChecker *cc, 
                                     ActionSet* as, PlanningParams *pm):
   EnvironmentROBARM3D(grid, rmodel, cc, as, pm) {
}

int EGraphArm::getGoalHeuristic(int state_id){
    return GetFromToHeuristic(state_id, pdata_.goal_entry->stateID);
}

//int EGraphArm::getGoalHeuristic(int x, int y, int z){
//    return bfs_->getDistance(x,y,z);
//}


bool EGraphArm::snap(const std::vector<double>& from, 
                     const std::vector<double>& to, int& id, int& cost){
    vector<int> from_coord(from.size(),0);
    vector<int> to_coord(to.size(),0);
    anglesToCoord(from, from_coord);
    anglesToCoord(to, to_coord);

    if (from_coord == to_coord){
        return false;
    }
    double dist = 0;
    id = getStateID(to);
    cost = 400;
    BodyPose bp;
    //pviz_.setReferenceFrame("/base_footprint");
    //pviz_.visualizeRobot(from, from, bp, .1, "blah_from", 150,  1);
    //pviz_.visualizeRobot(to, to, bp, .1, "blah_to",150, 2);
    //std::cin.get();
    
    if (!cc_->isStateValid(from, false, false, dist)){
        return false;
    }
    if (!cc_->isStateValid(to, false, false, dist)){
        return false;
    }
    int temp = 0;
    int temp2 = 0;
    if (!cc_->isStateToStateValid(from, to, temp, temp2, dist)){
        return false;
    } else {
        return true;
    }
}

bool EGraphArm::getCoord(int id, vector<double>& coord){
    if (id == pdata_.start_entry->stateID){
        coordToAngles(pdata_.start_entry->coord, coord);
        return true;
    }

    StateID2Angles(id, coord);
    for (size_t i=0; i < coord.size(); i++){
        coord[i] = angles::normalize_angle_positive(coord[i]);
    }
    return true;
}

// return the DISCRETE coordinate of the goal in xyz
void EGraphArm::projectGoalToHeuristicSpace(vector<int>& coord) const{
    coord.resize(3,0);
    coord[0] = (pdata_.goal_entry->xyz[0]);
    coord[1] = (pdata_.goal_entry->xyz[1]);
    coord[2] = (pdata_.goal_entry->xyz[2]);
    //ROS_INFO("project to goal %d %d %d\n\n",coord[0],coord[1],coord[2]);

}

bool EGraphArm::isGoal(int id){
    if (id == pdata_.goal_entry->stateID){
        return true;
    }
    vector<double> coord;
    StateID2Angles(id, coord);
    vector<double> state;
    rmodel_->computePlanningLinkFK(coord,state);

    return isGoalState(state, pdata_.goal);
}

int EGraphArm::getStateID(const std::vector<double>& angles){
    vector<int> coord(7,0);
    anglesToCoord(angles, coord);

    if (coord == pdata_.start_entry->coord){
        return pdata_.start_entry->stateID;
    }

    EnvROBARM3DHashEntry_t* succ_entry;

    // discretize planning link pose
    vector<double> pose(6,0);
    if(!rmodel_->computePlanningLinkFK(angles, pose)){
        ROS_ERROR("couldn't compute planning link for angles!");
        assert(false);
    }
    // discretize planning link pose
    int endeff[3]={0};
    grid_->worldToGrid(pose[0],pose[1],pose[2],endeff[0],endeff[1],endeff[2]);
    
    //check if this state meets the goal criteria
    if(isGoalState(pose, pdata_.goal))
    {

      for (int k = 0; k < prm_->num_joints_; k++)
        pdata_.goal_entry->coord[k] = coord[k];

      pdata_.goal_entry->xyz[0] = endeff[0];
      pdata_.goal_entry->xyz[1] = endeff[1];
      pdata_.goal_entry->xyz[2] = endeff[2];
      pdata_.goal_entry->state = angles;
      pdata_.goal_entry->dist = 0;
      //ROS_INFO("setting goal state");
      //printVector(angles);
      if ((succ_entry = getHashEntry(coord, false)) == NULL){
         succ_entry = createHashEntry(coord, endeff);
      }
    }

    if ((succ_entry = getHashEntry(coord, false)) == NULL){
        // get pose of planning link

        succ_entry = createHashEntry(coord, endeff);
        succ_entry->state = angles;
        succ_entry->dist = 0;
        //ROS_ERROR("can't retrieve state id");
        //for (int i=0; i < 7; i++){
        //    printf("%d ", coord[i]);
        //}
        //printf("\n");
        //assert(false);
    }
    return succ_entry->stateID;
}

bool EGraphArm::isValidVertex(const vector<double>& coord){
    double dist;
    return cc_->isStateValid(coord, false, false, dist);
}

bool EGraphArm::isValidEdge(const vector<double>& start, const vector<double>& end, bool& changed_cost, int& cost){
    double dist;
 
        BodyPose bp;
        bp.z=.1;
    bool is_valid_start = cc_->isStateValid(start, false, false, dist);
    bool is_valid_end = cc_->isStateValid(end, false, false, dist);
    //if (!is_valid_start){
    //    visualization_msgs::MarkerArray ma = cc_->getCollisionModelVisualization(start);
    //    pviz_.publishMarkerArray(ma);
    //    ma = cc_->getVisualization("collisions");
    //    pviz_.publishMarkerArray(ma);
    //    ROS_INFO("invalid start");
    //    pviz_.visualizeRobot(start, start, bp, .1, "blah_from", 150,  1);
    //    std::cin.get();
    //}
    //if (!is_valid_end){
    //    ROS_INFO("invalid end");
    //    visualization_msgs::MarkerArray ma = cc_->getCollisionModelVisualization(end);
    //    pviz_.publishMarkerArray(ma);
    //    ma = cc_->getVisualization("collisions");
    //    pviz_.publishMarkerArray(ma);
    //    pviz_.visualizeRobot(end, end, bp, .1, "blah_from", 150,  1);
    //    std::cin.get();
    //}

    // something's not right here. if i use this, things go invalid when they
    // shouldn't.....
    int temp = 0;
    int temp2 = 0;
    bool is_valid = cc_->isStateToStateValid(start, end, temp, temp2, dist);
    if (is_valid_start && is_valid_end && is_valid){
        // sbpl secret sauce (TM)
        cost = prm_->cost_multiplier_;
    } else {
        //ROS_INFO("isvalid: %d %d %d", is_valid_start, is_valid_end, is_valid);
    }
    return is_valid_start && is_valid_end;
}
/*
void EGraphArm::displayPath(const DisplayEdges& edges){
    pviz_.deleteVisualizations("lazy_path", 200);
    pviz_.setReferenceFrame("/base_footprint");
    vector<geometry_msgs::Point> viz_points;
    bool segment = edges[0].second;
    int counter = 0;
    for (auto& edge : edges){
        auto from = edge.first.first;
        auto to = edge.first.second;
        auto validity = edge.second;

        if (segment != validity){

            if (segment){
                pviz_.visualizeLine(viz_points, "lazy_path", counter, 150, .02);
            } else {
                pviz_.visualizeLine(viz_points, "lazy_path", counter, 255, .02);
            }
            geometry_msgs::Point prev = viz_points.back();
            viz_points.clear();
            viz_points.push_back(prev);
            counter++;
            segment = validity;
        }
        vector<double> pose(6,0); 
        rmodel_->computePlanningLinkFK(from, pose);
        geometry_msgs::Point point;
        point.x = pose[0];
        point.y = pose[1];
        point.z = pose[2];
        viz_points.push_back(point);

    }
    if (segment){
        pviz_.visualizeLine(viz_points, "lazy_path", counter, 150, .02);
    } else {
        pviz_.visualizeLine(viz_points, "lazy_path", counter, 50, .02);
    }
}
*/

void EGraphArm::projectToHeuristicSpace(const std::vector<double>& coord, std::vector<int>& dp) const{
    vector<double> state;
    vector<int> endeff(3,0);
    rmodel_->computePlanningLinkFK(coord,state);
    grid_->worldToGrid(state[0],state[1],state[2],endeff[0],endeff[1],endeff[2]);
    vector<double> zeros(3,0);
    //pviz_.setReferenceFrame("/base_footprint");
    //pviz_.visualizeRobot(coord, coord, zeros, 0.1, 150, "expands", 1);
    //pviz_.visualizePose(state, "heuristic_pose");
    dp.clear();
    dp.push_back(endeff[0]);
    dp.push_back(endeff[1]);
    dp.push_back(endeff[2]);
}

void EGraphArm::discToCont(const vector<int>& d, vector<double>& c){
  c.resize(d.size());
  for(unsigned int i=0; i<d.size(); i++)
    c[i] = d[i]*prm_->coord_delta_[i];
}

void EGraphArm::contToDisc(const vector<double>& c, vector<int>& d){
  d.resize(c.size());
  for(unsigned int i=0; i<c.size(); i++)
    d[i] = int( c[i]/prm_->coord_delta_[i] + 0.5 );
}

void EGraphArm::printVector(vector<int> v){
    for (size_t i=0; i < v.size(); i++){
        printf("%d ", v[i]);
    }
    printf("\n");
}

void EGraphArm::printVector(vector<double> v){
    for (size_t i=0; i < v.size(); i++){
        printf("%f ", v[i]);
    }
    printf("\n");
}

visualization_msgs::MarkerArray EGraphArm::stateToVisualizationMarker(vector<double> coord){
    visualization_msgs::Marker marker;
    marker.header.frame_id = rmodel_->getPlanningFrame(); //"/map";
    marker.type = visualization_msgs::Marker::SPHERE;

    vector<double> state;
    rmodel_->computePlanningLinkFK(coord,state);

    marker.pose.position.x = state[0];
    marker.pose.position.y = state[1];
    marker.pose.position.z = state[2];
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);
    return ma;
}

visualization_msgs::MarkerArray EGraphArm::stateToDetailedVisualizationMarker(vector<double> coord){
  ROS_ERROR("Not implemented.");
  return visualization_msgs::MarkerArray();
    /*
    vector<double> l_arm(7,0);
    BodyPose pose;
    pose.z = 0.0;
    visualization_msgs::MarkerArray ma = pviz_.getRobotMarkerMsg(coord, l_arm, pose, 250, "detailed state", 0);
    for(unsigned int i=0; i<ma.markers.size(); i++)
        ma.markers[i].header.frame_id = "/map";
    return ma;
    */
}


visualization_msgs::MarkerArray EGraphArm::edgeToVisualizationMarker(vector<double> coord, vector<double> coord2){
    visualization_msgs::Marker marker;
    marker.header.frame_id = rmodel_->getPlanningFrame(); //"/map";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.01;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    geometry_msgs::Point p;

    vector<double> state;
    rmodel_->computePlanningLinkFK(coord,state);


    p.x = state[0];
    p.y = state[1];
    p.z = state[2];
    marker.points.push_back(p);

    rmodel_->computePlanningLinkFK(coord2,state);

    p.x = state[0];
    p.y = state[1];
    p.z = state[2];
    marker.points.push_back(p);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);
    return ma;
}
}

