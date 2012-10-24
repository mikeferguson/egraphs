#include<egraph_vis/egraph_visualizer.h>

EGraphVisualizer::EGraphVisualizer(EGraph* eg, EGraphMarkerMaker* converter){
  marker_pub_ = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("EGraph",1);
  eg_ = eg;
  converter_ = converter;
}

void EGraphVisualizer::visualize(){
  visualization_msgs::MarkerArray ma;
  int edge_count = 0;
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    EGraph::EGraphVertex* v = eg_->id2vertex[i];
    vector<double> coord;
    eg_->discToCont(v->coord,coord);
    visualization_msgs::Marker m = converter_->stateToVisualizationMarker(coord);
    m.ns = "vertices";
    m.id = i;
    m.header.stamp = ros::Time::now();
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration();
    ma.markers.push_back(m);

    for(unsigned int j=0; j<v->neighbors.size(); j++){
      EGraph::EGraphVertex* u = v->neighbors[j];
      if(v->id<u->id){
        vector<double> coord2;
        eg_->discToCont(u->coord,coord2);
        visualization_msgs::Marker m = converter_->edgeToVisualizationMarker(coord,coord2);
        m.ns = "edges";
        m.id = edge_count;
        m.header.stamp = ros::Time::now();
        m.action = visualization_msgs::Marker::ADD;
        m.lifetime = ros::Duration();
        edge_count++;
        ma.markers.push_back(m);
      }
    }
  }
  marker_pub_.publish(ma);
}

