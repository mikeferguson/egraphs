#include<egraph_vis/egraph_visualizer.h>

EGraphVisualizer::EGraphVisualizer(EGraph* eg, EGraphMarkerMaker* converter){
  eg_ = eg;
  converter_ = converter;

  //set up interactive marker server
  server_.reset(new interactive_markers::InteractiveMarkerServer("EGraph","",true));
  ros::Duration(0.1).sleep();
  menu_handler_.insert("Show/Hide Neighborhood", boost::bind(&EGraphVisualizer::processFeedback, this, _1));
  menu_handler_.insert("Show/Hide Shortcuts", boost::bind(&EGraphVisualizer::processFeedback, this, _1));
}

EGraphVisualizer::~EGraphVisualizer(){
  server_.reset();
}

void EGraphVisualizer::visualize(){
  server_->clear();
  for(unsigned int i=0; i<eg_->id2vertex.size(); i++){
    EGraph::EGraphVertex* v = eg_->id2vertex[i];
    vector<double> coord;
    eg_->discToCont(v->coord,coord);
    visualization_msgs::Marker m = converter_->stateToVisualizationMarker(coord);
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = m.header.frame_id;
    int_marker.pose = m.pose;
    int_marker.scale = 1;
    int_marker.description = "";
    int_marker.name = (string("egraph_vertex_") + boost::lexical_cast<string>(i)).c_str();

    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    control.name = (string("egraph_menu_control_") + boost::lexical_cast<string>(i)).c_str();

    control.markers.push_back(m);
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server_->insert(int_marker);
    server_->setCallback(int_marker.name, boost::bind(&EGraphVisualizer::processFeedback, this, _1));
    menu_handler_.apply(*server_, int_marker.name);

    for(unsigned int j=0; j<v->neighbors.size(); j++){
      EGraph::EGraphVertex* u = v->neighbors[j];
      if(v->id<u->id){
        vector<double> coord2;
        eg_->discToCont(u->coord,coord2);
        visualization_msgs::Marker m = converter_->edgeToVisualizationMarker(coord,coord2);

        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = m.header.frame_id;
        int_marker.pose = m.pose;
        int_marker.scale = 1;
        int_marker.description = "";
        int_marker.name = (string("egraph_edge_") + boost::lexical_cast<string>(i) + string("_") + boost::lexical_cast<string>(j)).c_str();

        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

        control.markers.push_back(m);
        control.always_visible = true;
        int_marker.controls.push_back(control);

        server_->insert(int_marker);
      }
    }
  }
  server_->applyChanges();
}

void EGraphVisualizer::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  //parse the marker name
  char name[128];
  strncpy(name,feedback->marker_name.c_str(),sizeof(name));
  char* pch = strtok(name,"_");
  if(!pch || strcmp(pch,"egraph")!=0)
    return;
  //ok, this is an egraph marker...
  
  pch = strtok (NULL, "_");
  if(!pch || strcmp(pch,"vertex")!=0)
    return;
  //and it's a vertex...

  pch = strtok (NULL, "_");
  if(!pch)
    return;
  int id = atoi(pch);
  //we got the vertex id

  //if there are any more tokens, then it must be a "detailed" tag
  pch = strtok (NULL, "_");
  bool isDetailed = pch;

  //TODO: in a thread safe manner, check the version number of the egraph and compare
  //it to the version number from when we last visualized. make sure they are the same
  //so that we know that this id is still valid!
  EGraph::EGraphVertex* v = eg_->id2vertex[id];
  //we got the vertex from the e-graph
  
  
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT){
    if(feedback->menu_entry_id == 1){
      //Show/Hide Neighborhood

    }
    else if(feedback->menu_entry_id == 2){
      //Show/Hide Shortcuts

    }
  }
  else if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN){
    //Draw the detailed version of this vertex (this has the same menu controls as the regular vertex)

    if(isDetailed){
      //if this is a detailed vertex, delete it from the server
      
    }
    else{
      //if this is a regular vertex and we haven't already drawn the detailed one, add it to the server
      
    }
  }
  server_->applyChanges();
}

