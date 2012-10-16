#include<egraphs/egraph.h>

EGraph::EGraph(vector<double>& min, vector<double>& max, vector<double>& resolution, vector<string>& names){
  min_ = min;
  max_ = max;
  res_ = resolution;
  names_ = names;

  //TODO: This is an arbitrary size...
  hashtable.resize(32*1024);
}

EGraph::EGraph(string filename){
  load(filename);
}

bool EGraph::addPath(vector<vector<double> >& coords, vector<int>& costs){
  //error checking
  if(coords.size() < 2){
    ROS_WARN("[EGraph] Less than 2 points in the added path...doing nothing.");
    return true;
  }
  if(costs.size() != coords.size()-1){
    ROS_ERROR("[EGraph] When giving a path to the E-Graph, it should have one less cost than it has coordinates");
    return false;
  }
  /*
  for(unsigned int i=0; i<names_.size(); i++){
    if(names_[i].compare(names[i]) != 0){
      ROS_ERROR("[EGraph] The given path has a different coordinate format than how this E-Graph was initialized!");
      return false;
    }
  }
  */
  
  //convert continuous coordinates to discrete ones
  vector<vector<int> > disc_coords;
  for(unsigned int i=0; i<coords.size(); i++){
    if(res_.size() != coords[i].size()){
      ROS_ERROR("[EGraph] There is a coordinate in the path that doesn't have enough fields!");
      return false;
    }
    vector<int> dc;
    contToDisc(coords[i], dc);
    disc_coords.push_back(dc);
  }

  //if a vertex is not in the graph add it
  vector<EGraphVertex*> path_vertices;
  for(unsigned int i=0; i<disc_coords.size(); i++){
    EGraphVertex* v = getVertex(disc_coords[i]);
    if(!v)
      v = createVertex(disc_coords[i]);
    path_vertices.push_back(v);
  }

  //add each edge
  for(unsigned int i=1; i<path_vertices.size(); i++)
    addEdge(path_vertices[i-1],path_vertices[i],costs[i-1]);

  ROS_INFO("[EGraph] addPath complete. EGraph now contains %d vertices",id2vertex.size());
  return true;
}

bool EGraph::save(string filename){
  return false;
}

bool EGraph::load(string filename){
  return false;
}

void EGraph::collisionCheck(){

}

unsigned int EGraph::inthash(unsigned int key){
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

int EGraph::getHashBin(vector<int>& coord){
  int hash = 0;
  for(unsigned int i=0; i<coord.size(); i++){
    hash += inthash(coord[i])<<i;
  }
  return inthash(hash) & (hashtable.size()-1);
}

EGraph::EGraphVertex* EGraph::getVertex(vector<int>& coord){
  int idx = getHashBin(coord);
  for(unsigned int i=0; i<hashtable[idx].size(); i++){
    bool isEqual = true;
    for(unsigned int j=0; j<hashtable[idx][i]->coord.size(); j++){
      if(coord[j]!=hashtable[idx][i]->coord[j]){
        isEqual = false;
        break;
      }
    }
    if(isEqual)
      return hashtable[idx][i];
  }
  return NULL;
}

EGraph::EGraphVertex* EGraph::createVertex(vector<int>& coord){
  EGraphVertex* v = new EGraphVertex();
  v->coord = coord;
  v->id = id2vertex.size();
  id2vertex.push_back(v);
  int idx = getHashBin(coord);
  hashtable[idx].push_back(v);
  return v;
}

void EGraph::addEdge(EGraphVertex* v1, EGraphVertex* v2, int cost){
  bool done = false;
  for(unsigned int i=0; i<v1->neighbors.size(); i++){
    if(v1->neighbors[i]==v2){
      if(cost < v1->costs[i]){
        v1->costs[i] = cost;
        ROS_WARN("[EGraph] This edge already exists, but the new one is cheaper. Overwriting...");
      }
      done = true;
      break;
    }
  }
  if(!done){
    v1->neighbors.push_back(v2);
    v1->costs.push_back(cost);
  }

  done = false;
  for(unsigned int i=0; i<v2->neighbors.size(); i++){
    if(v2->neighbors[i]==v1){
      if(cost < v2->costs[i]){
        v2->costs[i] = cost;
        ROS_WARN("[EGraph] This edge already exists, but the new one is cheaper. Overwriting...");
      }
      done = true;
      break;
    }
  }
  if(!done){
    v2->neighbors.push_back(v1);
    v2->costs.push_back(cost);
  }
}

void EGraph::discToCont(vector<int> d, vector<double>& c){
  c.clear();
  for(unsigned int i=0; i<d.size(); i++)
    c.push_back(d[i]*res_[i]+min_[i]);
}

void EGraph::contToDisc(vector<double> c, vector<int>& d){
  d.clear();
  for(unsigned int i=0; i<c.size(); i++)
    d.push_back(int((c[i]-min_[i])/(res_[i])));
}

