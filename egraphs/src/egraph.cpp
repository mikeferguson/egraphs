#include<egraphs/egraph.h>

//TODO: This is an arbitrary size...
#define ARBITRARY_HASH_TABLE_SIZE 32*1024

EGraph::EGraph(vector<double>& min, vector<double>& max, vector<double>& resolution, vector<string>& names, int num_constants){
  min_ = min;
  max_ = max;
  res_ = resolution;
  names_ = names;
  num_constants_ = num_constants;

  hashtable.resize(ARBITRARY_HASH_TABLE_SIZE);
}

EGraph::EGraph(string filename){
  hashtable.resize(ARBITRARY_HASH_TABLE_SIZE);
  load(filename);
}

EGraph::~EGraph(){
  clearEGraph();
}

void EGraph::clearEGraph(){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  min_.clear();
  max_.clear();
  res_.clear();
  names_.clear();
  hashtable.clear();
  hashtable.resize(ARBITRARY_HASH_TABLE_SIZE);

  for(unsigned int i=0; i<id2vertex.size(); i++)
    delete id2vertex[i];

  id2vertex.clear();
}

bool EGraph::addPath(vector<vector<double> >& coords, vector<int>& costs){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
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
  vector<vector<double> > constants;
  for(unsigned int i=0; i<coords.size(); i++){
    if(res_.size()+num_constants_ != coords[i].size()){
      ROS_ERROR("[EGraph] There is a coordinate in the path that doesn't have enough fields!");
      return false;
    }
    vector<int> dc;
    contToDisc(coords[i], dc);
    disc_coords.push_back(dc);
    vector<double> temp;
    for(unsigned int j=res_.size(); j<coords[i].size(); j++)
      temp.push_back(coords[i][j]);
    constants.push_back(temp);
  }

  //if a vertex is not in the graph add it
  vector<EGraphVertex*> path_vertices;
  for(unsigned int i=0; i<disc_coords.size(); i++){
    EGraphVertex* v = getVertex(disc_coords[i]);
    if(!v)
      v = createVertex(disc_coords[i],constants[i]);
    path_vertices.push_back(v);
  }

  //add each edge
  for(unsigned int i=1; i<path_vertices.size(); i++){
    ROS_INFO("adding %d->%d at a cost of %d",path_vertices[i-1]->id,path_vertices[i]->id,costs[i-1]);
    addEdge(path_vertices[i-1],path_vertices[i],costs[i-1]);
  }

  ROS_INFO("[EGraph] addPath complete. EGraph now contains %d vertices",int(id2vertex.size()));
  return true;
}

//print E-Graph
void EGraph::print(){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  for(unsigned int i=0; i<id2vertex.size(); i++){
    EGraphVertex* v = id2vertex[i];
    printf("id:%d coord:(",v->id);
    for(unsigned int j=0; j<v->coord.size(); j++)
      printf("%d,",v->coord[j]);
    printf(") constants:(");
    for(unsigned int j=0; j<v->constants.size(); j++)
      printf("%f,",v->constants[j]);
    printf(") neighbors:{");
    for(unsigned int j=0; j<v->neighbors.size(); j++)
      printf("(%d,%d),",v->neighbors[j]->id,v->costs[j]);
    printf("}\n");
  }
}

bool EGraph::save(string filename){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  FILE* fout = fopen(filename.c_str(),"w");
  if(!fout){
    ROS_ERROR("Could not open file \"%s\" to save E-Graph",filename.c_str());
    return false;
  }

  //save the dimension names, min, max, and resolution
  //for each dimension there is a line with format: "name min max resolution"
  fprintf(fout,"%d %d\n",int(names_.size()),num_constants_);
  for(unsigned int i=0; i<names_.size(); i++)
    fprintf(fout,"%s %f %f %f\n",names_[i].c_str(),min_[i],max_[i],res_[i]);

  //save the id2vertex table
  //one line for each vertex
  //"coord_values num_neighbors neighbor_ids costs_to_neighbors"
  fprintf(fout,"%d\n",int(id2vertex.size()));
  for(unsigned int i=0; i<id2vertex.size(); i++){
    EGraphVertex* v = id2vertex[i];
    vector<double> coord;
    discToCont(v,coord);
    for(unsigned int j=0; j<coord.size(); j++)
      fprintf(fout,"%f ",coord[j]);
    fprintf(fout,"%d ",int(v->neighbors.size()));
    for(unsigned int j=0; j<v->neighbors.size(); j++)
      fprintf(fout,"%d ",v->neighbors[j]->id);
    for(unsigned int j=0; j<v->costs.size(); j++)
      fprintf(fout,"%d ",v->costs[j]);
    fprintf(fout,"\n");
  }

  return true;
}

bool EGraph::load(string filename, bool clearCurrentEGraph){
  boost::recursive_mutex::scoped_lock lock(egraph_mutex_);
  if(clearCurrentEGraph)
    clearEGraph();
  else{
    ROS_ERROR("Loading an E-Graph without clearing the old one is not supported yet...");
    return false;
  }

  FILE* fin = fopen(filename.c_str(),"r");
  if(!fin){
    ROS_ERROR("Could not open file \"%s\" to load E-Graph",filename.c_str());
    return false;
  }

  //read the number of dimensions
  int num_dimensions;
  if(fscanf(fin,"%d", &num_dimensions) != 1){
    ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
    return false;
  }

  //read the number of constants
  if(fscanf(fin,"%d", &num_constants_) != 1){
    ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
    return false;
  }

  printf("read dimension details\n");
  //read in all the dimension details
  char name[128];
  double min,max,res;
  for(int i=0; i<num_dimensions; i++){
    if(fscanf(fin,"%s %lf %lf %lf",name,&min,&max,&res) != 4){
      ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
      return false;
    }
    names_.push_back(name);
    min_.push_back(min);
    max_.push_back(max);
    res_.push_back(res);
  }

  printf("read num vertices\n");
  //read in the number of vertices
  int num_vertices;
  if(fscanf(fin,"%d", &num_vertices) != 1){
    ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
    return false;
  }
  for(int i=0; i<num_vertices; i++){
    EGraphVertex* v = new EGraphVertex();
    v->id = i;
    id2vertex.push_back(v);
  }

  printf("read each vertex\n");
  //read in each vertex
  for(int i=0; i<num_vertices; i++){
    printf("vertex %d\n",i);
    //read in the vertex coordinate
    EGraphVertex* v = id2vertex[i];
    printf("1\n");
    double val;
    vector<double> coord;
    for(int j=0; j<num_dimensions; j++){
      if(fscanf(fin,"%lf",&val) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
        return false;
      }
      coord.push_back(val);
    }
    contToDisc(coord,v->coord);
    for(int j=0; j<num_constants_; j++){
      if(fscanf(fin,"%lf",&val) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
        return false;
      }
      v->constants.push_back(val);
    }
    printf("2\n");
    int idx = getHashBin(v->coord);
    printf("3\n");
    hashtable[idx].push_back(v);
    printf("4\n");

    printf("  read num neighbors\n");
    //read in the number of neighbors
    int num_neighbors;
    if(fscanf(fin,"%d", &num_neighbors) != 1){
      ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
      return false;
    }

    printf("  read in neighbors\n");
    //read in the neighbors
    int id;
    for(int j=0; j<num_neighbors; j++){
      if(fscanf(fin,"%d",&id) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
        return false;
      }
      v->neighbors.push_back(id2vertex[id]);
    }

    printf("  read in costs\n");
    //read in the costs to the neighbors
    int cost;
    for(int j=0; j<num_neighbors; j++){
      if(fscanf(fin,"%d",&cost) != 1){
        ROS_ERROR("E-Graph file \"%s\" is formatted incorrectly...",filename.c_str());
        return false;
      }
      v->costs.push_back(cost);
    }
  }

  return true;
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
  for(unsigned int i=0; i<res_.size(); i++){
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

EGraph::EGraphVertex* EGraph::createVertex(vector<int>& coord, vector<double>& constants){
  EGraphVertex* v = new EGraphVertex();
  v->coord = coord;
  v->constants = constants;
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

void EGraph::discToCont(EGraphVertex* v, vector<double>& c){
  c.clear();
  for(unsigned int i=0; i<v->coord.size(); i++)
    c.push_back(v->coord[i]*res_[i]+min_[i]);
  for(unsigned int i=0; i<v->constants.size(); i++)
    c.push_back(v->constants[i]);
}

double round(double r) {
  return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

void EGraph::contToDisc(vector<double> c, vector<int>& d){
  d.clear();
  for(unsigned int i=0; i<res_.size(); i++){
    //printf("%f-%f=%f\n",c[i],min_[i],c[i]-min_[i]);
    //printf("%f-%f)/%f=%f\n",c[i],min_[i],res_[i],(c[i]-min_[i])/(res_[i]));
    //printf("int(%f-%f)/%f)=%d\n",c[i],min_[i],res_[i],int((c[i]-min_[i])/(res_[i])));
    //d.push_back(int((c[i]-min_[i])/(res_[i])));
    d.push_back(round((c[i]-min_[i])/(res_[i])));
    //printf("huh.... %d\n",d.back());
  }
}
