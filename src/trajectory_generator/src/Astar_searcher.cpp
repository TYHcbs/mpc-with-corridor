#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initValue(double _resolution, Eigen::Vector3d local_xyz_l,
                                  Eigen::Vector3d local_xyz_u, Eigen::Vector3d global_xyz_l,
                                  Eigen::Vector3d global_xyz_u,int local_max_x_id,
                                  int local_max_y_id, int local_max_z_id, 
                                  int global_max_x_id, int global_max_y_id, int global_max_z_id) {
  lc_xl = local_xyz_l(0);
  lc_yl = local_xyz_l(1);
  lc_zl = local_xyz_l(2);

  lc_xu = local_xyz_u(0);
  lc_yu = local_xyz_u(1);
  lc_zu = local_xyz_u(2);

  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);
  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = local_max_x_id;//!!! //for local map size
  GLY_SIZE = local_max_y_id;
  GLZ_SIZE = local_max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  GLX_SIZE_GLB = global_max_x_id;
  GLY_SIZE_GLB = global_max_y_id;
  GLZ_SIZE_GLB = global_max_z_id;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  // std::cout<<"lc_xl_temp = "<<lc_xl_temp<<std::endl;
  // std::cout<<"lc_xu_temp = "<<lc_xu_temp<<std::endl;
  // std::cout<<"lc_yl_temp = "<<lc_xl_temp<<std::endl;
  // std::cout<<"lc_yu_temp = "<<lc_xl_temp<<std::endl;
  // std::cout<<"lc_zl_temp = "<<lc_xl_temp<<std::endl;
  // std::cout<<"lc_zu_temp = "<<lc_xl_temp<<std::endl;

  std::cout<<"GLX_SIZE = "<<GLX_SIZE<<std::endl;
  std::cout<<"GLY_SIZE = "<<GLY_SIZE<<std::endl;
  std::cout<<"GLZ_SIZE = "<<GLZ_SIZE<<std::endl; 
}

void AstarPathFinder::initGridMap(){
  std::cout<<"Inside initGridMap" << std::endl;
  data = new uint8_t[GLXYZ_SIZE];
  std::cout<<"inside initGridMap, after data = xxxx" << std::endl;
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
  std::cout<<"inside initGridMap, after memset(xxx)" << std::endl;

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  std::cout<<"inside initGridMap, after GridNodeMap initialize " << std::endl;
  // std::cout<<"GLX_SIZE, GLY_SIZE, GLZ_SIZE:  "<<GLX_SIZE<<", "<<GLY_SIZE<<", "<<GLZ_SIZE<<std::endl;
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE]; //selfadd:???
      for (int k = 0; k < GLZ_SIZE; k++) {
        Eigen::Vector3i tmpIdx(i,j,k);
        Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
        // std::cout<<"inside initGridMap, after gridIndex2coord " << std::endl;
        // std::cout<<"tmpIdx: "<< tmpIdx.transpose() << std::endl;
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
  std::cout<<"End of initGridMap, after for loop " << std::endl;
}


void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = INF;
  ptr->fScore = INF;
}

void AstarPathFinder::resetUsedGrids() {
  ROS_INFO("[!] Start resetUsedGrids,GLX_SIZE,GLY_SIZE,GLZ_SIZE= %d, %d, %d",GLX_SIZE,GLY_SIZE,GLZ_SIZE);
  for (int i = 0; i < GLX_SIZE; i++){
    for (int j = 0; j < GLY_SIZE; j++){
      for (int k = 0; k < GLZ_SIZE; k++){
        ROS_INFO("Local map Index =%d, %d, %d", i, j, k);  
        resetGrid(GridNodeMap[i][j][k]);
      }
    }
  }
  ROS_INFO("[Debug] resetUsedGrids() Ends.");
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
              const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
    coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;
  // //index in global map
  // int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);//!!??
  // int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  // int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  // //index in local map
  // int idx_x_local_mid = idx_x-center_idx_x;
  // int idx_y_local_mid = idx_y-center_idx_y;
  // int idx_z_local_mid = idx_z-center_idx_z;a
  Vector3i idx_pt;
  Vector3d coord_pt(coord_x,coord_y,coord_z);
  idx_pt = coord2gridIndex(coord_pt);
  data[idx_pt(0) * GLYZ_SIZE + idx_pt(1) * GLZ_SIZE + idx_pt(2)] = 1;
}

void AstarPathFinder::setObsVector(std::vector<Eigen::Vector3d> &cloud, double radius) {
  memset(data, 0, GLXYZ_SIZE*sizeof(uint8_t));//!!

  for (int i = 0; i < cloud.size(); i++) {
    Eigen::Vector3d new_pt;
    new_pt = cloud[i];
    if (coorIsInMap(new_pt) == false) continue;
    // new_pt << cloud[i].x(), cloud[i].y(), resolution*2;
    // bool flag = CheckPoint(new_pt);
    // flag = false;
    for (double x = -radius; x <= radius; x += resolution) {
      for (double y = -radius; y <= radius; y += resolution) {
        // if (flag == false) {
          for (double z = -radius; z <= radius; z += resolution) {
            setObs(cloud[i].x()+x, cloud[i].y()+y, cloud[i].z()+z);
          }
      }
    }
  }
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", (int)visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;
  // index in data -> cordinate in global sys
  pt(0) = ((double)index(0) + 0.5) * resolution + lc_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + lc_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + lc_zl;
  pt += center_;// added
  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  // index in data -> cordinate in global sys
  Vector3i idx;
  Eigen::Vector3d error_pt = pt - center_;
  idx << min(max(int((error_pt(0) - lc_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((error_pt(1) - lc_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((error_pt(2) - lc_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {

  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {
        // exclude the point it-self
        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2) + dz;

        // if not free, dont store it.
        // if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
        //     neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
        //     neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
        //   continue;
        // }
        if (!isFree(neighborIdx(0),neighborIdx(1),neighborIdx(2))) { //selfadd; why no need Astarxx::, why no need a = isFree()???
          continue;
        }

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  double heu = 0.0;
  double x1, y1, z1;
  double x2, y2, z2;
  double dx,dy,dz;
  double dmin,dmid,dmax;
  string heu_type;

  x1 = (double)node1->index[0];
  y1 = (double)node1->index[1];
  z1 = (double)node1->index[2];
  x2 = (double)node2->index[0];
  y2 = (double)node2->index[1];
  z2 = (double)node2->index[2];

  dx = fabs(x1 - x2);
  dy = fabs(y1 - y2);
  dz = fabs(z1 - z2);
  dmin = min({dx, dy, dz});
  dmax = max({dx, dy, dz});
  dmid = dx + dy + dz - dmin - dmax;

  // Define heuristics type:
  heu_type = "Diagonal";
  
  if(heu_type=="Euclidean"){
      heu = sqrt(dx * dx + dy * dy + dz * dz);
  }
  else if (heu_type=="Manhatten")
  {
      heu = dx+dy+dz;
  }
  else if (heu_type=="Diagonal")
  {
      heu = dmin*(sqrt(3)-sqrt(2)) + dmid*(sqrt(2)-1) + dmax; //selfadd: need explain!
  }
  else{
      ROS_INFO ("Error: invalid heuristic type!");
  }    
  
  // Tie breaker: //TODO: explain
  heu *= 1.001;
  // Test Info:
  //ROS_INFO("type=%s, heu=%.2f", heu_type, heu);
  return heu;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();
  cout <<"***start Astar graph search***"<< endl; // for test
  ROS_INFO("map size: %3fm ~ %3fm , center: %3f,%3f,%3f ,Index: GLX_SIZE,GLY_SIZE,GLZ_SIZE: %d, %d, %d",lc_xl,lc_xu,
      center_(0),center_(1),center_(2),
      GLX_SIZE,GLX_SIZE,GLX_SIZE);

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;
  std::cout<<"after coord2indx"<<std::endl;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);
  std::cout<<"after indx2coord"<<std::endl;

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);
  std::cout<<"after  startPtr,endPtr GridNode"<<std::endl;

  std::cout<<"start search, openset clear"<<std::endl;
  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->gScore = 0;
  startPtr->fScore = getHeu(startPtr, endPtr);
  startPtr->id = 1;
  startPtr->coord = start_pt; // start_pt:Vector3d
  openSet.insert(make_pair(startPtr->fScore, startPtr));
  std::cout<<"openSet.insert(make_pair(startPtr->fScore, startPtr))"<<std::endl;

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/

  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  // cout <<"***inside Astar graph search, before while loop***"<< endl; // for test 
  while (!openSet.empty()) {
    //get a current ptr from open set, //erase it from openset, and //put it into close set(id=-1)
    currentPtr = openSet.begin()->second;
    openSet.erase(openSet.begin());
    currentPtr->id = -1;// put in close set

    //if is goal->end
    if( currentPtr->index == goalIdx ){
        ros::Time time_2 = ros::Time::now();
        terminatePtr = currentPtr;
        ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
        return;
    }// if know the last one, than use "father" can know the whole path
    // cout <<"***inside Astar graph search, before AstarGetSucc***"<< endl; // for test
    //get neibour and sucdist
    AstarGetSucc(currentPtr,neighborPtrSets,edgeCostSets);//selfadd: do i need to add &?  
    // cout <<"***inside Astar graph search, before for loop for neighbour***"<< endl; // for test 
    //for every neighbour:
    for(int i = 0; i < (int)neighborPtrSets.size(); i++){
      neighborPtr = neighborPtrSets[i];//selfadd: () or []???
      tentative_gScore = currentPtr->gScore + (double)edgeCostSets[i];
      //if not free(obstacle and bound)(excluded by get succ)
      //if in openset 
      if(neighborPtr->id==1){
       //g=g_old+sucdist,if cost_new < cost_old-> g = cost_new, father = current
        if(tentative_gScore<neighborPtr->gScore){
          neighborPtr->gScore = tentative_gScore;
          neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
          neighborPtr->cameFrom = currentPtr;
        }
        continue;
      }      
      //if not in open not in close
      else if(neighborPtr->id==0){
        //neighbour g=g_old+sucdist, get heu, f=h+g, father = current, put into openset
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
        neighborPtr->cameFrom = currentPtr;
        openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));// is insert only for customized data structure like openset
        neighborPtr->id=1;
        continue;
      }
      //if in close set
      else{
        // do nothing
        continue;
      }
    }
    // cout <<"***inside Astar graph search, after for loop for neighbour***"<< endl; // for test 
  }
  // cout <<"***inside Astar graph search, after while loop***"<< endl; // for test 
  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  
}

vector<Vector3d> AstarPathFinder::getPath() { //所以在Astarpathsearcher中生成的node实体不会消失是吗
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;
  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  GridNodePtr tmpPtr = terminatePtr;
  while(tmpPtr != nullptr){
      gridPath.push_back(tmpPtr);
      tmpPtr = tmpPtr->cameFrom;
  }
  for (auto ptr: gridPath)//!coding
      path.push_back(ptr->coord);
      
  reverse(path.begin(),path.end());

  ROS_INFO("map size: %3fm ~ %3fm , center: %3f,%3f,%3f ,Index: GLX_SIZE,GLY_SIZE,GLZ_SIZE: %d, %d, %d",lc_xl,lc_xu,
        center_(0),center_(1),center_(2),
        GLX_SIZE,GLX_SIZE,GLX_SIZE);
  return path;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  // cout <<"***start pathSimplify***"<< endl; // for test
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/

  // Base case: if there are 2 or fewer points, return them
  if (path.size() <= 2) {
      return path;
  }
  // Find the point with the maximum distance
  double dmax = 0;
  size_t index = 0;
  const size_t end = path.size() - 1;
  // Calculate perpendicular distance for each point
  // change to use cross product 
  Vector3d start_point = path[0];
  Vector3d end_point = path[end];
  for (size_t i = 1; i < end; ++i) {
      // calculate perpendicular distance
      Vector3d i_point = path[i];

      Vector3d vec1 = end_point - start_point;
      Vector3d vec2 = i_point - start_point;
      double perpen_dist = (vec2.cross(vec1)).norm() / vec1.norm();


      if (perpen_dist > dmax) {
          index = i;
          dmax = perpen_dist;
      }
  }
  // check if dmax is larger than path_resolution
  if (dmax > path_resolution){

      vector<Vector3d> firstPart(path.begin(), path.begin() + index);
      vector<Vector3d> secondPart(path.begin() + index,path.end());

      vector<Vector3d> recResults1 = pathSimplify(firstPart, path_resolution);
      vector<Vector3d> recResults2 = pathSimplify(secondPart, path_resolution);

      subPath.reserve(recResults1.size() + recResults2.size() - 1);
      subPath.insert(subPath.end(), recResults1.begin(), recResults1.end());
      subPath.insert(subPath.end(), recResults2.begin() + 1, recResults2.end());


  }else{
    subPath.reserve(2); // added to prevent multiple reallocations
    subPath.push_back(path[0]);
    subPath.push_back(path[end]);
  }

  return subPath;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) { //polyCoeff: m x 3*8
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/
  // Number of segments in the trajectory
  int segments = time.size();
  
  // Check each segment
  for(int seg = 0; seg < segments; seg++) {
      double t = 0.0;
      // Sample points within the segment duration
      while(t <= time(seg)) {
          Vector3d pos = Vector3d::Zero();
          // Vector3d vel = Vector3d::Zero();
          
          // Evaluate position polynomial //selfadd:  MatrixXd PolyCoeff(m, 3 * p_num1d);
          int p_num1d = polyCoeff.cols()/3;
          for(int i = 0; i < p_num1d; i++) {
              Vector3d polyCoeff_trans;
              polyCoeff_trans(0) = polyCoeff(seg, i);
              polyCoeff_trans(1) = polyCoeff(seg, p_num1d + i);
              polyCoeff_trans(2) = polyCoeff(seg, 2*p_num1d + i);
              pos += polyCoeff_trans * pow(t/time(seg), i);// but pos 是列向量？
              // cout<<"pos= "<<pos<<endl;
              // if(i > 0) {
              //     vel += i * polyCoeff.block<3,1>(seg*3, i) * pow(t, i-1);
              // }
          }
          // int idx_x = static_cast<int>((pos.x() - gl_xl) * inv_resolution); //index in global sys
          // int idx_y = static_cast<int>((pos.y() - gl_yl) * inv_resolution);
          // int idx_z = static_cast<int>((pos.z() - gl_zl) * inv_resolution);
          // Check position bounds
          if(!isFree(pos.x(),pos.y(),pos.z())) {
              unsafe_segment = seg;
              std::cout<<"Find unsafe_segment!: "<<unsafe_segment<<std::endl;
              return unsafe_segment;
          }
          
          // // Check velocity constraints
          // double vel_norm = vel.norm();
          // if(vel_norm > _max_vel) {
          //     unsafe_segment = seg;
          //     return unsafe_segment;
          // }
          
          t += 0.1; // Sample step 
      }
  }

  return unsafe_segment;
}

void AstarPathFinder::FloydHandle(const std::vector<Eigen::Vector3d>& astar_path, 
                                    std::vector<Eigen::Vector3d>& waypoint)
{
  waypoint.clear();
  waypoint = pathSimplify(astar_path, 0.2);
  // std::cout << "simplify size:" << waypoint.size() << std::endl;

  // generate Floyd path(twice for optimal trajectory)
  for(int time = 0; time < 2; time++){
    for(int i = waypoint.size()-1; i > 0; i--){
      for(int j = 0; j < i-1; j++){
        if(AstarPathFinder::CheckLineObstacleFree(waypoint[i], waypoint[j]) == true){
          for(int k = i-1; k > j; k--) {
            waypoint.erase(waypoint.begin()+k); // delete redundant inflection points
          }
          i = j;
          break;
        }
      }
    }
  }
}