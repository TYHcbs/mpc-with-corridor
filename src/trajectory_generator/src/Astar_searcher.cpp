#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Eigen::Vector3d local_xyz_l,
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
        // Eigen::Vector3d pos = gridIndex2coord(tmpIdx);
        // std::cout<<"inside initGridMap, after gridIndex2coord " << std::endl;
        // std::cout<<"tmpIdx: "<< tmpIdx.transpose() << std::endl;
        // GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx);
      }
    }
  }
  std::cout<<"End of initGridMap, after for loop " << std::endl;
  ROS_INFO("Local map bounds: (%f,%f,%f) to (%f,%f,%f)",
           lc_xl, lc_yl, lc_zl, lc_xu, lc_yu, lc_zu);
  ROS_INFO("Global map bounds: (%f,%f,%f) to (%f,%f,%f)", 
           gl_xl, gl_yl, gl_zl, gl_xu, gl_yu, gl_zu);
  // initVisualization(ros::NodeHandle& nh);
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
  int obstacle_count = 0;
  for(int i=0; i<GLXYZ_SIZE; i++) {
      if(data[i] == 1) obstacle_count++;
  }
  ROS_INFO("Set %d obstacles", obstacle_count);
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
          visited_nodes.push_back(gridIndex2coord(GridNodeMap[i][j][k]->index));// modified,origin: gridIndex2coord
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

bool AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();
  ROS_WARN("***start Astar graph search***"); // for test
  ROS_INFO("map size: %3fm ~ %3fm , x range: %3fm ~ %3fm , y range: %3fm ~ %3fm ,center: %3f,%3f,%3f ,Index: GLX_SIZE,GLY_SIZE,GLZ_SIZE: %d, %d, %d",lc_xl,lc_xu,
    lc_xl+center_(0),lc_xu+center_(0),lc_yl+center_(1),lc_yu+center_(1),
    center_(0),center_(1),center_(2),
    GLX_SIZE,GLX_SIZE,GLX_SIZE);

  // 初始化时显示地图和障碍物
  visualizeLocalMapBound();
  visualizeObstacles();
  // Update basic 2D visualization
  // Update2DVisualization();

  // Mark start and end points
  // cv::Point start_pt_img = World2Img(start_pt);
  // cv::Point end_pt_img = World2Img(end_pt);
  // cv::circle(vis_img_, start_pt_img, 4, cv::Vec3b(0,255,0), -1); // Green
  // cv::circle(vis_img_, end_pt_img, 4, cv::Vec3b(0,0,255), -1);   // Red

  astar_mutex.lock();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;
  std::cout<<"after coord2indx"<<std::endl;

  visualizeStartEndPoints(start_pt, end_pt);

  if (isOccupied(start_idx)) {
      ROS_ERROR("Start point (%f,%f,%f) -> idx (%d,%d,%d) is occupied",
              start_pt.x(), start_pt.y(), start_pt.z(),
              start_idx(0), start_idx(1), start_idx(2));
      return false;
  }
  if (isOccupied(end_idx)) {
      ROS_ERROR("[Local Astar] end point is in obstacle!");
      return false;
  }

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);
  std::cout<<"after indx2coord"<<std::endl;

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx);//modified, origin: new GridNode(start_idx, start_pt)
  GridNodePtr endPtr = new GridNode(end_idx);//modified, origin: new GridNode(end_idx, end_pt)
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
  // startPtr->coord = start_pt; // start_pt:Vector3d //modifed for store index instead of coord, will get coord information back  in getpath
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

    // Visualize visited node in blue
    // Visualize2DNode(currentPtr, cv::Vec3b(255,0,0));

    //if is goal->end
    if( currentPtr->index == goalIdx ){
        ros::Time time_2 = ros::Time::now();
        terminatePtr = currentPtr;

        // Get and visualize the final path in green
        // std::vector<GridNodePtr> path;
        // GridNodePtr node = terminatePtr;
        // while (node->cameFrom != NULL) {
        //     path.push_back(node);
        //     node = node->cameFrom;
        // }
        // path.push_back(node);
        // Visualize2DPath(path, cv::Vec3b(0,255,0));

        ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
        std::cout<<"[ Debug] A* Find goal!---------------------------------------"<<std::endl;

        astar_mutex.unlock();
        return true;
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
      // 每次扩展后更新可视化
    visualizeVisited();
    ros::Duration(0.05).sleep();  // 控制可视化更新频率
  }
  // cout <<"***inside Astar graph search, after while loop***"<< endl; // for test 
  // if search fails
  ros::Time time_2 = ros::Time::now();
  ROS_WARN("A* failed with %ld nodes in open set", openSet.size());
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  astar_mutex.unlock();
  return false;
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
  for (auto ptr: gridPath){//!coding
      Eigen::Vector3d coord; 
      coord = gridIndex2coord(ptr->index);
      path.push_back(coord);
      // path.push_back(ptr->coord);
  }

  reverse(path.begin(),path.end());

  // ROS_INFO("map size: %3fm ~ %3fm , center: %3f,%3f,%3f ,Index: GLX_SIZE,GLY_SIZE,GLZ_SIZE: %d, %d, %d",lc_xl,lc_xu,
        // center_(0),center_(1),center_(2),
        // GLX_SIZE,GLX_SIZE,GLX_SIZE);
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
//-------------------------------For test---------------------------------
// 初始化这些publishers
void AstarPathFinder::initVisualization(ros::NodeHandle& nh) {
    map_vis_pub_ = nh.advertise<visualization_msgs::Marker>("/astar/local_map_bound", 1);
    obstacle_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/astar/obstacles", 1);
    visited_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/astar/visited", 1);
    search_region_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/astar/search_region", 1);
    start_end_vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/astar/start_end_points", 1);
}

// 可视化本地地图边界
void AstarPathFinder::visualizeLocalMapBound() {
    visualization_msgs::Marker bound_marker;
    bound_marker.header.frame_id = "world";
    bound_marker.type = visualization_msgs::Marker::LINE_LIST;
    bound_marker.action = visualization_msgs::Marker::ADD;
    bound_marker.scale.x = 0.05;  // 线宽
    bound_marker.color.b = 1.0;
    bound_marker.color.a = 1.0;

    // 计算local map在世界坐标系下的8个顶点
    vector<Vector3d> vertices;
    Vector3d local_min(center_.x() + lc_xl, center_.y() + lc_yl, center_.z() + lc_zl);
    Vector3d local_max(center_.x() + lc_xu, center_.y() + lc_yu, center_.z() + lc_zu);
    
    // 添加构成边框的12条线
    vector<pair<Vector3d, Vector3d>> lines = {
        // bottom 4 line
        {Vector3d(local_min.x(), local_min.y(), local_min.z()), 
         Vector3d(local_max.x(), local_min.y(), local_min.z())},

        {Vector3d(local_min.x(), local_min.y(), local_min.z()), 
         Vector3d(local_min.x(), local_max.y(), local_min.z())},

        {Vector3d(local_min.x(), local_max.y(), local_min.z()), 
         Vector3d(local_max.x(), local_max.y(), local_min.z())},

        {Vector3d(local_max.x(), local_min.y(), local_min.z()), 
         Vector3d(local_max.x(), local_max.y(), local_min.z())},

        {Vector3d(local_min.x(), local_min.y(), local_min.z()), 
         Vector3d(local_max.x(), local_min.y(), local_min.z())},

        //upper 4 line
        {Vector3d(local_min.x(), local_min.y(), local_max.z()), 
         Vector3d(local_max.x(), local_min.y(), local_max.z())},
        
        {Vector3d(local_min.x(), local_min.y(), local_max.z()), 
         Vector3d(local_min.x(), local_max.y(), local_max.z())},

        {Vector3d(local_min.x(), local_max.y(), local_max.z()), 
         Vector3d(local_max.x(), local_max.y(), local_max.z())},

        {Vector3d(local_max.x(), local_min.y(), local_max.z()), 
         Vector3d(local_max.x(), local_max.y(), local_max.z())},

        {Vector3d(local_min.x(), local_min.y(), local_max.z()), 
         Vector3d(local_max.x(), local_min.y(), local_max.z())},

        //vertical 4 lines
        {Vector3d(local_min.x(), local_min.y(), local_min.z()), 
         Vector3d(local_min.x(), local_min.y(), local_max.z())},
        
        {Vector3d(local_min.x(), local_max.y(), local_min.z()), 
         Vector3d(local_min.x(), local_max.y(), local_max.z())},

        {Vector3d(local_max.x(), local_min.y(), local_min.z()), 
         Vector3d(local_max.x(), local_min.y(), local_max.z())},

        {Vector3d(local_max.x(), local_max.y(), local_min.z()), 
         Vector3d(local_max.x(), local_max.y(), local_max.z())},

    };

    for(const auto& line : lines) {
        geometry_msgs::Point p1, p2;
        p1.x = line.first.x(); p1.y = line.first.y(); p1.z = line.first.z();
        p2.x = line.second.x(); p2.y = line.second.y(); p2.z = line.second.z();
        bound_marker.points.push_back(p1);
        bound_marker.points.push_back(p2);
        // ROS_INFO("Visualize local map region, p1 = %f,%f,%f, p2 = %f,%f,%f.",p1.x,p1.y,p1.z,p2.x,p2.y,p2.z);
    }

    map_vis_pub_.publish(bound_marker);
}

// 可视化障碍物格子
void AstarPathFinder::visualizeObstacles() {
    visualization_msgs::MarkerArray obstacle_markers;
    visualization_msgs::Marker obstacle_marker;
    obstacle_marker.header.frame_id = "world";
    obstacle_marker.type = visualization_msgs::Marker::CUBE_LIST;
    obstacle_marker.scale.x = resolution;
    obstacle_marker.scale.y = resolution;
    obstacle_marker.scale.z = resolution;
    obstacle_marker.color.r = 1.0;
    obstacle_marker.color.a = 0.5;

    // 遍历所有格子
    for(int x = 0; x < GLX_SIZE; x++) {
        for(int y = 0; y < GLY_SIZE; y++) {
            for(int z = 0; z < GLZ_SIZE; z++) {
                if(isOccupied(x, y, z)) {
                    Vector3d coord = gridIndex2coord(Vector3i(x, y, z));
                    geometry_msgs::Point p;
                    p.x = coord.x();
                    p.y = coord.y();
                    p.z = coord.z();
                    obstacle_marker.points.push_back(p);
                }
            }
        }
    }

    obstacle_markers.markers.push_back(obstacle_marker);
    obstacle_vis_pub_.publish(obstacle_markers);
}

// 可视化已访问的格子
void AstarPathFinder::visualizeVisited() {
    visualization_msgs::MarkerArray visited_markers;
    visualization_msgs::Marker visited_marker;
    visited_marker.header.frame_id = "world";
    visited_marker.type = visualization_msgs::Marker::CUBE_LIST;
    visited_marker.scale.x = resolution * 0.9;  // 略小于格子大小
    visited_marker.scale.y = resolution * 0.9;
    visited_marker.scale.z = resolution * 0.9;
    visited_marker.color.g = 0.5;
    visited_marker.color.b = 0.8;
    visited_marker.color.a = 0.3;

    // 添加所有id为-1的节点(closed list)
    for(int x = 0; x < GLX_SIZE; x++) {
        for(int y = 0; y < GLY_SIZE; y++) {
            for(int z = 0; z < GLZ_SIZE; z++) {
                if(GridNodeMap[x][y][z]->id == -1) {
                    Vector3d coord = gridIndex2coord(Vector3i(x, y, z));
                    geometry_msgs::Point p;
                    p.x = coord.x();
                    p.y = coord.y();
                    p.z = coord.z();
                    visited_marker.points.push_back(p);
                }
            }
        }
    }

    visited_markers.markers.push_back(visited_marker);
    visited_vis_pub_.publish(visited_markers);
}

void AstarPathFinder::visualizeStartEndPoints(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
    visualization_msgs::MarkerArray marker_array;
    
    // 起点标记(绿色球体)
    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "world";
    start_marker.id = 0;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.scale.x = resolution * 2;
    start_marker.scale.y = resolution * 2;
    start_marker.scale.z = resolution * 2;
    start_marker.color.g = 1.0;
    start_marker.color.a = 1.0;
    
    start_marker.pose.position.x = start_pt.x();
    start_marker.pose.position.y = start_pt.y();
    start_marker.pose.position.z = start_pt.z();
    
    // 终点标记(红色球体)
    visualization_msgs::Marker end_marker;
    end_marker.header.frame_id = "world";
    end_marker.id = 1;
    end_marker.type = visualization_msgs::Marker::SPHERE;
    end_marker.action = visualization_msgs::Marker::ADD;
    end_marker.scale.x = resolution * 2;
    end_marker.scale.y = resolution * 2;
    end_marker.scale.z = resolution * 2;
    end_marker.color.r = 1.0;
    end_marker.color.a = 1.0;
    
    end_marker.pose.position.x = end_pt.x();
    end_marker.pose.position.y = end_pt.y();
    end_marker.pose.position.z = end_pt.z();

    // 添加连接线(白色)
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "world";
    line_marker.id = 2;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = resolution * 0.5;  // 线宽
    line_marker.color.r = 1.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 1.0;
    line_marker.color.a = 0.5;

    geometry_msgs::Point p1, p2;
    p1.x = start_pt.x(); p1.y = start_pt.y(); p1.z = start_pt.z();
    p2.x = end_pt.x(); p2.y = end_pt.y(); p2.z = end_pt.z();
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);

    marker_array.markers.push_back(start_marker);
    marker_array.markers.push_back(end_marker);
    marker_array.markers.push_back(line_marker);
    
    start_end_vis_pub_.publish(marker_array);
}

// void AstarPathFinder::Init2DVisualization() {
//     // Set visualization parameters
//     vis_resolution_ = 50;  // 50 pixels per meter
//     double map_width = gl_xu - gl_xl;
//     double map_height = gl_yu - gl_yl;
//     img_width_ = map_width * vis_resolution_;
//     img_height_ = map_height * vis_resolution_;
    
//     // Create window
//     cv::namedWindow("A* 2D Visualization", cv::WINDOW_AUTOSIZE);
// }

// void AstarPathFinder::Update2DVisualization() {
//     // Create white background
//     vis_img_ = cv::Mat(img_height_, img_width_, CV_8UC3, cv::Scalar(255,255,255));
    
//     // Draw grid lines
//     for(double x = gl_xl; x <= gl_xu; x += resolution) {
//         cv::Point pt1 = World2Img(Eigen::Vector3d(x, gl_yl, 0));
//         cv::Point pt2 = World2Img(Eigen::Vector3d(x, gl_yu, 0));
//         cv::line(vis_img_, pt1, pt2, cv::Scalar(220,220,220), 1);
//     }
//     for(double y = gl_yl; y <= gl_yu; y += resolution) {
//         cv::Point pt1 = World2Img(Eigen::Vector3d(gl_xl, y, 0));
//         cv::Point pt2 = World2Img(Eigen::Vector3d(gl_xu, y, 0));
//         cv::line(vis_img_, pt1, pt2, cv::Scalar(220,220,220), 1);
//     }
    
//     // Draw obstacles
//     for(int i = 0; i < GLX_SIZE; i++) {
//         for(int j = 0; j < GLY_SIZE; j++) {
//             if(isOccupied(i, j, GLZ_SIZE/2)) {  // Check middle height for 2D view
//                 Eigen::Vector3d pos = gridIndex2coord(Eigen::Vector3i(i,j,GLZ_SIZE/2));
//                 cv::Point pt = World2Img(pos);
//                 cv::circle(vis_img_, pt, resolution*vis_resolution_/2, cv::Scalar(0,0,0), -1);
//             }
//         }
//     }
    
//     cv::imshow("A* 2D Visualization", vis_img_);
//     cv::waitKey(1);
// }

// void AstarPathFinder::Visualize2DNode(const GridNodePtr& node, const cv::Vec3b& color) {
//     Eigen::Vector3d pos = gridIndex2coord(node->index);
//     cv::Point pt = World2Img(pos);
//     cv::circle(vis_img_, pt, 2, color, -1);
    
//     cv::imshow("A* 2D Visualization", vis_img_);
//     cv::waitKey(1);
// }

// void AstarPathFinder::Visualize2DPath(const std::vector<GridNodePtr>& path, const cv::Vec3b& color) {
//     for(int i = 0; i < path.size()-1; i++) {
//         Eigen::Vector3d pos1 = gridIndex2coord(path[i]->index);
//         Eigen::Vector3d pos2 = gridIndex2coord(path[i+1]->index);
//         cv::Point pt1 = World2Img(pos1);
//         cv::Point pt2 = World2Img(pos2);
//         cv::line(vis_img_, pt1, pt2, color, 2);
//     }
    
//     cv::imshow("A* 2D Visualization", vis_img_);
//     cv::waitKey(1);
// }

