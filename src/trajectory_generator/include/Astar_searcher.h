#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"


class AstarPathFinder
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr *** GridNodeMap;
		Eigen::Vector3i goalIdx;
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLX_SIZE_GLB, GLY_SIZE_GLB, GLZ_SIZE_GLB;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution;
		double lc_xl, lc_yl, lc_zl;
		double lc_xu, lc_yu, lc_zu;
		double lc_xl_temp, lc_yl_temp, lc_zl_temp;
		double lc_xu_temp, lc_yu_temp, lc_zu_temp;

		double gl_xl, gl_yl, gl_zl;//!fuzhi
		double gl_xu, gl_yu, gl_zu;//initi map 函数输入
		Eigen::Vector3d center_;
		// int center_idx_x, center_idx_y, center_idx_z;

		GridNodePtr terminatePtr;
		std::multimap<double, GridNodePtr> openSet;

		double getHeu(GridNodePtr node1, GridNodePtr node2);
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;
		inline bool coorIsInMap(const Eigen::Vector3d & pt) {
			Eigen::Vector3d error = pt - center_;
			if (error.x() > lc_xl && error.x() < lc_xu && 
				error.y() > lc_yl && error.y() < lc_yu && 
				error.z() > lc_zl && error.z() < lc_zu) {
				return true;
			}
			return false;
    	}
		
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

		double inflation_radius_;
    	int inflation_cells_;

	public:
		AstarPathFinder(){};
		~AstarPathFinder(){};
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		void initValue(double _resolution, Eigen::Vector3d local_xyz_l,
                                  Eigen::Vector3d local_xyz_u, Eigen::Vector3d global_xyz_l,
                                  Eigen::Vector3d global_xyz_u,int local_max_x_id,
                                  int local_max_y_id, int local_max_z_id, 
                                  int global_max_x_id, int global_max_y_id, int global_max_z_id);
		void initGridMap();

		void setObs(const double coord_x, const double coord_y, const double coord_z);

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
		std::vector<Eigen::Vector3d> getVisitedNodes();
		std::vector<Eigen::Vector3d> pathSimplify(const std::vector<Eigen::Vector3d> &path, const double path_resolution);
		Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
		int safeCheck( Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);

		void setObsVector(std::vector<Eigen::Vector3d> &cloud, double radius = 0.2);

		void SetCenter(Eigen::Vector3d center) {
			ROS_INFO("In Setcenter");
			center_ = center;
			// //center index in global map
			// int center_idx_x = static_cast<int>((center_(0) - lc_xl) * inv_resolution);
			// int center_idx_y = static_cast<int>((center_(1) - lc_yl) * inv_resolution);
			// int center_idx_z = static_cast<int>((center_(2) - lc_zl) * inv_resolution);
		}
		void FloydHandle(const std::vector<Eigen::Vector3d>& astar_path, 
                                        std::vector<Eigen::Vector3d>& waypoint);
		
		bool CheckPoint(const Eigen::Vector3d& pt) {
			if (coorIsInMap(pt) == false) return true;
			if (isOccupied(coord2gridIndex(pt))) return false;
			return true;
    	}

		bool CheckStartEnd(const Eigen::Vector3d& pt) {
			Eigen::Vector3i idx = coord2gridIndex(pt);
			if (isOccupied(idx)) return false;
			else return true;
    	}
		bool CheckLineObstacleFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
			Eigen::Vector3d vector = p2 - p1;
			int point_num = vector.norm() / resolution;
			bool flag = true;
			for (int i = 1; i <= point_num; i++) {
				Eigen::Vector3d coor = p1 + vector * i / (point_num+1);
				if (coorIsInMap(coor) == false) continue;
				if (isOccupied(coord2gridIndex(coor))) {
					flag = false;
					break;
				}
			}
			return flag;
		}
		bool CheckPathFree(const std::vector<Eigen::Vector3d>& path) {
			bool flag = true;
			for (int i = 0; i < path.size(); i++) {
				if (coorIsInMap(path[i]) == false) continue;
				if (isOccupied(coord2gridIndex(path[i]))) {
					flag = false;
					break;
				}
			}
			return flag;
    	}

};
// init map, reset, setcenter, setObsVector, CheckLineObstacleFree, CheckPathFree, FloydHandle

//		// if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
		//     idx_y == GLY_SIZE)
		//   data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
		// else {
		//   data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
		//   int inflate_size = 4;  // 膨胀大小
		//   for(int i = -inflate_size; i <= inflate_size; i++){
		//       for(int j = -inflate_size; j <= inflate_size; j++){
		//           int x_number = idx_x + i;
		//           int y_number = idx_y + j;
		//           if(x_number >=0  && x_number < GLX_SIZE && 
		//              y_number >=0 && y_number < GLY_SIZE){
		//               data[x_number * GLYZ_SIZE + y_number * GLZ_SIZE + idx_z] = 1;
		//           }
		//       }
		//   }

		// }

#endif