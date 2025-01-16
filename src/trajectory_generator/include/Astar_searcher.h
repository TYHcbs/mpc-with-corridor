#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"
#include "node.h"
#include <mutex>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

// #include <opencv2/core/core.hpp>
// #include <opencv2/core/types.hpp>
// #include <opencv2/core/mat.hpp>

// // 在所有 OpenCV 头文件之前添加
// namespace cv {
//     class Mat;
//     class InputArray;
//     class OutputArray;
//     class InputOutputArray;
//     class InputArrayOfArrays;
//     class OutputArrayOfArrays;
//     struct Size;
//     struct Point;
//     struct Rect;
//     class TermCriteria;
// }

// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>


class AstarPathFinder
{	
	private:
		// 添加新的publishers
		ros::Publisher map_vis_pub_;        // 显示local地图范围
		ros::Publisher obstacle_vis_pub_;    // 显示障碍物格子
		ros::Publisher visited_vis_pub_;     // 显示已访问的格子
		ros::Publisher search_region_pub_;   // 显示搜索范围
		ros::Publisher start_end_vis_pub_;    // 用于可视化起点和终点

		std::mutex astar_mutex;

		// cv::Mat vis_img_;
		// int img_width_, img_height_;
		// double vis_resolution_;  // pixels per meter

		// // Convert world coordinates to image coordinates
		// cv::Point World2Img(const Eigen::Vector3d& pos) {
		// 	Eigen::Vector3d relative_pos = pos - center_;
		// 	int img_x = static_cast<int>((relative_pos.x() - lc_xl) * vis_resolution_ + img_width_/2);
		// 	int img_y = static_cast<int>(img_height_ - ((relative_pos.y() - gl_yl) * vis_resolution_ + img_height_/2));
		// 	return cv::Point(img_x, img_y);
		// }

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
		AstarPathFinder(){
			// 其他初始化代码
    		// Init2DVisualization();
		};
		~AstarPathFinder(){};
		bool AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		void initGridMap(double _resolution, Eigen::Vector3d local_xyz_l,
                                  Eigen::Vector3d local_xyz_u, Eigen::Vector3d global_xyz_l,
                                  Eigen::Vector3d global_xyz_u,int local_max_x_id,
                                  int local_max_y_id, int local_max_z_id, 
                                  int global_max_x_id, int global_max_y_id, int global_max_z_id);

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

		//---------------------------
		void initVisualization(ros::NodeHandle& nh);
		void visualizeLocalMapBound();
		void visualizeObstacles();
		void visualizeVisited();
		void visualizeStartEndPoints(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);

		// // For 2d visualization
		// void Init2DVisualization() {
		// 	// 设置可视化参数，考虑局部地图尺寸
		// 	vis_resolution_ = 50;  // 50 pixels per meter
		// 	img_width_ = static_cast<int>((lc_xu - lc_xl) * vis_resolution_);
		// 	img_height_ = static_cast<int>((lc_yu - lc_yl) * vis_resolution_);
			
		// 	// 创建窗口
		// 	cv::namedWindow("A* 2D Visualization", cv::WINDOW_AUTOSIZE);
    	// }

		// void Update2DVisualization() {
        // 	// 创建白色背景
		// 	vis_img_ = cv::Mat(img_height_, img_width_, CV_8UC3, cv::Scalar(255,255,255));
			
		// 	// 绘制网格线
		// 	for(double x = lc_xl; x <= lc_xu; x += resolution) {
		// 		cv::Point pt1 = World2Img(Eigen::Vector3d(x, lc_yl, 0));
		// 		cv::Point pt2 = World2Img(Eigen::Vector3d(x, lc_yu, 0));
		// 		cv::line(vis_img_, pt1, pt2, cv::Scalar(220,220,220), 1);
		// 	}
		// 	for(double y = lc_yl; y <= lc_yu; y += resolution) {
		// 		cv::Point pt1 = World2Img(Eigen::Vector3d(lc_xl, y, 0));
		// 		cv::Point pt2 = World2Img(Eigen::Vector3d(lc_xu, y, 0));
		// 		cv::line(vis_img_, pt1, pt2, cv::Scalar(220,220,220), 1);
		// 	}
			
		// 	// 绘制障碍物
		// 	for(int i = 0; i < GLX_SIZE; i++) {
		// 		for(int j = 0; j < GLY_SIZE; j++) {
		// 			if(isOccupied(i, j, GLZ_SIZE/2)) {  // 使用中间高度层作为2D视图
		// 				Eigen::Vector3d pos = gridIndex2coord(Eigen::Vector3i(i,j,GLZ_SIZE/2));
		// 				cv::Point pt = World2Img(pos);
		// 				cv::circle(vis_img_, pt, static_cast<int>(resolution*vis_resolution_/2), cv::Scalar(0,0,0), -1);
		// 			}
		// 		}
		// 	}
        
		// 	cv::imshow("A* 2D Visualization", vis_img_);
		// 	cv::waitKey(1);
    	// }

		// void Visualize2DNode(const GridNodePtr& node, const cv::Scalar& color) {
		// 	Eigen::Vector3d pos = gridIndex2coord(node->index);
		// 	cv::Point pt = World2Img(pos);
		// 	cv::circle(vis_img_, pt, 2, color, -1);
			
		// 	cv::imshow("A* 2D Visualization", vis_img_);
		// 	cv::waitKey(1);
		// }
		
		// void Visualize2DPath(const std::vector<GridNodePtr>& path, const cv::Scalar& color) {
		// 	if (path.empty()) return;
			
		// 	for(size_t i = 0; i < path.size()-1; i++) {
		// 		Eigen::Vector3d pos1 = gridIndex2coord(path[i]->index);
		// 		Eigen::Vector3d pos2 = gridIndex2coord(path[i+1]->index);
		// 		cv::Point pt1 = World2Img(pos1);
		// 		cv::Point pt2 = World2Img(pos2);
		// 		cv::line(vis_img_, pt1, pt2, color, 2);
		// 	}
			
		// 	cv::imshow("A* 2D Visualization", vis_img_);
		// 	cv::waitKey(1);
		// }

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