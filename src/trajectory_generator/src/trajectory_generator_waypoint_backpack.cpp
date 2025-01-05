#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>//PolyQPGeneration//selfadd ???
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d) //selfadd:MatrixXd path(int(grid_path.size()), 3); path.row(k) = grid_path[k]; 
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  cout << "[Debug] inside PolyQPGeneration, just start" << endl; // for test
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment
  // cout << "[Debug] inside PolyQPGeneration, 111" << endl; // for test
  int m = Time.size();
  cout << "[Debug] m segments:" << m << endl; // for test
  cout << "[Debug] path size:" << Path.rows() << endl; // for test
  MatrixXd PolyCoeff(m, 3 * p_num1d); //每行是一段polynomial，一行内，三个三个一组(x，y，z)，一共p_num1d组

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  //get C,A,Q
  // cout << "[Debug] inside PolyQPGeneration, 333" << endl; // for test
  Matrix3d I_3x3 = Matrix3d::Identity();
  MatrixXd I_12x12 = MatrixXd::Identity(12,12);
  MatrixXd One_3x3 = MatrixXd::Ones(3,3);
  double lambda = 1e-5;  // 正则化参数 origin: 1e-8
  // cout << "[Debug] inside PolyQPGeneration, 444" << endl; // for test
  MatrixXd Q_Matrix = MatrixXd::Zero(3*p_num1d*m,3*p_num1d*m);// define size? //24mx24m
  MatrixXd A_Matrix = MatrixXd::Zero(3*p_num1d*m,3*p_num1d*m); //24mx24m
  MatrixXd C_Matrix_Trans = MatrixXd::Zero(24*m,12*m+12); //24mx12m+12
  // cout << "[Debug] inside PolyQPGeneration, 555" << endl; // for test
  C_Matrix_Trans.block(0,0,3*4,3*4) = I_12x12;
  C_Matrix_Trans.block(24*m-24+12,3*(m+3),3*4,3*4) = I_12x12;
  // cout << "[Debug] inside PolyQPGeneration, 666" << endl; // for test
  for(int i = 0;i<m;i++){
  double tm = Time(i);
  // cout << "[Debug] inside polynomial trajectory, constructing Qi" << endl; // for test
  //Construct Qi:
  MatrixXd Q_i = MatrixXd::Zero(3 * p_num1d, 3 * p_num1d);
  // Q_i.block(21,21,3,3) = 705600*pow(tm,6)*One_3x3/pow(tm,6);
  // Q_i.block(21,18,3,3) = 302400*pow(tm,5)*One_3x3/pow(tm,5);
  // Q_i.block(21,15,3,3) = 100800*pow(tm,4)*One_3x3/pow(tm,4);
  // Q_i.block(21,12,3,3) = 20160*pow(tm,3)*One_3x3/pow(tm,3);

  // Q_i.block(18,21,3,3) = 302400*pow(tm,5)*One_3x3/pow(tm,5);
  // Q_i.block(18,18,3,3) = 129600*pow(tm,4)*One_3x3/pow(tm,4);
  // Q_i.block(18,15,3,3) = 43200*pow(tm,3)*One_3x3/pow(tm,3);
  // Q_i.block(18,12,3,3) = 8640*pow(tm,2)*One_3x3/pow(tm,2);

  // Q_i.block(15,21,3,3) = 100800*pow(tm,4)*One_3x3/pow(tm,4);
  // Q_i.block(15,18,3,3) = 43200*pow(tm,3)*One_3x3/pow(tm,3);
  // Q_i.block(15,15,3,3) = 14400*pow(tm,2)*One_3x3/pow(tm,2);
  // Q_i.block(15,12,3,3) = 2880*pow(tm,1)*One_3x3/pow(tm,1);

  // Q_i.block(12,21,3,3) = 20160*pow(tm,3)*One_3x3/pow(tm,3);
  // Q_i.block(12,18,3,3) = 8640*pow(tm,2)*One_3x3/pow(tm,2);
  // Q_i.block(12,15,3,3) = 2880*pow(tm,1)*One_3x3/pow(tm,1);
  // Q_i.block(12,12,3,3) = 576*One_3x3;
  Q_i = beta(tm,4,p_num1d).transpose()*beta(tm,4,p_num1d);

  // 添加正则化
  // Q_i.diagonal().array() += lambda;

  // 或者对高阶项使用更大的正则化系数
  for(int i = 0; i < p_num1d; i++) {
      double reg = lambda * pow(10, i);  // 高阶项用更大的正则化系数
      Q_i.block(3*i,3*i,3,3).diagonal().array() += reg;
  }



  // cout<<"Q_i= "<<Q_i<<endl; // for test
  // Q_i <<  705600*pow(Time(i),6)*I_3x3, 302400*pow(Time(i),5)*I_3x3, 100800*pow(Time(i),4)*I_3x3, 20160*pow(Time(i),3)*I_3x3,
  //         302400*pow(Time(i),5)*I_3x3, 129600*pow(Time(i),4)*I_3x3, 43200*pow(Time(i),3)*I_3x3, 8640*pow(Time(i),2)*I_3x3,
  //         100800*pow(Time(i),4)*I_3x3, 43200*pow(Time(i),3)*I_3x3, 14400*pow(Time(i),2)*I_3x3, 2880*pow(Time(i),1)*I_3x3,
  //         20160*pow(Time(i),3)*I_3x3,  8640*pow(Time(i),2)*I_3x3,  2880*pow(Time(i),1)*I_3x3,  576;
  // cout << "[Debug] inside polynomial trajectory, constructing Q_Matrix" << endl; // for test
  Q_Matrix.block(3*p_num1d*i, 3*p_num1d*i, 3*p_num1d, 3*p_num1d) = Q_i;
  // cout << "[Debug] inside polynomial trajectory, constructing Ai" << endl; // for test
  //Construct Ai:
  MatrixXd A_i = MatrixXd::Zero(3 * p_num1d, 3 * p_num1d);
  // A_i.block(0,3 * (p_num1d-1),3,3) = 1*I_3x3;
  // A_i.block(3,3 * (p_num1d-2),3,3) = 1*I_3x3;
  // A_i.block(6,3 * (p_num1d-3),3,3) = 2*I_3x3;
  // A_i.block(9,3 * (p_num1d-4),3,3) = 6*I_3x3;

  // A_i.block(12,3 * (p_num1d-1),3,3) = 1*I_3x3;

  A_i <<  1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3,
          0*I_3x3, 1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3,
          0*I_3x3, 0*I_3x3, 2*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3,
          0*I_3x3, 0*I_3x3, 0*I_3x3, 6*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3,
          1*I_3x3, pow(tm,1)*I_3x3, pow(tm,2)*I_3x3, pow(tm,3)*I_3x3, pow(tm,4)*I_3x3, pow(tm,5)*I_3x3, pow(tm,6)*I_3x3, pow(tm,7)*I_3x3,
          0*I_3x3, pow(tm,0)*I_3x3, 2*pow(tm,1)*I_3x3, 3*pow(tm,2)*I_3x3, 4*pow(tm,3)*I_3x3, 5*pow(tm,4)*I_3x3, 6*pow(tm,5)*I_3x3, 7*pow(tm,6)*I_3x3,
          0*I_3x3, 0*I_3x3, 2*pow(tm,0)*I_3x3, 6*pow(tm,1)*I_3x3, 12*pow(tm,2)*I_3x3, 20*pow(tm,3)*I_3x3, 30*pow(tm,4)*I_3x3, 42*pow(tm,5)*I_3x3,
          0*I_3x3, 0*I_3x3, 0*I_3x3, 6*pow(tm,0)*I_3x3, 24*pow(tm,1)*I_3x3, 60*pow(tm,2)*I_3x3, 120*pow(tm,3)*I_3x3, 210*pow(tm,4)*I_3x3;
          // pow(tm,7)*I_3x3, pow(tm,6)*I_3x3, pow(tm,5)*I_3x3, pow(tm,4)*I_3x3, pow(tm,3)*I_3x3, pow(tm,2)*I_3x3, pow(tm,1)*I_3x3, 1*I_3x3,
          // 7*pow(tm,6)*I_3x3, 6*pow(tm,5)*I_3x3, 5*pow(tm,4)*I_3x3, 4*pow(tm,3)*I_3x3, 3*pow(tm,2)*I_3x3, 2*pow(tm,1)*I_3x3, pow(tm,0)*I_3x3, 0*I_3x3,
          // 42*pow(tm,5)*I_3x3, 30*pow(tm,4)*I_3x3, 20*pow(tm,3)*I_3x3, 12*pow(tm,2)*I_3x3, 6*pow(tm,1)*I_3x3, 2*pow(tm,0)*I_3x3, 0*I_3x3, 0*I_3x3,
          // 210*pow(tm,4)*I_3x3, 120*pow(tm,3)*I_3x3, 60*pow(tm,2)*I_3x3, 24*pow(tm,1)*I_3x3, 6*pow(tm,0)*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3;
  // cout << "[Debug] inside polynomial trajectory, constructing A_Matrix" << endl; // for test
  A_Matrix.block(3*p_num1d*i, 3*p_num1d*i, 3*p_num1d, 3*p_num1d) = A_i; 
  // cout<< "A_i = " << A_i <<endl; // for test 
  
  // cout << "[Debug] inside polynomial trajectory, constructing C_Matrix"<<"m:"<<m<<"i:"<<i << endl; // for test
  //Construct C_Matrix: //24mx12m+12
  if(i>0){
    C_Matrix_Trans.block(24*i+0+ 0*3 +0, 12+ 3*(i-1) +0, 3, 3) = I_3x3; // d_{i,0}^{0},x/y/z
    C_Matrix_Trans.block(24*i+0+ 1*3 +0, 3*(m+7)+9*(i-1)+ 0*3 +0, 3, 3) = I_3x3; // d_{i,0}^{1},x/y/z
    C_Matrix_Trans.block(24*i+0+ 2*3 +0, 3*(m+7)+9*(i-1)+ 1*3 +0, 3, 3) = I_3x3; // d_{i,0}^{2},x/y/z
    C_Matrix_Trans.block(24*i+0+ 3*3 +0, 3*(m+7)+9*(i-1)+ 2*3 +0, 3, 3) = I_3x3; // d_{i,0}^{3},x/y/z
    // C_Matrix.block(24*i+0+ 4*3 +0, 3*(m+7)+9*(i-1)+ 4*3 +0, 3, 3) = I_3x3; // d_{i,0}^{4},x/y/z
  }
  if(i<m-1){
    C_Matrix_Trans.block(24*i+12+ 0*3 +0, 12+ 3*i +0, 3, 3) = I_3x3; // d_{i,T}^{0},x/y/z
    C_Matrix_Trans.block(24*i+12+ 1*3 +0, 3*(m+7)+9*i+ 0*3 +0, 3, 3) = I_3x3; // d_{i,T}^{1},x/y/z
    C_Matrix_Trans.block(24*i+12+ 2*3 +0, 3*(m+7)+9*i+ 1*3 +0, 3, 3) = I_3x3; // d_{i,T}^{2},x/y/z
    C_Matrix_Trans.block(24*i+12+ 3*3 +0, 3*(m+7)+9*i+ 2*3 +0, 3, 3) = I_3x3; // d_{i,T}^{3},x/y/z
    // C_Matrix.block(24*i+12+ 4*3 +0, 3*(m+7)+9*i+ 4*3 +0, 3, 3) = I_3x3; // d_{i,T}^{4},x/y/z
  }
  }
  //C A^T Q A^-1 C^T = R
  // cout << "[Debug] inside polynomial trajectory, constructing C_Trans" << endl; // for test
  MatrixXd C_Matrix = C_Matrix_Trans.transpose();
  // cout << "[Debug] inside polynomial trajectory, constructing A_inv" << endl; // for test
  MatrixXd A_inv(A_Matrix.rows(), A_Matrix.cols());
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A_Matrix);
  A_inv = lu.inverse();
  cout<<"A_inv size: "<<A_inv.size()<< endl;
  // cout<<"A_inv = "<<A_inv<<endl; // for test
  // cout << "[Debug] inside polynomial trajectory, constructing A_invTrans" << endl; // for test
  MatrixXd A_invTrans = A_inv.transpose(); // selfadd:matrix inverse note!
  cout << "[Debug] inside polynomial trajectory, constructing R_Matrix" << endl; // for test
  MatrixXd R_Matrix = C_Matrix*A_invTrans*Q_Matrix*A_inv*C_Matrix_Trans;
  cout<<"R_Matrix: done"<<endl; // for test

  //dP∗=− RPP^−1 * RFP^T *dF
  cout << "[Debug] inside polynomial trajectory, constructing dF" << endl; // for test
  MatrixXd R_fp = R_Matrix.block(0,3*m+21,3*m+21,9*m-9);
  MatrixXd R_pp = R_Matrix.block(3*m+21,3*m+21,9*m-9,9*m-9);
  cout<<"R_pp: done"<<endl; // for test

  MatrixXd Rpp_inv(R_pp.rows(), R_pp.cols());
  Eigen::FullPivLU<Eigen::MatrixXd> lu_2(R_pp);
  Rpp_inv = lu_2.inverse();
  // MatrixXd Rpp_inv = R_pp.inverse();

  MatrixXd Rfp_trans = R_fp.transpose();
  cout<<"Rpp_inv: done "<<endl; // for test
  cout<<"Rfp_trans: done"<<endl; // for test

  VectorXd dF = VectorXd::Zero(3*(m+7));
  cout<<"dF = VectorXd::Zero(3*(m+7));"<<endl; //for test
  // fill in dF 
  dF.block(0,0,3,1) = Path.row(0).transpose(); //Position0
  cout<<"dF = Position0"<<endl; //for test
  cout<<"3*(m+3)= "<<3*(m+3)<<"m"<<m<<endl; //for test
  dF.block(3*(m+3),0,3,1) = Path.row(m).transpose(); //PositionM
  cout<<"dF = PositionM"<<endl; //for test
  for(int j =1;j<m;j++){
    cout<<"j= "<<j<<endl; //for test
    dF.block(12+3*(j-1),0,3,1) =Path.row(j).transpose(); //position condition(except p0 and pM)
  }
  cout<<"dF: "<<dF; // for test
  // cout << "[Debug] inside polynomial trajectory, constructing dP_star" << endl; // for test
  VectorXd dP_star = -1*Rpp_inv*Rfp_trans*dF;
  cout<<"dP_star: done"<<endl; // for test
  //P = A^-1*C^T*[dp,df]^T
  // cout << "[Debug] inside polynomial trajectory, constructing dPF" << endl; // for test
  VectorXd dPF(12*m+12); 
  dPF << dF,dP_star;
  // cout<<"dPF:"<<dPF<<endl; // for test

  VectorXd D;
  D = C_Matrix_Trans*dPF;
  cout<<"D_Matrix = done"<<endl;

  // cout << "[Debug] inside polynomial trajectory, constructing PolyCoeff" << endl; // for test
  PolyCoeff = A_inv*C_Matrix_Trans*dPF;
  // cout<<"Polycoeff  before resize:"<<PolyCoeff<<endl; // for test
  // PolyCoeff.resize(m, 3 * p_num1d);
  // MatrixXd PolyCoeff_resize = MatrixXd::Zero(3, 3);
  // Map<MatrixXd, RowMajor> row_major(PolyCoeff.data(), PolyCoeff.rows(), PolyCoeff.cols());
  // PolyCoeff.conservativeResize(m, 3 * p_num1d);

  //check continuous
  // VectorXd D_2;
  // D_2 = A_Matrix*PolyCoeff;
  // cout<<"D_2-D = "<<D_2-D<<endl;

  // Create new matrix with desired size
  MatrixXd PolyCoeff_resized(m, 3 * p_num1d);

  // Copy data in row-major order
  // for(int s = 0; s <  m; ++s) {
  //   for(int k = 0; k < 3 * p_num1d; ++k) { // modified: std::min(PolyCoeff.cols(), 3 * p_num1d)
  //     try{
  //       PolyCoeff_resized(s,k) = PolyCoeff(s*3 * p_num1d+k);
  //     }catch (const std::exception& e) {
  //     cout<<"resize failed.. i,j= "<< s<<" "<<k<<endl;
  //     }
  //   } 
  // }
  for(int s = 0; s < m; s++) {
    for(int dim = 0; dim < 3; dim++) {
        for(int j = 0; j < p_num1d; j++) {
          // cout<<"s,j,dim= "<<s<<j<<dim<<endl;
            PolyCoeff_resized(s, dim * p_num1d + j) = PolyCoeff(3 * s * p_num1d + j * 3 + dim);
        }
    }
  }
  cout<<"Polycoeff:"<<PolyCoeff_resized<<endl; // for test

  // 在得到PolyCoeff_resized后添加 for test
  for(int s = 0; s < m; s++) {
      std::cout << "段 " << s << " 的系数：\n";
      for(int dim = 0; dim < 3; dim++) {
          std::cout << "维度 " << dim << ": ";
          for(int j = 0; j < p_num1d; j++) {
              std::cout << PolyCoeff_resized(s, dim * p_num1d + j) << " ";
          }
          std::cout << "\n";
      }
  }

 // testing continuousty
  for(int i = 0; i < m-1; i++) {
    // 检查段i结束点和段i+1起始点的连续性
    Vector3d pos_end = getPosPoly(PolyCoeff_resized, i, Time(i));
    Vector3d pos_start = getPosPoly(PolyCoeff_resized, i+1, 0);
    Vector3d vel_end = getVelPoly(PolyCoeff_resized, i, Time(i));
    Vector3d vel_start = getVelPoly(PolyCoeff_resized, i+1, 0);
    Vector3d acc_end = getAccPoly(PolyCoeff_resized, i, Time(i));
    Vector3d acc_start = getAccPoly(PolyCoeff_resized, i+1, 0);
    
    std::cout << "段 " << i << " 和 " << i+1 << " 的连接点：\n";
    std::cout << "位置差： " << (pos_end - pos_start).norm() << "\n";
    std::cout << "速度差： " << (vel_end - vel_start).norm() << "\n";
    std::cout << "加速度差： " << (acc_end - acc_start).norm() << "\n";
}

// 检查A_Matrix的结构
std::cout << "A_Matrix 的条件数：" << 
    A_Matrix.jacobiSvd().singularValues()(0) / 
    A_Matrix.jacobiSvd().singularValues()(A_Matrix.jacobiSvd().singularValues().size()-1) 
    << "\n";

// // 检查Q_Matrix的结构
// std::cout << "Q_Matrix 非零元素的位置：\n";
// for(int i = 0; i < Q_Matrix.rows(); i++) {
//     for(int j = 0; j < Q_Matrix.cols(); j++) {
//         if(abs(Q_Matrix(i,j)) > 1e-10) {
//             std::cout << "(" << i << "," << j << "): " << Q_Matrix(i,j) << "\n";
//         }
//     }
// }

// 检查R_Matrix的结构
std::cout << "R_fp的维度: " << R_fp.rows() << "x" << R_fp.cols() << "\n";
std::cout << "R_pp的维度: " << R_pp.rows() << "x" << R_pp.cols() << "\n";
std::cout << "R_pp的条件数: " << 
    R_pp.jacobiSvd().singularValues()(0) / 
    R_pp.jacobiSvd().singularValues()(R_pp.jacobiSvd().singularValues().size()-1) 
    << "\n";


  return PolyCoeff_resized;
}
//********************************************************************************************** */
Eigen::MatrixXd TrajectoryGeneratorWaypoint::CorridorMinsnapSolve(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d) //selfadd:MatrixXd path(int(grid_path.size()), 3); path.row(k) = grid_path[k]; 
    const double Vel,  // boundary velocity
    const double Acc,  // boundary acceleration
    const Eigen::VectorXd &Time, // time allocation in each segment
    const std::vector<Eigen::Matrix<double, 6, -1>> &corridors) // corridor
{
  cout << "[Debug] inside PolyQPGeneration, just start" << endl; // for test
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = N_ORDER + 1;     // the number of variables in each segment
  // cout << "[Debug] inside PolyQPGeneration, 111" << endl; // for test
  int m = Time.size();
  cout << "[Debug] m segments:" << m << endl; // for test
  cout << "[Debug] path size:" << Path.rows() << endl; // for test
  cout <<"[Debug] Path:"<< Path << endl; // for test
  MatrixXd PolyCoeff(m, 3 * p_num1d); //每行是一段polynomial，一行内，三个三个一组(x，y，z)，一共p_num1d组
  
  double vmax = Vel;
  double amax = Acc;

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  //get C,A,Q
  cout << "[Debug] inside PolyQPGeneration, 333" << endl; // for test
  Matrix3d I_3x3 = Matrix3d::Identity();
  MatrixXd I_12x12 = MatrixXd::Identity(12,12);
  MatrixXd One_3x3 = MatrixXd::Ones(3,3);
  double lambda = 1e-2;  // 正则化参数 origin:1e-8
  cout << "[Debug] inside PolyQPGeneration, 444" << endl; // for test
  // MatrixXd Q_Matrix = MatrixXd::Zero(3*p_num1d*m,3*p_num1d*m);// define size? //24mx24m
  _Q.resize(DIM*p_num1d*m,DIM*p_num1d*m);
  _Q.setZero();

  int  n_hyperplanes = 0;
  for (int i = 0; i < corridors.size() - 1; i++) {
    int c_prev = corridors[i].cols();
    int c_next = corridors[i + 1].cols();
    n_hyperplanes += c_prev;
    n_hyperplanes += c_next;
  }
  int N = 21*m+9+n_hyperplanes;
  int W = DIM*p_num1d*(m+1);
  _A.resize(N,W); //[(15m+9)+(6m-3)+96m] x24m = 107m+6 x24m  // 24+15(m-1)=15m+9,15m+9+6m=21m+9
  _A.setZero();
  // MatrixXd A_Equal = MatrixXd::Zero(3*5*m+9,3*p_num1d*m); //15m+9x24m
  // MatrixXd A_Inequal = MatrixXd::Zero(6*m-3,3*p_num1d*m); //6m-3x24m
  // MatrixXd A_Corridor; //96mx24m
  _ub.resize(N);
  _ub.setZero();
  _lb.resize(N);
  _lb.setZero();

  cout << "[Debug] inside PolyQPGeneration, 555" << endl; // for test

  Eigen::Matrix<double, 1, Par_number> pos_1d;
  Eigen::Matrix<double, 1, Par_number> vel_1d;
  Eigen::Matrix<double, 1, Par_number> acc_1d;
  Eigen::Matrix<double, 1, Par_number> jer_1d;
  pos_1d << 1, 1, 1, 1, 1, 1, 1, 1;
  vel_1d << 0, 1, 2, 3, 4, 5, 6, 7;
  acc_1d << 0, 0, 2, 6, 12, 20, 30, 42;
  jer_1d << 0, 0, 0, 6, 24, 60, 120, 210;

  //At t=0,M constrains
  //equality constrain A0
  double t0 = Time(0);
  // // cout << "[Debug] inside PolyQPGeneration, after t0 = Time(0);" << endl; // for test
  // _A.block(0, 0, 3, 24) << I_3x3, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
  //                        Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(); //A_0(seg idx)_0_0(degree) (P0)
  // // cout << "[Debug] inside PolyQPGeneration, after _A.block(0, 0, 3, 24)" << endl; // for test
  // _A.block(3, 0, 3, 24) << Eigen::Matrix3d::Zero(), I_3x3, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(); //A_0(seg idx)_0_0(degree) (P0)
  // _A.block(6, 0, 3, 24) << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), 2*I_3x3, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(); //A_0(seg idx)_0_0(degree) (P0)
  // _A.block(9, 0, 3, 24) << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), 6*I_3x3, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(); //A_0(seg idx)_0_0(degree) (P0)
  // _A.block(12+0, 0, 3, 24) = beta(t0,0,p_num1d); //A_0_t0_0 (P1) 
  // _A.block(12+3, 0, 3, 24) = beta(t0,1,p_num1d); //A_0_t0_1 (P1)
  // _A.block(12+6, 0, 3, 24) = beta(t0,2,p_num1d); //A_0_t0_2 (P1)
  // _A.block(24+9, 0, 3, 24) = beta(t0,3,p_num1d); //A_0_t0_3 (P1)
  // _A.block(12+0, 24, 3, 24) << -1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_1_0_0 (P1)
  // _A.block(12+3, 24, 3, 24) << 0*I_3x3, -1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_1_0_1 (P1)
  // _A.block(12+6, 24, 3, 24) << 0*I_3x3, 0*I_3x3, -2*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_1_0_2 (P1)
  // _A.block(12+9, 24, 3, 24) << 0*I_3x3, 0*I_3x3, 0*I_3x3, -6*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_1_0_3 (P1)   
  // // cout << "[Debug] inside PolyQPGeneration, equality constrain A0" << endl; // for test
  
  //equality constrain A0
  for(int dim=0;dim<DIM;dim++){
    cout<<"dim= "<<dim<<endl;
    // Derivative constrain For Position0
    _A(0+dim,dim*8+0) = 1; //p
    _A(3+dim,dim*8+1) = 1; //v
    _A(6+dim,dim*8+2) = 2; //a
    _A(9+dim,dim*8+3) = 6;  //j
    cout<<"P0 pvaj done"<<endl;
    // Continuous constrain for piece0
    // A0(t0)
    _A.block(12+0 + dim, dim * 8, 1, p_num1d) = pos_1d;//Position1=piece0(t0)
    _A.block(12+3 + dim, dim * 8, 1, p_num1d) = vel_1d / t0;
    _A.block(12+6 + dim, dim * 8, 1, p_num1d) = acc_1d /t0 / t0;
    _A.block(12+9 + dim, dim * 8, 1, p_num1d) = jer_1d / t0 / t0 / t0;
    cout<<"A0(0) done"<<endl;
    // -A1(0)
    if (m>1){
      double t1 = Time(1);
      _A(12+0 + dim, 24+ dim * 8 +0) = -1;
      cout<<"aaa"<<endl;
      _A(12+3 + dim, 24+ dim * 8 +1) = -1/t1;
      cout<<"bbb"<<endl;
      _A(12+6 + dim, 24+ dim * 8 +2) = -2 / t1 / t1;
      cout<<"ccc"<<endl;
      _A(12+9 + dim, 24+ dim * 8 +3) = -6 / t1 / t1 / t1;
      cout<<"A0(t0) done"<<endl;
    }

  }
  _ub.block(0,0,3,1) = Path.row(0).transpose(); //Position0
  _lb.block(0,0,3,1) = Path.row(0).transpose(); //Position0 // same as _ub
  if(m==1){
    _ub.block(12+0,0,3,1) = Path.row(1).transpose(); //Position0
    _lb.block(12+0,0,3,1) = Path.row(1).transpose(); //Position0 // same as _ub
  }
  
  cout << "[Debug] inside PolyQPGeneration, equality constrain AM" << endl; // for test
  if(m>1){
  //equality constrain AM
    double tM = Time(m-1);
    for(int dim=0;dim<DIM;dim++){
      //dirivative constrain for
      _A(24+15*(m-2)+dim, (m-1)*24+dim*8+0) = 1; //AM-1(0)_0 (PM-1)
      cout << "[Debug] inside PolyQPGeneration, equality constrain PM=piecem(tM)" << endl; // for test
      //continuous constrain but acctually constrain for PM=piecem(tM)
      _A.block(24+15*(m-2)+3+0+dim,(m-1)*24+dim*8,1,p_num1d) = pos_1d;
      _A.block(24+15*(m-2)+3+3+dim,(m-1)*24+dim*8,1,p_num1d) = vel_1d / tM;
      _A.block(24+15*(m-2)+3+6+dim,(m-1)*24+dim*8,1,p_num1d) = acc_1d /tM / tM;
      _A.block(24+15*(m-2)+3+9+dim,(m-1)*24+dim*8,1,p_num1d) = jer_1d / tM / tM / tM;
      // cout << "[Debug] inside PolyQPGeneration, equality constrain lbub" << endl; // for test
      //inequality constrain A_inequal_0
      // _A.block(15*m+9 + 0,0,3,24) << Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero();
      // cout << "[Debug] inside PolyQPGeneration, inequality constrain A0" << endl; // for test
    }
    _ub.block(24+15*(m-2), 0, 3, 1) = Path.row(m-1).transpose(); //PositionM-1
    _lb.block(24+15*(m-2), 0, 3, 1) = Path.row(m-1).transpose(); //PositionM-1 // same as _ub
    _ub.block(24+15*(m-2)+3, 0, 3, 1) = Path.row(m).transpose(); //PositionM
    _lb.block(24+15*(m-2)+3, 0, 3, 1) = Path.row(m).transpose(); //PositionM
  }
  cout << "[Debug] inside polynomial trajectory, constructing Q_Matrix" << endl; // for test
  //Construct Qi:
  cout << "[Debug] inside polynomial trajectory, constructing Qi" << endl; // for test
  MatrixXd Q_i = MatrixXd::Zero(p_num1d, p_num1d);
  for (int k = 0; k <p_num1d; k++) {
    for (int q = 0; q <p_num1d; q++) {
      if (k < 4 || q < 4) {
        Q_i(k, q) = 0;
      }
      if (k >= 4 && q >= 4 && (k + q > p_order)) {
        Q_i(k, q) = k * (k - 1) * (k - 2) * (k - 3) * q * (q - 1) * (q - 2) *
                  (q - 3) / (k + q - p_order);
      }
    }
  }

  //正则化
  for(int i = 0; i < p_num1d; i++) {
      double reg = lambda * pow(2, i);  // 高阶项用更大的正则化系数
      Q_i(i,i) = Q_i(i,i)+reg;
  }

  cout<<"Q_i insert"<<Q_i<<std::endl ;
  /* for all dimensions and all pieces */
  for (int i = 0; i < m * DIM; i++) {
  _Q.block(i * Par_number, i * Par_number, Par_number, Par_number) = Q_i;
  // _Q.block(i * Par_number, i * Par_number, Par_number, Par_number) = Q_i;
  // _Q.block(i * Par_number, i * Par_number, Par_number, Par_number) = Q_i;
  // _Q.block(i * D, i * D, D, D) = Q_i;
  }

  for(int i = 0;i<m;i++){
  double tm = Time(i);
  

  // Q_i.block(21,21,3,3) = 705600*pow(tm,7)*I_3x3*(1/7);// /pow(tm,6)
  // Q_i.block(21,18,3,3) = 302400*pow(tm,6)*I_3x3*(1/6);// /pow(tm,5)
  // Q_i.block(21,15,3,3) = 100800*pow(tm,5)*I_3x3*(1/5);// /pow(tm,4)
  // Q_i.block(21,12,3,3) = 20160*pow(tm,4)*I_3x3*(1/4);// /pow(tm,3)

  // Q_i.block(18,21,3,3) = 302400*pow(tm,6)*I_3x3*(1/6);
  // Q_i.block(18,18,3,3) = 129600*pow(tm,5)*I_3x3*(1/5);
  // Q_i.block(18,15,3,3) = 43200*pow(tm,4)*I_3x3*(1/4);
  // Q_i.block(18,12,3,3) = 8640*pow(tm,3)*I_3x3*(1/3);

  // Q_i.block(15,21,3,3) = 100800*pow(tm,5)*I_3x3*(1/5);
  // Q_i.block(15,18,3,3) = 43200*pow(tm,4)*I_3x3*(1/4);
  // Q_i.block(15,15,3,3) = 14400*pow(tm,3)*I_3x3*(1/3);
  // Q_i.block(15,12,3,3) = 2880*pow(tm,2)*I_3x3*(1/2);

  // Q_i.block(12,21,3,3) = 20160*pow(tm,4)*I_3x3*(1/4);
  // Q_i.block(12,18,3,3) = 8640*pow(tm,3)*I_3x3*(1/3);
  // Q_i.block(12,15,3,3) = 2880*pow(tm,2)*I_3x3*(1/2);
  // Q_i.block(12,12,3,3) = 576*pow(tm,1)*I_3x3;


  // 添加正则化
  // Q_i.diagonal().array() += lambda;

  // 或者对高阶项使用更大的正则化系数


  // cout<<"Q_i= "<<Q_i<<endl; // for test
  
  // _Q.block(3*p_num1d*i, 3*p_num1d*i, 3*p_num1d, 3*p_num1d) = Q_i;
  // cout << "[Debug] inside polynomial trajectory, constructing Ai" << endl; // for test

  // //Construct equality constrains 
  // if(i>0 && i<m-1){
  //   cout << "[Debug] inside polynomial trajectory, before equality constrains in _A" << endl; // for test
  //   //equality constrains in _A
  //   // MatrixXd A_i = MatrixXd::Zero(3 * p_num1d, 3 * p_num1d); //!!!!!!!!!!!!
  //   //position constrain
  //   _A.block(24+15*(i-1)+0, 24+24*(i-1), 3, 24) << 1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i(seg idx)_0_0(degree)
  //   //continous constrain
  //   _A.block(24+15*(i-1)+3, 24+24*(i-1), 3, 24) = beta(tm,0,p_num1d); //A_i_ti_0
  //   _A.block(24+15*(i-1)+6, 24+24*(i-1), 3, 24) = beta(tm,1,p_num1d); //A_i_ti_1
  //   _A.block(24+15*(i-1)+9, 24+24*(i-1), 3, 24) = beta(tm,2,p_num1d); //A_i_ti_2
  //   _A.block(24+15*(i-1)+12, 24+24*(i-1), 3, 24) = beta(tm,3,p_num1d); //A_i_ti_3

  //   // 段终点的位置约束
  //   // _A.block(24+15*(i-1)+3, 24*i, 3, 24) = beta(tm,0,p_num1d); //A_i_ti_0 //added
  //   _A.block(24+15*(i-1)+3, 24+24*(i-1)+24, 3, 24) << -1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_0
  //   _A.block(24+15*(i-1)+6, 24+24*(i-1)+24, 3, 24) << 0*I_3x3, -1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_1
  //   _A.block(24+15*(i-1)+9, 24+24*(i-1)+24, 3, 24) << 0*I_3x3, 0*I_3x3, -2*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_2
  //   _A.block(24+15*(i-1)+12, 24+24*(i-1)+24, 3, 24) << 0*I_3x3, 0*I_3x3, 0*I_3x3, -6*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_3    
  // //   cout << "[Debug] inside polynomial trajectory, before equality constrains in _lb _ub" << endl; // for test
  //   //equality constrains in _lb _ub
  //   _ub.block(24+15*(i-1)+0, 0, 3, 1) = Path.row(i).transpose(); //Positioni 
  //   _lb.block(24+15*(i-1)+0, 0, 3, 1) = Path.row(i).transpose(); //Positioni // same as _ub
  //   // 段终点的位置约束
  //   // _ub.block(24+15*(i-1)+3, 0, 3, 1) = Path.row(i+1).transpose();
  //   // _lb.block(24+15*(i-1)+3, 0, 3, 1) = Path.row(i+1).transpose();
  // }
  cout<<"now i= "<<i<<std::endl;
  if(i>0 && i<m-1){ 
      int row_idx = 24+15*(i-1); 
      int col_idx = 24*i; 
      //position constrain
      for (int dim = 0; dim < DIM; dim++) {
      _A(row_idx + dim, col_idx +dim * 8+0) = 1;
      }
      cout<<"111 "<<i<<std::endl;
      _ub.block(row_idx, 0, 3, 1) = Path.row(i).transpose();
      _lb.block(row_idx, 0, 3, 1) = Path.row(i).transpose();
      cout<<"222"<<i<<std::endl;
      //continuous constrain
      for (int dim = 0; dim < DIM; dim++) {
      _A.block(row_idx+3 + dim, col_idx + dim * 8, 1, p_num1d) = pos_1d;//row_idx+3, col_idx, 3, 24
      _A.block(row_idx+6 + dim, col_idx + dim * 8, 1, p_num1d) = vel_1d / tm;
      _A.block(row_idx+9 + dim, col_idx + dim * 8, 1, p_num1d) = acc_1d /tm / tm;
      _A.block(row_idx+12 + dim, col_idx + dim * 8, 1, p_num1d) = jer_1d / tm / tm / tm;
      cout<<"333, dim= "<<dim<<std::endl;
      _A(row_idx+3 + dim, col_idx + DIM*p_num1d +dim * 8+0) = -1;
      _A(row_idx+6 + dim, col_idx + DIM*p_num1d +dim * 8+1) = -1 / Time(i+1);
      _A(row_idx+9 + dim, col_idx + DIM*p_num1d +dim * 8+2) = -2 / Time(i+1) / Time(i+1);
      _A(row_idx+12 + dim, col_idx + DIM*p_num1d +dim * 8+3) = -6 / Time(i+1) / Time(i+1) / Time(i+1);
      cout<<"444, dim= "<<dim<<std::endl;
      //continous constrain _lb,_up bound is 0
      }  
      cout<<"555"<<std::endl;
    // int row_idx = 24+15*(i-1); //54
    // int col_idx = 24*i; //72
    
    // // 位置约束 //54，72
    // _A.block(row_idx, col_idx, 3, 24) << I_3x3, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
    //                                     Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero();
    // // cout << "11111" << endl; // for test

    // // 连续性约束
    // // 位置连续性 //57，72
    // _A.block(row_idx+3, col_idx, 3, 24) = beta(tm,0,p_num1d);
    // _A.block(row_idx+3, col_idx+24, 3, 24) << -1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_0
    // // cout << "22222" << endl; // for test
    // // 速度连续性 //60， 72
    // _A.block(row_idx+6, col_idx, 3, 24) = beta(tm,1,p_num1d);
    // _A.block(row_idx+6, col_idx+24, 3, 24) << 0*I_3x3, -1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_1
    // // cout << "33333" << endl; // for test
    // // 加速度连续性 63， 72
    // _A.block(row_idx+9, col_idx, 3, 24) = beta(tm,2,p_num1d);
    // _A.block(row_idx+9, col_idx+24, 3, 24) << 0*I_3x3, 0*I_3x3, -2*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_2
    // // cout << "44444" << endl; // for test
    // // // 跃度连续性 66，72
    // // _A.block(row_idx+12, col_idx, 3, 24) = beta(tm,3,p_num1d);
    // // _A.block(row_idx+12, col_idx+24, 3, 24) << 0*I_3x3, 0*I_3x3, 0*I_3x3, -6*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i+1_0_3
    // // // cout << "55555" << endl; // for test
    // // 设置约束边界
    // _ub.block(row_idx, 0, 3, 1) = Path.row(i).transpose();
    // _lb.block(row_idx, 0, 3, 1) = Path.row(i).transpose();
    
    // // 连续性约束的边界都应该是0（因为差值应该为0）
    // _ub.block(row_idx+3, 0, 12, 1).setZero();
    // _lb.block(row_idx+3, 0, 12, 1).setZero();

  //   // 测试代码   
  //   // 在赋值之前打印更多信息
  //   std::cout << "Left side size (_A.block): " 
  //             << _A.block(row_idx+3, col_idx+24, 3, 24).rows() << " x "
  //             << _A.block(row_idx+3, col_idx+24, 3, 24).cols() << std::endl;

  //   std::cout << "Right side size (beta): "
  //             << beta(0, 0, p_num1d).rows() << " x "
  //             << beta(0, 0, p_num1d).cols() << std::endl;

  //   std::cout << "row_base = " << row_idx << ", col_base = " << col_idx << std::endl;
  //   std::cout << "_A size = " << _A.rows() << " x " << _A.cols() << std::endl;
  //   MatrixXd test_constraint = MatrixXd::Zero(3, 24);
  //   test_constraint << I_3x3, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
  //                     Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero();
  //   std::cout << "测试约束矩阵大小: " << test_constraint.rows() << " x " << test_constraint.cols() << std::endl;

  //   MatrixXd beta_result = beta(Time(i), 0, p_num1d);
  //   std::cout << "beta结果矩阵大小: " << beta_result.rows() << " x " << beta_result.cols() << std::endl;
  //   if (row_idx+3+3 > _A.rows() || col_idx+24+24 > _A.cols()) {
  //   std::cout << "Warning: Indices out of bounds!" << std::endl;
  //   std::cout << "Attempting to access up to row " << row_idx+3+3 
  //             << " and col " << col_idx+24+24 << std::endl;
  //   }
  }

  
  // // cout << "[Debug] inside polynomial trajectory, before Construct inequality constrains for vmax amax" << endl; // for test
  // //Construct inequality constrains for vmax amax
      //inequality contrains in A_Matrix:
    // _A.block(15*m+9 + 3+6*(i-1), 24*i,3,24) << 0*I_3x3, 1*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i_0_1
    // _A.block(15*m+9 + 3+6*(i-1)+3, 24*i,3,24) << 0*I_3x3, 0*I_3x3, 2*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3; //A_i_0_2 //debug -
  for (int dim = 0; dim < DIM; dim++) {
      _A.block(15*m+9 +6*i +dim, 24*i + dim * 8, 1, p_num1d) = vel_1d / tm;
      _A.block(15*m+9 +6*i+3 +dim, 24*i + dim * 8, 1, p_num1d) = acc_1d /tm / tm;
    //inequality contrains in _ub _lb:
    _ub.segment(15*m+9 +6*i +dim,3) << vmax,vmax,vmax;  //vi 
    _ub.segment(15*m+9 +6*i +dim,3) << amax,amax,amax;  //ai
    _lb.segment(15*m+9 +6*i +dim,3) << -vmax,-vmax,-vmax;  //vi 
    _lb.segment(15*m+9 +6*i +dim,3) << -amax,-amax,-amax;  //ai
  }
  }
  // }
  
  
  // //数值稳定性
  // for(int i = 0; i < _A.rows(); i++) {
  //   double row_norm = _A.row(i).norm();
  //   if(row_norm > 1e-6) {  // 避免除以零
  //       _A.row(i) /= row_norm;
  //       _ub(i) /= row_norm;
  //       _lb(i) /= row_norm;
  //   }
  // }
  
  // checkConstraints( _A,  _ub,  _lb,  m);

  // cout << "[Debug] inside polynomial trajectory, out of loop, before Construct corridor constrain" << endl; // for test
  //Construct corridor constrain
  std::vector<Eigen::Matrix<double, 6, -1>> _Polygons = corridors;
  cout << "[Debug] corridor constrain, 111" << endl; // for test
  int row_index = 21*m+6;
  // int C = N_ORDER + 1;
  // int S = N * (N_ORDER + 1) * DIM;
  // int N_PIECES = DIM * C;
  double delta = 1.0 / N_SAMPLES;
  int num_polyhedra = 0;
  /* iterate over all pieces */
  cout << "[Debug] corridor constrain, before loop" << endl; // for test
  for (int idx = 0; idx < m; idx++) {
    std::cout <<"new loop,segment idx:  "<< idx <<std::endl;
    if (m <= 0 || _Polygons.empty() || delta <= 0) {
      std::cerr << "Invalid input parameters" << std::endl;
    }
    Eigen::MatrixXd polyhedra = _Polygons[idx];
    std::cout <<"polyhedra: "<< polyhedra << std::endl;
    num_polyhedra += polyhedra.cols();
    std::cout<<"now number of polyhedra: "<<num_polyhedra;
    /* iterate over uniformly sample positions */
    for (double dt = 0; dt < 1; dt += delta) {
      /* iterate over all polygons */
      for (int i = 0; i < polyhedra.cols(); i++) {
        Eigen::Vector3d a = polyhedra.block<3, 1>(0, i);
        Eigen::Vector3d p = polyhedra.block<3, 1>(3, i);
        // a = -a;  // Negate the normal vector // for test

        // std::cout << "piece\t" << idx << std::endl;
        // std::cout << "time\t" << dt << std::endl;
        std::cout << "surface indx" << i << std::endl;

        // 在corridor约束循环中，添加有效值检查
        if (std::isnan(a.norm()) || std::isnan(p.norm())) {
            std::cerr << "检测到无效的corridor值" << endl;
            continue;
        }

        /* add a single corridor constraint */
        // cout << "[Debug] corridor constrain, before d" << endl; // for test
        Eigen::Matrix<double, 3, 24> d; //_A.block(row_idx+3 + dim, col_idx + dim * 8, 1, p_num1d) = pos_1d;//row_idx+3, col_idx, 3, 24
        for(int dim=0;dim<DIM;dim++){
            d.block(dim,dim * 8,1,p_num1d) << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5), pow(dt, 6),pow(dt, 7);
        }

        // cout << "[Debug] corridor constrain, before d3" << endl; // for test
        Eigen::Matrix<double, 1, 24> d3;
        d3 = a.transpose()*d;
        // cout << "[Debug] corridor constrain, before A_i" << endl; // for test
        Eigen::VectorXd A_i(24*m);
        A_i.setZero();
        // A_i.segment(24*idx, 24) = d3;// debug
        // _A.row(row_index) = A_i; // for modify
        _A.row(row_index).segment(24*idx, 24) = d3; //debug try
        // cout << "[Debug] corridor constrain, before _ub(row_index)" << endl; // for test
        _ub(row_index) = a(0) * p(0) + a(1) * p(1) + a(2) * p(2);
        _lb(row_index) = -OSQP_INFTY;
        row_index++;
      }
    }
  //   // std::cout << row_index << std::endl;
  }
  

  // // 在求解前添加这些检查
  // std::cout << "Q矩阵条件数: " 
  //           << _Q.eigenvalues().maxCoeff() / _Q.eigenvalues().minCoeff() << std::endl;
  // std::cout << "A矩阵条件数: " 
  //           << _A.jacobiSvd().singularValues().maxCoeff() / 
  //             _A.jacobiSvd().singularValues().minCoeff() << std::endl;

  // 在求解前添加
  bool isInitiallyFeasible = true;
  for(int i = 0; i < _lb.size(); i++) {
      if(_lb(i) > _ub(i)) {
          std::cout << "约束在索引 " << i << " 处不可行: lb=" 
                    << _lb(i) << " > ub=" << _ub(i) << std::endl;
          isInitiallyFeasible = false;
      }
  }
  if(!isInitiallyFeasible) {
      std::cerr << "问题初始状态不可行！" << std::endl;
  }
  //check Q value problem
  if (_Q.eigenvalues().real().maxCoeff() / _Q.eigenvalues().real().minCoeff() > 1e10) {
    ROS_WARN("Warning: Q matrix is ill-conditioned");
  }
  //约束独立性检查
  // 检查约束矩阵的秩
  Eigen::FullPivLU<Eigen::MatrixXd> lu(_A);
  int rank = lu.rank();

  // 理论上需要的独立约束数
  int expected_constraints = 
      12 +           // 起点约束
      12 * (m-1) +  // 中间点连续性约束
      12;           // 终点约束

  std::cout << "矩阵秩：" << rank << std::endl;
  std::cout << "预期秩：" << expected_constraints << std::endl;
  
  // 检查线性相关的约束
  double threshold = 1e-10;  // 设置阈值
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(_A);
  VectorXd singular_values = svd.singularValues();
  for(int i = 0; i < singular_values.size(); i++) {
      if(singular_values(i) < threshold) {
          std::cout << "发现接近线性相关的约束，奇异值 " << i << ": " << singular_values(i) << std::endl;
      }
  }
  //约束自由度分析
  // 检查系统的自由度是否合理
  int total_variables = 3 * p_num1d * m;  // 总变量数
  int total_constraints = rank;           // 独立约束数
  int degrees_of_freedom = total_variables - total_constraints;

  std::cout << "总变量数：" << total_variables << std::endl;
  std::cout << "独立约束数：" << total_constraints << std::endl;
  std::cout << "系统自由度：" << degrees_of_freedom << std::endl;

  // 约束可视化
  // 打印约束矩阵的稀疏模式
  // std::cout << "约束矩阵非零模式：" << std::endl;
  // for(int i = 0; i < _A.rows(); i++) {
  //     for(int j = 0; j < _A.cols(); j++) {
  //         if(abs(_A(i,j)) > 1e-10) {
  //             std::cout << "1 ";
  //         } else {
  //             std::cout << "0 ";
  //         }
  //     }
  //     std::cout << std::endl;
  // }


  try{
    //Solve QP
    PolyCoeff = SolveQP(p_num1d,m);
  }catch (const std::exception& e) {
    cout << "[Error] Exception in SolveQP " << e.what() << endl; //for test
  }

  // cout << "[Debug] inside polynomial trajectory, after slove QP" << endl; // for test
  // cout << "[Debug] inside polynomial trajectory, constructing C_Matrix"<<"m:"<<m<<"i:"<<i << endl; // for test
  //C A^T Q A^-1 C^T = R
  // cout << "[Debug] inside polynomial trajectory, constructing C_Trans" << endl; // for test

  // Create new matrix with desired size
  MatrixXd PolyCoeff_resized(m, 3 * p_num1d);

  for(int s = 0; s < m; s++) {
    for(int dim = 0; dim < 3; dim++) {
        for(int j = 0; j < p_num1d; j++) {
          // cout<<"s,j,dim= "<<s<<j<<dim<<endl;
            PolyCoeff_resized(s, dim * p_num1d + j) = PolyCoeff(3 * s * p_num1d + j + dim*p_num1d);
        }
    }
  }
  cout<<"Polycoeff:"<<PolyCoeff_resized<<endl; // for test

  // 在得到PolyCoeff_resized后添加 for test
  for(int s = 0; s < m; s++) {
      std::cout << "段 " << s << " 的系数：\n";
      for(int dim = 0; dim < 3; dim++) {
          std::cout << "维度 " << dim << ": ";
          for(int j = 0; j < p_num1d; j++) {
              std::cout << PolyCoeff_resized(s, dim * p_num1d + j) << " ";
          }
          std::cout << "\n";
      }
  }
 // testing continuousty
  for(int i = 0; i < m-1; i++) {
    // 检查段i结束点和段i+1起始点的连续性
    Vector3d pos_end = getPosPoly(PolyCoeff_resized, i, 1);//modified, origin:Time(i)
    Vector3d pos_start = getPosPoly(PolyCoeff_resized, i+1, 0);
    Vector3d vel_end = getVelPoly(PolyCoeff_resized, i, 1);
    Vector3d vel_start = getVelPoly(PolyCoeff_resized, i+1, 0);
    Vector3d acc_end = getAccPoly(PolyCoeff_resized, i, 1);
    Vector3d acc_start = getAccPoly(PolyCoeff_resized, i+1, 0);
    
    std::cout << "段 " << i << " 和 " << i+1 << " 的连接点：\n";
    std::cout << "位置差： " << (pos_end - pos_start).norm() << "\n";
    std::cout << "速度差： " << (vel_end - vel_start).norm() << "\n";
    std::cout << "加速度差： " << (acc_end - acc_start).norm() << "\n";
  }

// 检查A_Matrix的结构
std::cout << "A_Matrix 的条件数：" << 
    _A.jacobiSvd().singularValues()(0) / 
    _A.jacobiSvd().singularValues()(_A.jacobiSvd().singularValues().size()-1) 
    << "\n";


  return PolyCoeff_resized;
}


//********************************************************************************************** */
Eigen::MatrixXd TrajectoryGeneratorWaypoint::beta(const int tm, const int degree,const int p_num1d ){
  MatrixXd multiplyer_block = MatrixXd::Zero(3,3*p_num1d); //modified deleted all tm
  Matrix3d I_3x3 = Matrix3d::Identity();
  // Validate degree range
  if (degree < 0 || degree > 3) {
      ROS_WARN("[trajectory_generator_waypoint] empty trajectory, beta_ti_degree, degree too large ");
      return multiplyer_block;
  }
  switch (degree)
  {
  case 0:
    multiplyer_block << 1*I_3x3, I_3x3, I_3x3, I_3x3, I_3x3, I_3x3, I_3x3, I_3x3;
    break;
  case 1:
    multiplyer_block << 0*I_3x3, I_3x3, 2*I_3x3, 3*I_3x3, 4*I_3x3, 5*I_3x3, 6*I_3x3, 7*I_3x3;
    break;
  case 2:
    multiplyer_block << 0*I_3x3, 0*I_3x3, 2*I_3x3, 6*I_3x3, 12*I_3x3, 20*I_3x3, 30*I_3x3, 42*I_3x3;
    break;
  case 3:
    multiplyer_block << 0*I_3x3, 0*I_3x3, 0*I_3x3, 6*I_3x3, 24*I_3x3, 60*I_3x3, 120*I_3x3, 210*I_3x3;
    break;
  case 4:
    multiplyer_block << 0*I_3x3, 0*I_3x3, 0*I_3x3, 0*I_3x3, 24*I_3x3, 120*I_3x3, 360*I_3x3, 840*I_3x3;
  break;
  default:
    ROS_WARN("[trajectory_generator_waypoint] empty trajectory, beta_ti_degree, degree too large ");
    break;
  }
  return multiplyer_block; 
}
//********************************************************************************************** */
Eigen::VectorXd TrajectoryGeneratorWaypoint::SolveQP(const int p_num1d, const int m) {
  ROS_INFO("[TrajOpt] start solving");
  Eigen::VectorXd q;
  q.resize(m * p_num1d * DIM);
  q.setZero();
  Eigen::SparseMatrix<double> Q = _Q.sparseView();
  Eigen::SparseMatrix<double> A = _A.sparseView();
  // // 在求解器设置中，使用更宽松的参数
  // qpSolver_.setSettings(10000, true, true, true);
  // // 设置详细的求解器参数
  // qpSolver_.setAdvancedSettings(1e-2, 1e-2, 10000, true, 25, true);
  // 在设置矩阵之前检查问题设置
  bool is_valid = qpSolver_.checkProblemSetup(Q, A, _lb, _ub);
  if (!is_valid) {
      std::cout << "Problem setup has issues!" << std::endl;
  }

  //set qp , solve
  qpSolver_.setMats(Q, q, A, _lb, _ub,1e-2,1e-2);// eps_abs,eps_rel
  std::cout<<"qp setMats done"<<std:endl;
  qpSolver_.solve();
  std::cout<<"qp solve done"<<std:endl;
  int ret = qpSolver_.getStatus();
  if (ret != 1) {
    ROS_ERROR("fail to solve QP!");
    // // 检查约束可行性
    // Eigen::VectorXd sol = qpSolver_.getPrimalSol();
    // Eigen::VectorXd constraint_violation = A * sol;
    // double max_violation = 0.0;
    // for(int i = 0; i < constraint_violation.size(); i++) {
    //     max_violation = std::max(max_violation, 
    //         std::max(_lb(i) - constraint_violation(i),
    //                 constraint_violation(i) - _ub(i)));
    // }
    // ROS_ERROR("Max constraint violation: %.2e", max_violation);
  }

  // 获取详细的诊断信息
  auto diagnostics = qpSolver_.getDiagnostics();
  std::cout << "Iterations: " << diagnostics.iterations << std::endl;
  std::cout << "Status: " << diagnostics.status_msg << std::endl;
  std::cout << "Objective value: " << diagnostics.obj_val << std::endl;
  // 分析解的质量
  qpSolver_.analyzeSolution();

  Eigen::VectorXd sol = qpSolver_.getPrimalSol();
  return sol;
}

// // 检查约束设置
// void checkConstraints(const Eigen::MatrixXd& _A, const Eigen::VectorXd& _ub, const Eigen::VectorXd& _lb, int m) {
//     std::cout << "检查约束矩阵维度：" << std::endl;
//     std::cout << "_A: " << _A.rows() << " x " << _A.cols() << std::endl;
//     std::cout << "_ub: " << _ub.size() << std::endl;
//     std::cout << "_lb: " << _lb.size() << std::endl;
    
//     // 检查约束条件
//     for(int i = 1; i < m-1; i++) {
//         int row_idx = 24+15*(i-1);
//         std::cout << "\n段 " << i << " 的约束：" << std::endl;
        
//         // 检查位置约束
//         std::cout << "位置约束矩阵：\n" << _A.block(row_idx, 24*i, 3, 24) << std::endl;
        
//         // 检查连续性约束
//         std::cout << "连续性约束矩阵：\n" << _A.block(row_idx+3, 24*i, 12, 24) << std::endl;
        
//         // 检查约束边界
//         std::cout << "上边界：" << _ub.segment(row_idx, 15).transpose() << std::endl;
//         std::cout << "下边界：" << _lb.segment(row_idx, 15).transpose() << std::endl;
//     }
// }


// from hw6 //todense？ sparseview ???
// qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
// qpSolver_.solve();
// int ret = qpSolver_.getStatus();
// std::cout<<"****************qpslover return :****************"<<ret<<std::endl;
// if (ret != 1) {
//   ROS_ERROR("fail to solve QP!");
//   return ret;
// }
// Eigen::VectorXd sol = qpSolver_.getPrimalSol(); //Eigen::VectorXd
// Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), m, N_); //selfadd: m=2: input 2 things: a, delta/ N: 40 # horizon
// Eigen::VectorXd solState = BB * sol + AA * x0 + gg;
// Eigen::MatrixXd predictMat = Eigen::Map<const Eigen::MatrixXd>(solState.data(), n, N_);

/// from o47xxx
// bool CorridorMiniSnap::optimize() {
//   getCostFunc();
//   ROS_INFO("[TrajOpt] Generated Cost Func");
//   getHeadTailConstraint();
//   ROS_INFO("[TrajOpt] Generated Head Tail Constraint");
//   getTransitionConstraint();
//   ROS_INFO("[TrajOpt] Generated Transitional Constraint");
//   getContinuityConstraint();
//   ROS_INFO("[TrajOpt] Generated Continuity Constraint");
//   // getCorridorConstraint();
//   // std::cout << "Generated Corridor Constraint" << std::endl;
//   bool isSuccess = primarySolveQP();
//   return isSuccess;
// }


//********************************************************************************************** */
// void CorridorMiniSnapOriginal::getCorridorConstraint(const std::vector<Eigen::Matrix<double, 6, -1>> &corridors) {
//   td::vector<Eigen::Matrix<double, 6, -1>> _Polygons = corridors;
//   int row_index = DIM * 4 * 2 + DIM * 4 * (N - 1) + n_hyperplanes;
//   int C = N_ORDER + 1;
//   int S = N * (N_ORDER + 1) * DIM;
//   int N_PIECES = DIM * C;
//   double delta = 1.0 / N_SAMPLES;
//   /* iterate over all pieces */
//   for (int idx = 0; idx < N; idx++) {
//     Eigen::MatrixXd polyhedra = _Polygons[idx];

//     /* iterate over uniformly sample positions */
//     for (double dt = 0; dt < 1; dt += delta) {
//       /* iterate over all polygons */
//       for (int i = 0; i < polyhedra.cols(); i++) {
//         Eigen::Vector3d a = polyhedra.block<3, 1>(0, i);
//         Eigen::Vector3d p = polyhedra.block<3, 1>(3, i);

//         // std::cout << "piece\t" << idx << std::endl;
//         // std::cout << "time\t" << dt << std::endl;

//         /* add a single corridor constraint */
//         Eigen::Matrix<double, 1, N_ORDER + 1> d;
//         d << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5), pow(dt, 6),
//             pow(dt, 7);
//         Eigen::Matrix<double, 1, 3 * (N_ORDER + 1)> d3;
//         d3 << a(0) * d, a(1) * d, a(2) * d;

//         Eigen::VectorXd A(S);
//         A.setZero();
//         A.segment(idx * N_PIECES, N_PIECES) = d3;
//         _A.row(row_index) = A; // for modify
//         _ub(row_index) = a(0) * p(0) + a(1) * p(1) + a(2) * p(2);
//         _lb(row_index) = -OSQP_INFTY;
//         row_index++;
//       }
//     }
//     // std::cout << row_index << std::endl;
//   }
// }

//*********************************************************************************************** */
double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t){
  
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);
    for (int j = 0; j < _poly_num1D; j++){
        if (j == 0){
          time(j) = 1.0;
        }
        else{
          time(j) = pow(t, j); //selfadd:[1, t, t^2, t^3, ...]
        }
    }
    ret(dim) = coeff.dot(time); //selfadd: 使用点积计算多项式在时间 t 的值。这等价于计算 a0 + a1*t + a2*t^2 + ...
      // cout << "dim:" << dim << " coeff:" << coeff << endl;
    
  }
  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t){
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);
    for (int j = 0; j < _poly_num1D; j++){
      if (j == 0 || j == 1){
        time(j) = 0.0;
      }
      else{
        time(j) = j * (j - 1) * pow(t, j - 2);
      }
    }
    ret(dim) = coeff.dot(time);
  }

  return ret;
}