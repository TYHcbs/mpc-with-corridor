#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>//PolyQPGeneration
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
  Matrix3d I_3x3 = Matrix3d::Identity();
  MatrixXd I_12x12 = MatrixXd::Identity(12,12);
  MatrixXd One_3x3 = MatrixXd::Ones(3,3);
  double lambda = 1e-5;  // 正则化参数 origin: 1e-8
  MatrixXd Q_Matrix = MatrixXd::Zero(3*p_num1d*m,3*p_num1d*m); //24mx24m
  MatrixXd A_Matrix = MatrixXd::Zero(3*p_num1d*m,3*p_num1d*m); //24mx24m
  MatrixXd C_Matrix_Trans = MatrixXd::Zero(24*m,12*m+12); //24mx12m+12
  C_Matrix_Trans.block(0,0,3*4,3*4) = I_12x12;
  C_Matrix_Trans.block(24*m-24+12,3*(m+3),3*4,3*4) = I_12x12;
  for(int i = 0;i<m;i++){
  double tm = Time(i);
  // cout << "[Debug] inside polynomial trajectory, constructing Qi" << endl; // for test
  //Construct Qi:
  MatrixXd Q_i = MatrixXd::Zero(3 * p_num1d, 3 * p_num1d);

  Q_i = beta(tm,4,p_num1d).transpose()*beta(tm,4,p_num1d);

  // 添加正则化
  // Q_i.diagonal().array() += lambda;

  // 或者对高阶项使用更大的正则化系数
  for(int i = 0; i < p_num1d; i++) {
      double reg = lambda * pow(10, i);  // 高阶项用更大的正则化系数
      Q_i.block(3*i,3*i,3,3).diagonal().array() += reg;
  }
  Q_Matrix.block(3*p_num1d*i, 3*p_num1d*i, 3*p_num1d, 3*p_num1d) = Q_i;
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

  A_Matrix.block(3*p_num1d*i, 3*p_num1d*i, 3*p_num1d, 3*p_num1d) = A_i; 
  //Construct C_Matrix: //24mx12m+12
  if(i>0){
    C_Matrix_Trans.block(24*i+0+ 0*3 +0, 12+ 3*(i-1) +0, 3, 3) = I_3x3; // d_{i,0}^{0},x/y/z
    C_Matrix_Trans.block(24*i+0+ 1*3 +0, 3*(m+7)+9*(i-1)+ 0*3 +0, 3, 3) = I_3x3; // d_{i,0}^{1},x/y/z
    C_Matrix_Trans.block(24*i+0+ 2*3 +0, 3*(m+7)+9*(i-1)+ 1*3 +0, 3, 3) = I_3x3; // d_{i,0}^{2},x/y/z
    C_Matrix_Trans.block(24*i+0+ 3*3 +0, 3*(m+7)+9*(i-1)+ 2*3 +0, 3, 3) = I_3x3; // d_{i,0}^{3},x/y/z
  }
  if(i<m-1){
    C_Matrix_Trans.block(24*i+12+ 0*3 +0, 12+ 3*i +0, 3, 3) = I_3x3; // d_{i,T}^{0},x/y/z
    C_Matrix_Trans.block(24*i+12+ 1*3 +0, 3*(m+7)+9*i+ 0*3 +0, 3, 3) = I_3x3; // d_{i,T}^{1},x/y/z
    C_Matrix_Trans.block(24*i+12+ 2*3 +0, 3*(m+7)+9*i+ 1*3 +0, 3, 3) = I_3x3; // d_{i,T}^{2},x/y/z
    C_Matrix_Trans.block(24*i+12+ 3*3 +0, 3*(m+7)+9*i+ 2*3 +0, 3, 3) = I_3x3; // d_{i,T}^{3},x/y/z
  }
  }
  //C A^T Q A^-1 C^T = R
  MatrixXd C_Matrix = C_Matrix_Trans.transpose();
  MatrixXd A_inv(A_Matrix.rows(), A_Matrix.cols());
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A_Matrix);
  A_inv = lu.inverse();
  cout<<"A_inv size: "<<A_inv.size()<< endl;
  MatrixXd A_invTrans = A_inv.transpose(); // selfadd:matrix inverse note!
  MatrixXd R_Matrix = C_Matrix*A_invTrans*Q_Matrix*A_inv*C_Matrix_Trans;
  cout<<"R_Matrix: done"<<endl; // for test

  //dP∗=− RPP^−1 * RFP^T *dF
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
  // fill in dF 
  dF.block(0,0,3,1) = Path.row(0).transpose(); //Position0
  dF.block(3*(m+3),0,3,1) = Path.row(m).transpose(); //PositionM
  for(int j =1;j<m;j++){
    cout<<"j= "<<j<<endl; //for test
    dF.block(12+3*(j-1),0,3,1) =Path.row(j).transpose(); //position condition(except p0 and pM)
  }
  VectorXd dP_star = -1*Rpp_inv*Rfp_trans*dF;
  cout<<"dP_star: done"<<endl; // for test

  VectorXd dPF(12*m+12); 
  dPF << dF,dP_star;

  VectorXd D;
  D = C_Matrix_Trans*dPF;
  cout<<"D_Matrix = done"<<endl;

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

// // Check A_Matrix 
// std::cout << "A_Matrix 的条件数：" << 
//     A_Matrix.jacobiSvd().singularValues()(0) / 
//     A_Matrix.jacobiSvd().singularValues()(A_Matrix.jacobiSvd().singularValues().size()-1) 
//     << "\n";

// // Check Q_Matrix
// std::cout << "Q_Matrix 非零元素的位置：\n";
// for(int i = 0; i < Q_Matrix.rows(); i++) {
//     for(int j = 0; j < Q_Matrix.cols(); j++) {
//         if(abs(Q_Matrix(i,j)) > 1e-10) {
//             std::cout << "(" << i << "," << j << "): " << Q_Matrix(i,j) << "\n";
//         }
//     }
// }

//Check R_Matrix
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
  cout<<"[Debug] Path: "<<Path<<endl;
  cout<<"[Debug] Time: "<<Time<<endl;

  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = N_ORDER + 1;     // the number of variables in each segment
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
  double lambda = 1e-2;  // Regularization  parameter origin:1e-8
  _Q.resize(DIM*p_num1d*m,DIM*p_num1d*m);
  _Q.setZero();

  int  n_hyperplanes = 0;
  if(corridors.size()==1){
    n_hyperplanes = corridors[0].cols();
  }
  for (int i = 0; i < corridors.size() - 1; i++) {
    int c_prev = corridors[i].cols();
    int c_next = corridors[i + 1].cols();
    n_hyperplanes += c_prev;
    n_hyperplanes += c_next;
  }
  std::cout<<"n_hyperplanes: "<< n_hyperplanes <<std::endl;
  int N = 21*m+9+n_hyperplanes;
  int W = DIM*p_num1d*(m+1);
  std::cout<<"N: "<< N <<std::endl;
  _A.resize(N+10000,W); //[(15m+9)+(6m-3)+96m] x24m = 107m+6 x24m  // 24+15(m-1)=15m+9,15m+9+6m=21m+9
  _A.setZero();
  _ub.resize(N+10000);
  _ub.setZero();
  _lb.resize(N+10000);
  _lb.setZero();

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
  //equality constrain A0
  for(int dim=0;dim<DIM;dim++){
    cout<<"dim= "<<dim<<endl;
    // Derivative constrain For Position0
    _A(0+dim,dim*8+0) = 1; //p
    _A(3+dim,dim*8+1) = 1; //v
    _A(6+dim,dim*8+2) = 2; //a
    _A(9+dim,dim*8+3) = 6;  //j

    // Continuous constrain for piece0
    // A0(t0)
    _A.block(12+0 + dim, dim * 8, 1, p_num1d) = pos_1d;//Position1=piece0(t0)
    _A.block(12+3 + dim, dim * 8, 1, p_num1d) = vel_1d / t0;
    _A.block(12+6 + dim, dim * 8, 1, p_num1d) = acc_1d /t0 / t0;
    _A.block(12+9 + dim, dim * 8, 1, p_num1d) = jer_1d / t0 / t0 / t0;

    // -A1(0)
    if (m>1){
      double t1 = Time(1);
      _A(12+0 + dim, 24+ dim * 8 +0) = -1;
      _A(12+3 + dim, 24+ dim * 8 +1) = -1/t1;
      _A(12+6 + dim, 24+ dim * 8 +2) = -2 / t1 / t1;
      _A(12+9 + dim, 24+ dim * 8 +3) = -6 / t1 / t1 / t1;
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

  // Regulization
  for(int i = 0; i < p_num1d; i++) {
      double reg = lambda * pow(2, i);  // 高阶项用更大的正则化系数
      Q_i(i,i) = Q_i(i,i)+reg;
  }

  /* for all dimensions and all pieces */
  for (int i = 0; i < m * DIM; i++) {
  _Q.block(i * Par_number, i * Par_number, Par_number, Par_number) = Q_i;

  }

  for(int i = 0;i<m;i++){
  double tm = Time(i);

  // // Regulization
  // Q_i.diagonal().array() += lambda;

  // use bigger regulization parameter for higher order

  if(i>0 && i<m-1){ 
      int row_idx = 24+15*(i-1); 
      int col_idx = 24*i; 
      //position constrain
      for (int dim = 0; dim < DIM; dim++) {
      _A(row_idx + dim, col_idx +dim * 8+0) = 1;
      }
      _ub.block(row_idx, 0, 3, 1) = Path.row(i).transpose();
      _lb.block(row_idx, 0, 3, 1) = Path.row(i).transpose();
      //continuous constrain
      for (int dim = 0; dim < DIM; dim++) {
      _A.block(row_idx+3 + dim, col_idx + dim * 8, 1, p_num1d) = pos_1d;//row_idx+3, col_idx, 3, 24
      _A.block(row_idx+6 + dim, col_idx + dim * 8, 1, p_num1d) = vel_1d / tm;
      _A.block(row_idx+9 + dim, col_idx + dim * 8, 1, p_num1d) = acc_1d /tm / tm;
      _A.block(row_idx+12 + dim, col_idx + dim * 8, 1, p_num1d) = jer_1d / tm / tm / tm;
      _A(row_idx+3 + dim, col_idx + DIM*p_num1d +dim * 8+0) = -1;
      _A(row_idx+6 + dim, col_idx + DIM*p_num1d +dim * 8+1) = -1 / Time(i+1);
      _A(row_idx+9 + dim, col_idx + DIM*p_num1d +dim * 8+2) = -2 / Time(i+1) / Time(i+1);
      _A(row_idx+12 + dim, col_idx + DIM*p_num1d +dim * 8+3) = -6 / Time(i+1) / Time(i+1) / Time(i+1);
      //continous constrain _lb,_up bound is 0
      }   
  }
  // //Construct inequality constrains for vmax amax
  for (int dim = 0; dim < DIM; dim++) {
      _A.block(15*m+9 +6*i +dim, 24*i + dim * 8, 1, p_num1d) = vel_1d / tm;
      _A.block(15*m+9 +6*i+3 +dim, 24*i + dim * 8, 1, p_num1d) = acc_1d /tm / tm;
    //inequality contrains in _ub _lb:
    _ub.segment(15*m+9 +6*i +dim,3) << vmax,vmax,vmax;  //vi 
    _ub.segment(15*m+9 +6*i+3 +dim,3) << amax,amax,amax;  //ai
    _lb.segment(15*m+9 +6*i +dim,3) << -vmax,-vmax,-vmax;  //vi 
    _lb.segment(15*m+9 +6*i+3 +dim,3) << -amax,-amax,-amax;  //ai
  }
  }

  // //numerical stablize
  // for(int i = 0; i < _A.rows(); i++) {
  //   double row_norm = _A.row(i).norm();
  //   if(row_norm > 1e-6) {  // 避免除以零
  //       _A.row(i) /= row_norm;
  //       _ub(i) /= row_norm;
  //       _lb(i) /= row_norm;
  //   }
  // }
 
  cout << "[Debug] *********before Construct corridor constrain**********" << endl; // for test
  //Construct corridor constrain
  std::vector<Eigen::Matrix<double, 6, -1>> _Polygons = corridors;
  int row_index = 21*m+9;

  double delta = 1.0 / N_SAMPLES;
  int num_polyhedra = 0;
  /* iterate over all pieces */
  for (int idx = 0; idx < m; idx++) {
    if (m <= 0 || _Polygons.empty() || delta <= 0) {
      std::cerr << "Invalid input parameters" << std::endl;
    }
    Eigen::MatrixXd polyhedra = _Polygons[idx];
    // std::cout <<"polyhedra: "<< polyhedra << std::endl;
    num_polyhedra += polyhedra.cols();
    /* iterate over uniformly sample positions */
    for (double dt = 0; dt < 1; dt += delta) {
      /* iterate over all polygons */
      for (int i = 0; i < polyhedra.cols(); i++) {
        Eigen::Vector3d a = polyhedra.block<3, 1>(3, i);
        Eigen::Vector3d p = polyhedra.block<3, 1>(0, i);
        p = p-0.2*a;
        // 在corridor约束循环中，添加有效值检查
        if (std::isnan(a.norm()) || std::isnan(p.norm())) {
            std::cerr << "invalid corridor value!" << endl;
            continue;
        }

        /* add a single corridor constraint */
        Eigen::Matrix<double, 3, 24> d; 
        d.setZero();
        for(int dim=0;dim<DIM;dim++){
            d.block(dim,dim * 8,1,p_num1d) << 1, dt, pow(dt, 2), pow(dt, 3), pow(dt, 4), pow(dt, 5), pow(dt, 6),pow(dt, 7);
        }

        Eigen::Matrix<double, 1, 24> d3;
        d3 = a.transpose()*d;
        Eigen::VectorXd A_i(24*m);
        A_i.setZero();
        _A.row(row_index).segment(24*idx, 24) << d3; 
        _ub(row_index) = a(0) * p(0) + a(1) * p(1) + a(2) * p(2);//= a(0) * p(0) + a(1) * p(1) + a(2) * p(2);
        _lb(row_index) = -OSQP_INFTY;
        row_index++;
      }
    }
  }
  std::cout<<"corridor constrain DONE!"<<std::endl;

  // Constrain Feasibility Check
  bool isInitiallyFeasible = true;
  for(int i = 0; i < _lb.size(); i++) {
      if(_lb(i) > _ub(i)) {
          std::cout << "constrain at index " << i << " infeasible: lb=" 
                    << _lb(i) << " > ub=" << _ub(i) << std::endl;
          isInitiallyFeasible = false;
      }
  }
  if(!isInitiallyFeasible) {
      std::cerr << "Problem initial state infeasible!" << std::endl;
  }
  //check Q value problem
  if (_Q.eigenvalues().real().maxCoeff() / _Q.eigenvalues().real().minCoeff() > 1e10) {
    ROS_WARN("Warning: Q matrix is ill-conditioned");
  }

  // //Constrain independence check
  // //check rank
  // Eigen::FullPivLU<Eigen::MatrixXd> lu(_A);
  // int rank = lu.rank();
  // // independent constrain needed
  // int expected_constraints = 
  //     12 +           // start
  //     12 * (m-1) +  // intermediate
  //     12;           // target

  // std::cout << "rank：" << rank << std::endl;
  // std::cout << "expected rank：" << expected_constraints << std::endl;
  
  // Constrain Linear Correlation Analysis
  // double threshold = 1e-10;  // set threshold
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd(_A);
  // VectorXd singular_values = svd.singularValues();
  // for(int i = 0; i < singular_values.size(); i++) {
  //     if(singular_values(i) < threshold) {
  //         std::cout << "Discovering constraints that are close to linear correlation, singular values " << i << ": " << singular_values(i) << std::endl;
  //     }
  // }

  // //Constrain degrees of freedom check
  // int total_variables = 3 * p_num1d * m;  // total variables
  // int total_constraints = rank;           // independent constrains number
  // int degrees_of_freedom = total_variables - total_constraints;

  // std::cout << "Total_variables：" << total_variables << std::endl;
  // std::cout << "Independent contrains：" << total_constraints << std::endl;
  // std::cout << "Degrees of freedom：" << degrees_of_freedom << std::endl;


  try{
    //Solve QP
    PolyCoeff = SolveQP(p_num1d,m);
  }catch (const std::exception& e) {
    cout << "[Error] Exception in SolveQP " << e.what() << endl; //for test
  }

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

// //Print out coefficient
// for(int s = 0; s < m; s++) {
//     std::cout << "segment " << s << " parmeter：\n";
//     for(int dim = 0; dim < 3; dim++) {
//         std::cout << "dimention " << dim << ": ";
//         for(int j = 0; j < p_num1d; j++) {
//             std::cout << PolyCoeff_resized(s, dim * p_num1d + j) << " ";
//         }
//         std::cout << "\n";
//     }
// }

//  // testing continuousty
//   for(int i = 0; i < m-1; i++) {
//     // check segment i end and segment i+1 start point continuouty
//     Vector3d pos_end = getPosPoly(PolyCoeff_resized, i, 1);//modified, origin:Time(i)
//     Vector3d pos_start = getPosPoly(PolyCoeff_resized, i+1, 0);
//     Vector3d vel_end = getVelPoly(PolyCoeff_resized, i, 1);
//     Vector3d vel_start = getVelPoly(PolyCoeff_resized, i+1, 0);
//     Vector3d acc_end = getAccPoly(PolyCoeff_resized, i, 1);
//     Vector3d acc_start = getAccPoly(PolyCoeff_resized, i+1, 0);
//     std::cout << "segment " << i << " and " << i+1 << " 的connection point：\n";
//     std::cout << "position error： " << (pos_end - pos_start).norm() << "\n";
//     std::cout << "velocity error： " << (vel_end - vel_start).norm() << "\n";
//     std::cout << "accelaration error： " << (acc_end - acc_start).norm() << "\n";
//   }

// std::cout << "A_Matrix 的条件数：" << 
//     _A.jacobiSvd().singularValues()(0) / 
//     _A.jacobiSvd().singularValues()(_A.jacobiSvd().singularValues().size()-1) 
//     << "\n";

  return PolyCoeff_resized;
}



//********************************************************************************************** */
Eigen::VectorXd TrajectoryGeneratorWaypoint::SolveQP(const int p_num1d, const int m) {
  ROS_INFO("[TrajOpt] start solving");
  Eigen::VectorXd q;
  q.resize(m * p_num1d * DIM);
  q.setZero();
  Eigen::SparseMatrix<double> Q = _Q.sparseView();
  Eigen::SparseMatrix<double> A = _A.sparseView();

  // check problem set
  bool is_valid = qpSolver_.checkProblemSetup(Q, A, _lb, _ub);
  if (!is_valid) {
      std::cout << "Problem setup has issues!" << std::endl;
  }
  //>>>check before setMats>>>
  // check dimention
  int n = m * p_num1d * DIM; // # of variable
  
  // check matrix dimention
  if (Q.rows() != n || Q.cols() != n) {
    ROS_ERROR("Q matrix dimention not right:expected %dx%d,get %ldx%ld", 
              n, n, Q.rows(), Q.cols());
    throw std::runtime_error("Q matrix dimention not right");
  }
  
  // check q matrix dimention
  if (q.size() != n) {
    ROS_ERROR("q matrix dimention not right:expected  %d,get %ld", n, q.size());
    throw std::runtime_error("q matrix dimention not right");
  }
  
  // 验证A矩阵维度是否与lb/ub匹配
  if (_lb.size() != _ub.size() || _A.cols() != n+24) {
    ROS_ERROR("_A matrix dimention not right");
    throw std::runtime_error("_A matrix dimention not right");
  }

  // 检查NaN/Inf值
  if (!_Q.allFinite() ) {//|| !_A.allFinite() || !_lb.allFinite() || !_ub.allFinite()
    ROS_ERROR("Invalid values detected in the optimized _q matrix (NaN/Inf)");
    throw std::runtime_error("_Q has invalid value");
  }
  if (!_A.allFinite() ) {//|| !_A.allFinite() || !_lb.allFinite() || !_ub.allFinite()
    ROS_ERROR("Invalid values detected in the optimized _A matrix(NaN/Inf)");
    throw std::runtime_error("_A has invalid value");
  } 
  if (!_lb.allFinite() ) {//|| !_A.allFinite() || !_lb.allFinite() || !_ub.allFinite()
    ROS_ERROR("Invalid values detected in the optimized _lb matrix(NaN/Inf)");
    throw std::runtime_error("_lb has invalid value");
  }
  if (!_ub.allFinite() ) {//|| !_A.allFinite() || !_lb.allFinite() || !_ub.allFinite()
    ROS_ERROR("Invalid values detected in the optimized _ub matrix(NaN/Inf)");
    throw std::runtime_error("_ub has invalid value");
  }
  //<<<end of check Mats
  //set qp , solve
  qpSolver_.setMats(Q, q, A, _lb, _ub,1e-3,1e-3);// eps_abs,eps_rel
  std::cout<<"qp setMats done"<<std::endl;
  qpSolver_.solve();
  std::cout<<"qp solve done"<<std::endl;
  int ret = qpSolver_.getStatus();
  if (ret != 1) {
    ROS_ERROR("fail to solve QP!");
  }

  // Diagonse information
  auto diagnostics = qpSolver_.getDiagnostics();
  std::cout << "Iterations: " << diagnostics.iterations << std::endl;
  std::cout << "Status: " << diagnostics.status_msg << std::endl;
  std::cout << "Objective value: " << diagnostics.obj_val << std::endl;
  // Solution quality
  qpSolver_.analyzeSolution();

  Eigen::VectorXd sol = qpSolver_.getPrimalSol();
  return sol;
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
          time(j) = pow(t, j); //[1, t, t^2, t^3, ...]
        }
    }
    ret(dim) = coeff.dot(time); //use dot product for polynomial value at t.equal a0 + a1*t + a2*t^2 + ...
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