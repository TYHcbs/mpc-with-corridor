#pragma once

#include <osqp/osqp.h>
#include <iostream> //selfadd
#include <Eigen/Sparse>
#include <memory>

/**
 * osqp interface:
 * minimize     0.5 x' P x + q' x
 * subject to   l <= A x <= u
 **/

namespace osqp {

class IOSQP {
 public:
  IOSQP() : UNBOUNDED_VAL(OSQP_INFTY),
            pWork(nullptr),
            pSettings(nullptr),
            pData(nullptr) {
    pSettings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    pData = (OSQPData *)c_malloc(sizeof(OSQPData));
    if (pSettings)
      osqp_set_default_settings(pSettings);
      pSettings->verbose = 1;  // selfadd directly set verbose
  }

  ~IOSQP() {
    if (pWork)
      osqp_cleanup(pWork);
    if (pSettings)
      c_free(pSettings);
    if (pData)
      c_free(pData);
  }

  const double UNBOUNDED_VAL;

  inline c_int setMats(Eigen::SparseMatrix<double> &P,
                       Eigen::VectorXd &q,
                       Eigen::SparseMatrix<double> &A,
                       Eigen::VectorXd &l,
                       Eigen::VectorXd &u,
                       const double &eps_abs = 1e-3,
                       const double &eps_rel = 1e-3) {
    if (pWork)
      osqp_cleanup(pWork);

    P = P.triangularView<Eigen::Upper>();
    P.makeCompressed();
    A.makeCompressed();

    pData->n = P.rows();
    pData->m = A.rows();

    Eigen::Map<const Eigen::VectorXi> iIdxP(P.innerIndexPtr(), P.nonZeros());
    Eigen::Map<const Eigen::VectorXi> oIdxP(P.outerIndexPtr(), P.cols() + 1);
    Eigen::Matrix<c_int, -1, 1> innerIdxP = iIdxP.cast<c_int>();
    Eigen::Matrix<c_int, -1, 1> outerIdxP = oIdxP.cast<c_int>();
    pData->P = csc_matrix(pData->n,
                          pData->n,
                          P.nonZeros(),
                          P.valuePtr(),
                          innerIdxP.data(),
                          outerIdxP.data());

    Eigen::Map<const Eigen::VectorXi> iIdxA(A.innerIndexPtr(), A.nonZeros());
    Eigen::Map<const Eigen::VectorXi> oIdxA(A.outerIndexPtr(), A.cols() + 1);
    Eigen::Matrix<c_int, -1, 1> innerIdxA = iIdxA.cast<c_int>();
    Eigen::Matrix<c_int, -1, 1> outerIdxA = oIdxA.cast<c_int>();
    pData->A = csc_matrix(pData->m,
                          pData->n,
                          A.nonZeros(),
                          A.valuePtr(),
                          innerIdxA.data(),
                          outerIdxA.data());

    Eigen::Matrix<c_float, -1, 1> pDqVec = q.cast<c_float>();
    pData->q = pDqVec.data();
    Eigen::Matrix<c_float, -1, 1> pDlVec = l.cast<c_float>();
    pData->l = pDlVec.data();
    Eigen::Matrix<c_float, -1, 1> pDuVec = u.cast<c_float>();
    pData->u = pDuVec.data();

    pSettings->eps_abs = eps_abs;
    pSettings->eps_rel = eps_rel;
    // selfadd following : set before osqp_setup
    pSettings->eps_abs = eps_abs;
    pSettings->eps_rel = eps_rel;
    pSettings->verbose = 1;            // 打印求解过程
    pSettings->max_iter = 10000;       // 最大迭代次数
    pSettings->polish = 1;             // 使用polish
    pSettings->adaptive_rho = 1;       // 自适应rho
    pSettings->warm_start = 1;         // 启用热启动
    pSettings->scaled_termination = 1;  // 使用缩放的终止准则

    c_int exitflag = osqp_setup(&pWork, pData, pSettings);

    if (pData->A)
      c_free(pData->A);
    if (pData->P)
      c_free(pData->P);

    return exitflag;
  }

  inline c_int solve() const {
    return osqp_solve(pWork);
  }

  inline c_int getStatus() const {
    return pWork->info->status_val;
  }

  inline Eigen::VectorXd getPrimalSol() const {
    return Eigen::Map<const Eigen::VectorXd>(pWork->solution->x,
                                             pWork->data->n);
  }

  inline void setSettings(const int max_iter, // selfadd
                        const bool verbose = true,
                        const bool polish = true,
                        const bool adaptive_rho = true) {
      if (pSettings) {
          pSettings->max_iter = max_iter;
          pSettings->verbose = verbose;
          pSettings->polish = polish;
          pSettings->adaptive_rho = adaptive_rho;
      }
  }
  // 添加新的诊断信息结构体
  struct DiagnosticInfo {
      int iterations;
      double obj_val;
      double pri_res;
      double dua_res;
      std::string status_msg;
      double solve_time;
  };

  // 添加获取诊断信息的方法
  DiagnosticInfo getDiagnostics() const {
      DiagnosticInfo info;
      if (pWork && pWork->info) {
          info.iterations = pWork->info->iter;
          info.obj_val = pWork->info->obj_val;
          info.pri_res = pWork->info->pri_res;
          info.dua_res = pWork->info->dua_res;
          info.solve_time = pWork->info->solve_time;
          info.status_msg = getStatusString(pWork->info->status_val);
      }
      return info;
  }
  // 添加设置详细的求解器参数的方法
  void setAdvancedSettings(double eps_abs = 1e-3,
                          double eps_rel = 1e-3,
                          int max_iter = 10000,
                          bool verbose = true,
                          int adaptive_rho_interval = 25,
                          bool scaled_termination = true) {
      if (pSettings) {
          pSettings->eps_abs = eps_abs;
          pSettings->eps_rel = eps_rel;
          pSettings->max_iter = max_iter;
          pSettings->verbose = verbose;
          pSettings->adaptive_rho_interval = adaptive_rho_interval;
          pSettings->scaled_termination = scaled_termination;
          
          // 添加更多有用的设置
          pSettings->warm_start = true;
          pSettings->polish = true;
          pSettings->polish_refine_iter = 3;
          pSettings->adaptive_rho = true;
      }
  }

  // 添加检查问题条件的方法
  bool checkProblemSetup(const Eigen::SparseMatrix<double>& P,
                        const Eigen::SparseMatrix<double>& A,
                        const Eigen::VectorXd& l,
                        const Eigen::VectorXd& u) const {
      std::cout << "\n=== Problem Diagnostics ===" << std::endl;
      
      // 检查矩阵维度
      std::cout << "Problem dimensions:" << std::endl;
      std::cout << "Variables (n): " << P.rows() << std::endl;
      std::cout << "Constraints (m): " << A.rows() << std::endl;
      
      // 检查稀疏性
      std::cout << "\nSparsity information:" << std::endl;
      std::cout << "P nonzeros: " << P.nonZeros() 
                << " (" << (double)P.nonZeros()/(P.rows()*P.cols())*100 
                << "%)" << std::endl;
      std::cout << "A nonzeros: " << A.nonZeros() 
                << " (" << (double)A.nonZeros()/(A.rows()*A.cols())*100 
                << "%)" << std::endl;

      // 检查约束可行性
      bool constraints_feasible = true;
      for (int i = 0; i < l.size(); i++) {
          if (l(i) > u(i)) {
              std::cout << "Infeasible constraints at index " << i 
                        << ": " << l(i) << " > " << u(i) << std::endl;
              constraints_feasible = false;
          }
      }
      std::cout <<"check done"<<std::endl;
      return constraints_feasible;
  }

  // 添加求解后的结果分析
  void analyzeSolution() const {
      if (!pWork || !pWork->info) return;

      std::cout << "\n=== Solution Analysis ===" << std::endl;
      std::cout << "Status: " << getStatusString(pWork->info->status_val) << std::endl;
      std::cout << "Iterations: " << pWork->info->iter << std::endl;
      std::cout << "Objective value: " << pWork->info->obj_val << std::endl;
      std::cout << "Primal residual: " << pWork->info->pri_res << std::endl;
      std::cout << "Dual residual: " << pWork->info->dua_res << std::endl;
      std::cout << "Solve time: " << pWork->info->solve_time << " seconds" << std::endl;

      // 检查解的约束违反情况
      if (pWork->solution && pData) {
          Eigen::VectorXd solution = getPrimalSol();
          // 这里可以添加约束检查代码
      }
  }

  

 private:
  OSQPWorkspace *pWork;
  OSQPSettings *pSettings;
  OSQPData *pData;

  std::string getStatusString(int status_val) const {
      switch (status_val) {
          case OSQP_SOLVED: return "Problem solved";
          case OSQP_SOLVED_INACCURATE: return "Problem solved inaccurately";
          case OSQP_PRIMAL_INFEASIBLE: return "Primal infeasible";
          case OSQP_DUAL_INFEASIBLE: return "Dual infeasible";
          case OSQP_MAX_ITER_REACHED: return "Maximum iterations reached";
          default: return "Unknown status";
      }
  }
};

}  // namespace osqp