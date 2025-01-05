#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <Eigen/LU> // selfadd
#include <vector>
#include <iosqp.hpp>

#define N_ORDER 7      // order of polynomial 
#define Par_number 8      // number of parameter in each piece polynomial Par_number=N_ORDER+1
// #define D_ORDER 4      // order of maximum derivative (4 for minisnap)
#define DIM 3          // number of dimensions 
// #define N_POLYHEDRA 6  // number of polygons in polyhedra
#define N_SAMPLES 100   // number of samples for each pieces (Mellinger et al.) //modified,origin:12



class TrajectoryGeneratorWaypoint {
private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
        Eigen::MatrixXd _A; // for qp
        Eigen::VectorXd _ub; // for qp
        Eigen::VectorXd _lb; // for qp
		Eigen::VectorXd _Px, _Py, _Pz;

        uint8_t * data;
        int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
        int GLXYZ_SIZE, GLYZ_SIZE;
        double resolution, inv_resolution;
        double gl_xl, gl_yl, gl_zl;
        double gl_xu, gl_yu, gl_zu;

        osqp::IOSQP qpSolver_;
public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

        Eigen::MatrixXd CorridorMinsnapSolve(
            const int d_order,           // the order of derivative
            const Eigen::MatrixXd &Path, // waypoints coordinates (3d) //selfadd:MatrixXd path(int(grid_path.size()), 3); path.row(k) = grid_path[k]; 
            const double Vel,  // boundary velocity
            const double Acc,  // boundary acceleration
            const Eigen::VectorXd &Time, // time allocation in each segment
            const std::vector<Eigen::Matrix<double, 6, -1>> &corridors); // corridor

        Eigen::MatrixXd beta(const int tm, const int degree,const int p_num1d );
        Eigen::VectorXd SolveQP(const int p_num1d, const int m);
        void checkConstraints(const Eigen::MatrixXd& _A, const Eigen::VectorXd& _ub, const Eigen::VectorXd& _lb, int m);


        double getObjective();
        Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getVelPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getAccPoly( Eigen::MatrixXd polyCoeff, int k, double t );
};

#endif
