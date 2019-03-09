#ifndef ICUB_SOLVER_QP_SOLVER_HPP
#define ICUB_SOLVER_QP_SOLVER_HPP

#include <qpOASES.hpp>

#include <icub/model/iCub.hpp>

namespace icub {
    namespace solver {
        template <typename QPProblem = qpOASES::QProblem>
        class QPSolver {
        public:
            QPSolver() {}
            QPSolver(model::iCub* icub) : _icub(icub) {}

            void set_icub(model::iCub* icub)
            {
                _icub = icub;
            }

            void solve(const Eigen::VectorXd& desired_eef_acc)
            {
                setup_matrices(desired_eef_acc);
                solve();
            }

            void setup_matrices(const Eigen::VectorXd& desired_eef_acc)
            {
                // Get mass matrix
                _M = _icub->robot()->skeleton()->getMassMatrix();
                // Get gravity/coriolis forces
                _Cg = _icub->robot()->skeleton()->getCoriolisAndGravityForces();
                // Selection matrix
                _S = Eigen::MatrixXd::Identity(_icub->robot()->skeleton()->getNumDofs(), _icub->robot()->skeleton()->getNumDofs());
                _S.diagonal().head(6) = Eigen::VectorXd::Zero(6);
                // Get joint velocities
                _dq = _icub->robot()->skeleton()->getVelocities();
                // Get Jacobians for COM
                _J_com = _icub->robot()->skeleton()->getCOMJacobian();
                _dJ_com = _icub->robot()->skeleton()->getCOMJacobianSpatialDeriv();
                // Get Jacobians for end-effectors
                auto legs = model::leg_eefs();
                auto hands = model::arm_eefs();

                _J_rhand = _icub->robot()->skeleton()->getWorldJacobian(_icub->robot()->skeleton()->getBodyNode(hands[0]));
                _dJ_rhand = _icub->robot()->skeleton()->getJacobianSpatialDeriv(_icub->robot()->skeleton()->getBodyNode(hands[0]), dart::dynamics::Frame::World());

                _J_lhand = _icub->robot()->skeleton()->getWorldJacobian(_icub->robot()->skeleton()->getBodyNode(hands[1]));
                _dJ_lhand = _icub->robot()->skeleton()->getJacobianSpatialDeriv(_icub->robot()->skeleton()->getBodyNode(hands[1]), dart::dynamics::Frame::World());

                _J_rfoot = _icub->robot()->skeleton()->getWorldJacobian(_icub->robot()->skeleton()->getBodyNode(legs[0]));
                _dJ_rfoot = _icub->robot()->skeleton()->getJacobianSpatialDeriv(_icub->robot()->skeleton()->getBodyNode(legs[0]), dart::dynamics::Frame::World());

                _J_lfoot = _icub->robot()->skeleton()->getWorldJacobian(_icub->robot()->skeleton()->getBodyNode(legs[1]));
                _dJ_lfoot = _icub->robot()->skeleton()->getJacobianSpatialDeriv(_icub->robot()->skeleton()->getBodyNode(legs[1]), dart::dynamics::Frame::World());

                // std::cout << "M: " << _M.rows() << "x" << _M.cols() << std::endl;
                // std::cout << "Cg: " << _Cg.size() << std::endl;
                // std::cout << "S: " << _S.rows() << "x" << _S.cols() << std::endl;
                // std::cout << "dq: " << _dq.size() << std::endl;
                // std::cout << "J_com: " << _J_com.rows() << "x" << _J_com.cols() << std::endl;
                // std::cout << "dJ_com: " << _dJ_com.rows() << "x" << _dJ_com.cols() << std::endl;
                // std::cout << "J_rhand: " << _J_rhand.rows() << "x" << _J_rhand.cols() << std::endl;
                // std::cout << "dJ_rhand: " << _dJ_rhand.rows() << "x" << _dJ_rhand.cols() << std::endl;
                // std::cout << "J_lhand: " << _J_lhand.rows() << "x" << _J_lhand.cols() << std::endl;
                // std::cout << "dJ_lhand: " << _dJ_lhand.rows() << "x" << _dJ_lhand.cols() << std::endl;
                // std::cout << "J_rfoot: " << _J_rfoot.rows() << "x" << _J_rfoot.cols() << std::endl;
                // std::cout << "dJ_rfoot: " << _dJ_rfoot.rows() << "x" << _dJ_rfoot.cols() << std::endl;
                // std::cout << "J_lfoot: " << _J_lfoot.rows() << "x" << _J_lfoot.cols() << std::endl;
                // std::cout << "dJ_lfoot: " << _dJ_lfoot.rows() << "x" << _dJ_lfoot.cols() << std::endl;

                size_t dofs = _icub->robot()->skeleton()->getNumDofs();
                _dim = 2 * dofs + 5 * 6;
                Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(_J_com.rows() * 5, _dim);

                size_t index = 0;
                tmp.block(index, 0, _J_com.rows(), _J_com.cols()) = _J_com;
                index += _J_com.rows();
                tmp.block(index, 0, _J_rhand.rows(), _J_rhand.cols()) = _J_rhand;
                index += _J_rhand.rows();
                tmp.block(index, 0, _J_lhand.rows(), _J_lhand.cols()) = _J_lhand;
                index += _J_lhand.rows();
                tmp.block(index, 0, _J_rfoot.rows(), _J_rfoot.cols()) = _J_rfoot;
                index += _J_rfoot.rows();
                tmp.block(index, 0, _J_lfoot.rows(), _J_lfoot.cols()) = _J_lfoot;

                Eigen::VectorXd tmp_vec = desired_eef_acc;
                index = 0;
                tmp_vec.segment(index, 6) -= _dJ_com * _dq;
                index += 6;
                tmp_vec.segment(index, 6) -= _dJ_rhand * _dq;
                index += 6;
                tmp_vec.segment(index, 6) -= _dJ_lhand * _dq;
                index += 6;
                tmp_vec.segment(index, 6) -= _dJ_rfoot * _dq;
                index += 6;
                tmp_vec.segment(index, 6) -= _dJ_lfoot * _dq;

                _H = tmp.transpose() * tmp;
                _g = -tmp.transpose() * tmp_vec;

                _lb = Eigen::VectorXd::Zero(_dim);
                _ub = Eigen::VectorXd::Zero(_dim);

                _lb.head(dofs) = _icub->robot()->skeleton()->getAccelerationLowerLimits();
                _ub.head(dofs) = _icub->robot()->skeleton()->getAccelerationUpperLimits();

                _lb.segment(dofs, dofs) = _icub->robot()->skeleton()->getForceLowerLimits();
                _ub.segment(dofs, dofs) = _icub->robot()->skeleton()->getForceUpperLimits();

                // _lb.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Zero(30);
                // _ub.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Zero(30);
                // _lb.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Constant(5 * 6, -std::numeric_limits<double>::max());
                // _ub.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Constant(5 * 6, std::numeric_limits<double>::max());

                _A = Eigen::MatrixXd::Zero(dofs, _dim);

                _A.block(0, 0, dofs, dofs) = _M;
                _A.block(0, dofs, dofs, dofs) = -_S;
                _A.block(0, 2 * dofs, dofs, 6) = -_J_com.transpose();
                _A.block(0, 2 * dofs + 6, dofs, 6) = -_J_rhand.transpose();
                _A.block(0, 2 * dofs + 12, dofs, 6) = -_J_lhand.transpose();
                _A.block(0, 2 * dofs + 18, dofs, 6) = -_J_rfoot.transpose();
                _A.block(0, 2 * dofs + 24, dofs, 6) = -_J_lfoot.transpose();

                // TO-DO: Check this sign
                _lbA = -_Cg;
                _ubA = -_Cg;
            }

            void solve()
            {
                size_t dofs = _icub->robot()->skeleton()->getNumDofs();

                if (!_solver)
                    _solver = std::unique_ptr<QPProblem>(new QPProblem(_dim, dofs));

                // qpOASES::real_t* H = new qpOASES::real_t[_dim * _dim];
                // qpOASES::real_t* A = new qpOASES::real_t[dofs * _dim];
                // qpOASES::real_t* g = new qpOASES::real_t[_dim];
                // qpOASES::real_t* lb = new qpOASES::real_t[_dim];
                // qpOASES::real_t* ub = new qpOASES::real_t[_dim];
                // qpOASES::real_t* lbA = new qpOASES::real_t[dofs];
                // qpOASES::real_t* ubA = new qpOASES::real_t[dofs];

                // memcpy(H, _H.data(), _dim * _dim * sizeof(double));
                // memcpy(A, _A.data(), dofs * _dim * sizeof(double));
                // memcpy(g, _g.data(), _dim * sizeof(double));
                // memcpy(lb, _lb.data(), _dim * sizeof(double));
                // memcpy(ub, _ub.data(), _dim * sizeof(double));
                // memcpy(lbA, _lbA.data(), dofs * sizeof(double));
                // memcpy(ubA, _ubA.data(), dofs * sizeof(double));

                auto options = _solver->getOptions();
                options.printLevel = qpOASES::PL_LOW;
                _solver->setOptions(options);
                int nWSR = 200;
                // qpOASES uses row-major storing
                // check if values are passed correctly
                _solver->init(_H.transpose().data(), _g.data(), _A.transpose().data(), _lb.data(), _ub.data(), _lbA.data(), _ubA.data(), nWSR);

                Eigen::VectorXd x(_dim);
                _solver->getPrimalSolution(x.data());
                std::cout << x.transpose() << std::endl;

                // delete[] H;
                // delete[] A;
                // delete[] g;
                // delete[] lb;
                // delete[] ub;
                // delete[] lbA;
                // delete[] ubA;
            }

        protected:
            std::unique_ptr<QPProblem> _solver = nullptr;
            model::iCub* _icub;

            // Robot matrices
            Eigen::MatrixXd _M, _S;
            Eigen::VectorXd _Cg, _dq;
            Eigen::MatrixXd _J_com, _J_rhand, _J_lhand, _J_rfoot, _J_lfoot;
            Eigen::MatrixXd _dJ_com, _dJ_rhand, _dJ_lhand, _dJ_rfoot, _dJ_lfoot;

            // QP matrices
            size_t _dim;
            Eigen::MatrixXd _H, _A;
            Eigen::VectorXd _g, _ub, _lb, _ubA, _lbA;
        };
    } // namespace solver
} // namespace icub

#endif