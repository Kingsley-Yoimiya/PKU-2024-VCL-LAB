#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/Final_Project/tasks.h"


namespace VCX::Labs::Animation {
    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    Eigen::VectorXf calc_y(MassSpringSystem &system, Eigen::VectorXf const Positions, float const dt) {
        return  Positions + dt * glm2eigen(system.Velocities) + 
                dt * dt / system.Mass * glm2eigen(
                    std::vector<glm::vec3>(
                        system.Positions.size(), 
                        glm::vec3(0, -system.Gravity, 0)
                    )
                );
    }

    glm::vec3 getVec3(Eigen::VectorXf x, int pos) { 
        return glm::vec3({ x[pos * 3], x[pos * 3 + 1], x[pos * 3 + 2] });
    }

    void setVec3(Eigen::VectorXf &x, int pos, glm::vec3 ret) {
        x[pos * 3] = ret[0];
        x[pos * 3 + 1] = ret[1];
        x[pos * 3 + 2] = ret[2];
    }

    float calc_g(MassSpringSystem &system, Eigen::VectorXf const Positions, Eigen::VectorXf const y, float const dt) {
        Eigen::VectorXf tmp = Positions - y;
        float ret = tmp.dot(tmp) * system.Mass / (2 * dt * dt);
        for(const auto spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = getVec3(Positions, p1) - getVec3(Positions, p0);
            // glm::vec3 const e01 = glm::normalize(x01);
            ret += 0.5 * system.Stiffness * glm::pow(glm::length(x01) - spring.RestLength, 2);
        }
        return ret;
    }
    
    Eigen::VectorXf calc_dg(MassSpringSystem &system, Eigen::VectorXf const Positions, Eigen::VectorXf const y, float const dt) {
        Eigen::VectorXf ret = (Positions - y) * system.Mass / (dt * dt); //Eigen::VectorXf::Zero(Positions.size());
        for(const auto spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = getVec3(Positions, p1) - getVec3(Positions, p0);
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = system.Stiffness * (glm::length(x01) - spring.RestLength) * e01;
            for(int i = 0; i < 3; i++) {
                ret[p0 * 3 + i] -= f[i];
                ret[p1 * 3 + i] += f[i];
            }
        }
        return ret;
    }

    Eigen::SparseMatrix<float> calc_ddg(MassSpringSystem &system, Eigen::VectorXf const Positions, Eigen::VectorXf const y, float const dt) {
        std::vector<Eigen::Triplet<float>> triplets;
        for(const auto spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = getVec3(Positions, p1) - getVec3(Positions, p0);
            glm::vec3 const e01 = glm::normalize(x01);
            glm::mat3 f(0);
            for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) 
                f[i][j] += system.Stiffness * (
                    x01[i] * x01[j] / glm::dot(x01, x01) + 
                    (1 - spring.RestLength / glm::length(x01)) * ((i == j) - x01[i] * x01[j] / glm::dot(x01, x01))
                );
            for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) {
                triplets.emplace_back(p0 * 3 + i, p0 * 3 + j, f[i][j]);
                triplets.emplace_back(p0 * 3 + i, p1 * 3 + j, -f[i][j]);
                triplets.emplace_back(p1 * 3 + i, p0 * 3 + j, -f[i][j]);
                triplets.emplace_back(p1 * 3 + i, p1 * 3 + j, +f[i][j]);
            }
        }
        for(int i = 0; i < Positions.size() * 3; i++) {
            triplets.emplace_back(i, i, system.Mass / (dt * dt));
        }
        return CreateEigenSparseMatrix(Positions.size() * 3, triplets);
    }

    void integrateOriginal(MassSpringSystem & system, float const dt, int iteration_num) {
        int steps = iteration_num * 100;
        float const ddt = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            for (auto const spring : system.Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }
        }
    }

    void UpdateSystem(MassSpringSystem & system, Eigen::VectorXf const & x, Eigen::VectorXf const & x_origin, float const dt) {
        std::vector<glm::vec3> newV = eigen2glm((x - x_origin) / dt);
        std::vector<glm::vec3> newX = eigen2glm(x);
        for(int i = 0; i < system.Positions.size(); i++) {
            if(system.Fixed[i]) continue;
            system.Positions[i] = newX[i];
            system.Velocities[i] = newV[i];
            // printf("%.3f %.3f %.3f\n", newX[i].x, newX[i].y, newX[i].z);
        }
    }
    void integrateNewtonDescent(MassSpringSystem & system, float const dt, int iteration_num) {
        Eigen::VectorXf x_origin = glm2eigen(system.Positions);
        Eigen::VectorXf y = calc_y(system, glm2eigen(system.Positions), dt);
        Eigen::VectorXf x = y;
        float g = calc_g(system, x, y, dt);
        int numIter = iteration_num;
        for(int k = 1; k < numIter; k++) {
            Eigen::VectorXf delta_g = calc_dg(system, x, y, dt);
            Eigen::SparseMatrix delta_g2 = calc_ddg(system, glm2eigen(system.Positions), y, dt);
            Eigen::VectorXf delta_x = ComputeSimplicialLLT(delta_g2, -delta_g);
            float beta = 0.8, alpha = 1 / beta, g_new = g, gamma = 0.001;
            Eigen::VectorXf x_new = x; //, y_new = y;
            do {
                alpha *= beta;
                x_new = x + alpha * delta_x;
                // y_new = calc_y(system, x_new, dt); 
                g_new = calc_g(system, x_new, y, dt);
            } while(g_new > g + alpha * gamma * delta_g.dot(delta_x));
            x = x_new;
            g = g_new;
            // y = y_new;
        }
        UpdateSystem(system, x, x_origin, dt);
    }

    bool seted = false;
    Eigen::SparseMatrix<float> A;
    Eigen::SparseMatrix<float> J;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<float>, Eigen::Upper> prefactored_solver;
    Eigen::VectorXf m_external_force;
    Eigen::SparseMatrix<float> m_identity_matrix;
    Eigen::SparseMatrix<float> m_weighted_laplacian;
    Eigen::SparseMatrix<float> m_mass_matrix;

    void setWeightedLaplacianMatrix(MassSpringSystem & system) {
        m_weighted_laplacian.resize(system.Positions.size()*3, system.Positions.size()*3);
        std::vector<Eigen::Triplet<float>> l_triplets;
        l_triplets.clear();
        for (auto it = system.Springs.begin(); it != system.Springs.end(); ++it) {
            float ks = system.Stiffness;
            int m_p1 = it->AdjIdx.first, m_p2 = it->AdjIdx.second;
            l_triplets.emplace_back(3*m_p1+0, 3*m_p1+0, ks);
            l_triplets.emplace_back(3*m_p1+1, 3*m_p1+1, ks);
            l_triplets.emplace_back(3*m_p1+2, 3*m_p1+2, ks);
            // block 1 2
            l_triplets.emplace_back(3*m_p1+0, 3*m_p2+0, -ks);
            l_triplets.emplace_back(3*m_p1+1, 3*m_p2+1, -ks);
            l_triplets.emplace_back(3*m_p1+2, 3*m_p2+2, -ks);
            // block 2 1
            l_triplets.emplace_back(3*m_p2+0, 3*m_p1+0, -ks);
            l_triplets.emplace_back(3*m_p2+1, 3*m_p1+1, -ks);
            l_triplets.emplace_back(3*m_p2+2, 3*m_p1+2, -ks);
            // block 2 2
            l_triplets.emplace_back(3*m_p2+0, 3*m_p2+0, ks);
            l_triplets.emplace_back(3*m_p2+1, 3*m_p2+1, ks);
            l_triplets.emplace_back(3*m_p2+2, 3*m_p2+2, ks);
        }
        m_weighted_laplacian.setFromTriplets(l_triplets.begin(), l_triplets.end());
    }
    void setJMatrix(MassSpringSystem & system, Eigen::SparseMatrix<float> &J) {
        J.resize(system.Positions.size()*3, system.Springs.size()*3);
        std::vector<Eigen::Triplet<float>> J_triplets;
        J_triplets.clear();

        for (unsigned int index = 0; index < system.Springs.size(); ++index)
        {
            float ks = system.Stiffness;
            int m_p1 = system.Springs[index].AdjIdx.first, m_p2 = system.Springs[index].AdjIdx.second;
            // block 1 1
            J_triplets.emplace_back(3*m_p1+0, 3*index+0, ks);
            J_triplets.emplace_back(3*m_p1+1, 3*index+1, ks);
            J_triplets.emplace_back(3*m_p1+2, 3*index+2, ks);
            // block 2 2
            J_triplets.emplace_back(3*m_p2+0, 3*index+0, -ks);
            J_triplets.emplace_back(3*m_p2+1, 3*index+1, -ks);
            J_triplets.emplace_back(3*m_p2+2, 3*index+2, -ks);
        }
        J.setFromTriplets(J_triplets.begin(), J_triplets.end());
    }
    void factorizeDirectSolverLLT(const Eigen::SparseMatrix<float>& A, Eigen::SimplicialLLT<Eigen::SparseMatrix<float>, Eigen::Upper>& lltSolver) {
        Eigen::SparseMatrix<float> A_prime = A;
        lltSolver.analyzePattern(A_prime);
        lltSolver.factorize(A_prime);
        float Regularization = 0.00001;
        bool success = true;
        while (lltSolver.info() != Eigen::Success) {
            Regularization *= 10;
            A_prime = A_prime + Regularization*m_identity_matrix;
            lltSolver.factorize(A_prime);
            success = false;
        }
        if (!success)
            std::cout << "Warning: " <<  " adding "<< Regularization <<" identites.(llt solver)" << std::endl;
    }
    void prefactorize(MassSpringSystem & system, float dt, bool &reseted) {
        if(reseted || !seted) {
            m_identity_matrix = Eigen::SparseMatrix<float>(system.Positions.size()*3, system.Positions.size()*3);
            m_mass_matrix = Eigen::SparseMatrix<float>(system.Positions.size()*3, system.Positions.size()*3);
            // Set Identity_matrix
            std::vector<Eigen::Triplet<float>> i_triplets, m_triplets;
            for(int i = 0; i < system.Positions.size()*3; i++) {
                i_triplets.emplace_back(i, i, 1);
                m_triplets.emplace_back(i, i, system.Mass);
            }
            m_identity_matrix.setFromTriplets(i_triplets.begin(), i_triplets.end());
            m_mass_matrix.setFromTriplets(m_triplets.begin(), m_triplets.end());
            m_external_force.resize(system.Positions.size()*3);
            m_external_force.setZero();
            for (unsigned int i = 0; i < system.Positions.size(); ++i) {
                m_external_force[3*i+1] += -system.Gravity;// * system.Mass;
            }
            // printf("set external force\n");
            setWeightedLaplacianMatrix(system);
            // printf("set laplacian\n");
            setJMatrix(system, J);
            // printf("set J\n");
            A = m_weighted_laplacian * dt * dt + m_mass_matrix;
            factorizeDirectSolverLLT(A, prefactored_solver);
            // reseted = false;
            // seted = true;
            // printf("prefactorize\n");
        }
    }
    void evaluateDVector(MassSpringSystem & system, const Eigen::VectorXf& x, Eigen::VectorXf& d) {
        d.resize(system.Springs.size()*3);
        d.setZero();

        for (unsigned int index = 0; index < system.Springs.size(); ++index) {
            auto sp = system.Springs[index];
            glm::vec3 x_ij = getVec3(x, sp.AdjIdx.first) - getVec3(x, sp.AdjIdx.second);
            glm::vec3 di = glm::normalize(x_ij) * sp.RestLength;
            setVec3(d, index, di);
        }
    }

    Eigen::VectorXf calculateInertiaY(MassSpringSystem & system, float const dt) {
        return glm2eigen(system.Positions) + glm2eigen(system.Velocities) * dt;
    }

    void integrateGlobal_Local(MassSpringSystem & system, float const dt, int iteration_num, bool &reseted) {
        Eigen::VectorXf y = calculateInertiaY(system, dt);
        Eigen::VectorXf x_origin = glm2eigen(system.Positions);
        Eigen::VectorXf x_next = y;
        for (unsigned int iter = 0; iter < iteration_num; ++iter) {
            prefactorize(system, dt, reseted);
            // printf("prefactorize done\n");
            Eigen::VectorXf d;
            evaluateDVector(system, x_next, d);
            Eigen::VectorXf b = system.Mass * y + dt*dt*(J*d+m_external_force);
            x_next = prefactored_solver.solve(b);
        }
        UpdateSystem(system, x_next, x_origin, dt);
    } 

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt, CaseMassSpring::AlgorithmType _algType, int iteration_num, bool &reseted) {
        switch(_algType) {
            case(CaseMassSpring::AlgorithmType::Original):
                integrateOriginal(system, dt, iteration_num);
                break;
            case(CaseMassSpring::AlgorithmType::NewtonDescent):
                integrateNewtonDescent(system, dt, iteration_num);
                break;
            case(CaseMassSpring::AlgorithmType::Global_Local):
                integrateGlobal_Local(system, dt, iteration_num, reseted);
                break;
        }
    }
}
