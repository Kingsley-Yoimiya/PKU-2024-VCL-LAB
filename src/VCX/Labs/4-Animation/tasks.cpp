#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index = 0;
        for (int i = 0; i < nums; i++) {
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
        }
        custom->resize(index);
        return custom;
    }

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

    std::vector<glm::vec3> vecadd(std::vector<glm::vec3> a, std::vector<glm::vec3> b, float rate = 1.f) {
        for(int i = 0; i < a.size() && i < b.size(); i++) { // size a should equal to size b
            a[i] += b[i] * rate;
        }
        return a;
    }

    float vecdot(std::vector<glm::vec3> a, std::vector<glm::vec3> b) {
        float ret = 0;
        for(int i = 0; i < a.size() && i < b.size(); i++) {
            ret += glm::dot(a[i], b[i]);
        }
        return ret;
    }

    std::vector<glm::vec3> calc_y(MassSpringSystem & system, std::vector<glm::vec3> Positions, float dt) {
        std::vector<glm::vec3> y(Positions.size(), glm::vec3());
        for(int i = 0; i < Positions.size(); i++) {
            y[i] = Positions[i] + dt * system.Velocities[i] + 
                dt * dt / system.Mass * glm::vec3(0, -system.Gravity, 0);
        }
        return y;
    }

    float calc_g(MassSpringSystem & system, std::vector<glm::vec3> Positions, float dt) {
        std::vector<glm::vec3> y    = calc_y(system, Positions, dt);
        std::vector<glm::vec3> tPos = eigen2glm(glm2eigen(Positions) - glm2eigen(y));
        float ret = vecdot(tPos, tPos) / 2 / dt / dt;
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 x0 = Positions[p0], x1 = Positions[p1];
            ret += 1. / 2 * system.Stiffness * pow(glm::length(x0 - x1) - spring.RestLength, 2);
        }
        return ret;
    }

    Eigen::VectorXf calc_dg(MassSpringSystem & system, std::vector<glm::vec3> Positions, float dt) {
        auto y = calc_y(system, Positions, dt);
        std::vector<glm::vec3> dg(Positions.size(), glm::vec3(0));
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 x0 = Positions[p0], x1 = Positions[p1];
            dg[p0] += system.Stiffness * (glm::length(x1 - x0) - spring.RestLength) * glm::normalize(x1 - x0);
            dg[p1] += system.Stiffness * (glm::length(x1 - x0) - spring.RestLength) * glm::normalize(x0 - x1);
        }
        for(int i = 0; i < Positions.size(); i++) {
            dg[i] += 1. / dt / dt * system.Mass * (Positions[i] - y[i]);
            dg[i] += glm::vec3(0, -system.Gravity, 0);// f_ext
        }
        return glm2eigen(dg);
    }

    static Eigen::SparseMatrix<float> calc_Hessian(MassSpringSystem & system) {
        std::vector<Eigen::Triplet<float>> Htriplets;
        std::vector<glm::mat3> Hidentity(system.Positions.size(), glm::mat3(0));
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 x0 = system.Positions[p0], x1 = system.Positions[p1];
            glm::mat3 H_e(0);
            float dfrac = glm::dot(x0 - x1, x0 - x1);
            for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) 
                H_e[j][i] += system.Stiffness * (
                    (x0[i] - x1[i]) * (x0[j] - x1[j]) / dfrac + 
                    (1 - spring.RestLength / glm::length(x0 - x1)) * ((i == j) - (x0[i] - x1[i]) * (x0[j] - x1[j]) / dfrac)
                );
            for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) {
                // Htriplets.emplace_back(p0 * 3 + i, p0 * 3 + j, H_e[j][i]);
                Htriplets.emplace_back(p0 * 3 + i, p1 * 3 + j, -H_e[j][i]);
                Htriplets.emplace_back(p1 * 3 + i, p0 * 3 + j, -H_e[j][i]);
                // Htriplets.emplace_back(p1 * 3 + i, p1 * 3 + j, H_e[j][i]);
            }
            Hidentity[p0] += H_e;
            Hidentity[p1] += H_e;
        }
        for(int x = 0; x < Hidentity.size(); x++) 
            for(int i = 0; i < 3; i++) for(int j = 0; j < 3; j++) {
                Htriplets.emplace_back(x * 3 + i, x * 3 + j, -Hidentity[x][j][i]);
            }
        return CreateEigenSparseMatrix(3 * system.Positions.size(), Htriplets);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        /*
        int const steps = 1000;
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
        */
        auto zero = std::vector<glm::vec3>(system.Positions.size(), glm::vec3(0, -system.Gravity, 0));
        auto g = calc_g(system, system.Positions, dt);
        auto dg = calc_dg(system, system.Positions, dt);
        auto H = calc_Hessian(system);
        auto PosDelta = eigen2glm(
            ComputeSimplicialLLT(H, -dg)
        );
        float alpha = 1, beta = 0.95, gamma = 0.0001, evalg = 0;
        auto nPositions = system.Positions;
        int cnt = 0;
        do {
            nPositions = vecadd(system.Positions, PosDelta, alpha);
            evalg = calc_g(system, nPositions, dt);
            alpha = alpha * beta; cnt++;
        } while(cnt <= 10 && evalg <= g + gamma * alpha * vecdot(eigen2glm(dg), PosDelta));
        // update system info.
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0)); // forces ext
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 x0 = nPositions[p0], x1 = nPositions[p1];
            forces[p0] += system.Stiffness * (glm::length(x0 - x1) - spring.RestLength) * glm::normalize(x1 - x0);
            forces[p1] += system.Stiffness * (glm::length(x0 - x1) - spring.RestLength) * glm::normalize(x0 - x1);
        }
        for(int i = 0; i < system.Positions.size(); i++) {
            if(system.Fixed[i]) continue;
            system.Velocities[i] += dt * ((forces[i] / system.Mass) + glm::vec3(0, -system.Gravity, 0));
            system.Positions[i] = nPositions[i];
        }
    }
}
