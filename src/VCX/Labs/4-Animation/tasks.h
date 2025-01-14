#pragma once

#include "Labs/4-Animation/MassSpringSystem.h"
#include "Labs/4-Animation/CaseMassSpring.h"

namespace VCX::Labs::Animation {
    // lab4 mass spring system
    void AdvanceMassSpringSystem(MassSpringSystem &, float const, CaseMassSpring::AlgorithmType, bool &);
}
