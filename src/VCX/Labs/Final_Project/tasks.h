#pragma once

#include "Labs/Final_Project/MassSpringSystem.h"
#include "Labs/Final_Project/CaseMassSpring.h"

namespace VCX::Labs::Animation {
    // lab4 mass spring system
    void AdvanceMassSpringSystem(MassSpringSystem &, float const, CaseMassSpring::AlgorithmType, bool &);
}
