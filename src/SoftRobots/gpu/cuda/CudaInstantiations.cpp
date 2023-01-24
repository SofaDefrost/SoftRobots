/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/core/ObjectFactory.h>
#include <SofaCUDA/sofa/gpu/cuda/CudaTypes.h>

#include <SoftRobots/component/forcefield/PartialRigidificationForceField.inl>
#include <SoftRobots/component/forcefield/PipeForceField.inl>
#include <SoftRobots/component/forcefield/PREquivalentStiffnessForceField.inl>

#include <sofa/core/behavior/MixedInteractionForceField.inl>

using namespace sofa::gpu::cuda;

namespace sofa::component::interactionforcefield
{
#ifdef SOFA_GPU_CUDA_DOUBLE
    template class PartialRigidificationForceField<CudaVec3dTypes, CudaRigid3dTypes>;
#endif
}

namespace sofa::component::forcefield
{
#ifdef SOFA_GPU_CUDA_DOUBLE
    template class PipeForceField<CudaVec3dTypes>;
    template class PREquivalentStiffnessForceField<CudaRigid3dTypes>;
#endif
}

namespace sofa::gpu::cuda
{
#ifdef SOFA_GPU_CUDA_DOUBLE

int PartialRigidificationForceFieldClass = core::RegisterObject("Supports GPU-side computations using CUDA")
    .add< sofa::component::interactionforcefield::PartialRigidificationForceField<CudaVec3dTypes, CudaRigid3dTypes> >();
int PipeForceFieldClass = core::RegisterObject("Supports GPU-side computations using CUDA")
    .add< sofa::component::forcefield::PipeForceField<CudaVec3dTypes> >();
int PREquivalentStiffnessForceFieldClass = core::RegisterObject("Supports GPU-side computations using CUDA")
    .add< sofa::component::forcefield::PREquivalentStiffnessForceField<CudaRigid3dTypes> >();

#endif
}
