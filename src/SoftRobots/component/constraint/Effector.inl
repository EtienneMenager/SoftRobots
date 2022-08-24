/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_BEHAVIOR_EFFECTOR_INL
#define SOFA_CORE_BEHAVIOR_EFFECTOR_INL

#include "Effector.h"

namespace sofa
{

namespace core
{

namespace behavior
{

template<class DataTypes>
Effector<DataTypes>::Effector(MechanicalState<DataTypes> *mm)
    : SoftRobotsConstraint<DataTypes>(mm)
    , d_limitShiftToTarget(initData(&d_limitShiftToTarget, false, "limitShiftToTarget", "If true will limit the effector goal to be at \n"
                                                                                        "maxShiftToTarget."))

    , d_maxShiftToTarget(initData(&d_maxShiftToTarget, Real(1.), "maxShiftToTarget", "Maximum shift to effector goal if limitShiftToTarget \n"
                                                                                     "is set to true."))
{
    m_constraintType = EFFECTOR;
}

template<class DataTypes>
Effector<DataTypes>::~Effector()
{
}

template<class DataTypes>
std::string Effector<DataTypes>::getTemplateName() const
{
    return templateName(this);
}

template<class DataTypes>
std::string Effector<DataTypes>::templateName(const Effector<DataTypes>*)
{
    return DataTypes::Name();
}


template<class DataTypes>
SReal Effector<DataTypes>:: getTarget(const Real& target, const Real& current)
{
    Real newTarget = target;
    if(d_limitShiftToTarget.getValue())
    {
        Real shift = abs(target-current);
        if(shift>d_maxShiftToTarget.getValue())
        {
            if(target>current)
                newTarget = current + d_maxShiftToTarget.getValue();
            else
                newTarget = current - d_maxShiftToTarget.getValue();
        }
    }

    return newTarget;
}

template<class DataTypes>
typename DataTypes::Coord Effector<DataTypes>:: getTarget(const Coord& target, const Coord& current)
{
    Coord newTarget = target;
    for(int i=0; i<DataTypes::Coord::total_size; i++)
        newTarget[i]=getTarget(target[i], current[i]);

    return newTarget;
}

} // namespace behavior

} // namespace core

} // namespace sofa

#endif // SOFA_CORE_BEHAVIOR_EFFECTOR_INL
