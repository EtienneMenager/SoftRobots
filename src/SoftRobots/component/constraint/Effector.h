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
#ifndef SOFA_CORE_BEHAVIOR_EFFECTOR_H
#define SOFA_CORE_BEHAVIOR_EFFECTOR_H

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>
#include <SoftRobots/component/initSoftRobots.h>

namespace sofa
{

namespace core
{

namespace behavior
{

/**
 *  \brief This class specifies the type of the constraint as effector.
 */

template<class DataTypes>
class Effector : virtual public SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(Effector,DataTypes), SOFA_TEMPLATE(SoftRobotsConstraint, DataTypes));
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;

    //////////////////////////////// Come from Base /////////////////////////////////
    std::string getTemplateName() const override;
    /////////////////////////////////////////////////////////////////////////////////

    static std::string templateName(const Effector<DataTypes>* = nullptr);

    Effector(MechanicalState<DataTypes> *mm = nullptr);
    ~Effector() override;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using SoftRobotsBaseConstraint::m_constraintType ;
    using SoftRobotsBaseConstraint::EFFECTOR ;
    ////////////////////////////////////////////////////////////////////////////

protected:

    Data<bool>   d_limitShiftToTarget;
    Data<Real>   d_maxShiftToTarget;

    SReal getTarget(const Real& target, const Real& current);
    Coord getTarget(const Coord& target, const Coord& current);
};

// Force template specialization for the most common sofa float related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class SOFA_SOFTROBOTS_API Effector<defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API Effector<defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_API Effector<defaulttype::Vec1Types>;
extern template class SOFA_SOFTROBOTS_API Effector<defaulttype::Rigid3Types>;

} // namespace behavior

} // namespace core

} // namespace sofa

#endif
