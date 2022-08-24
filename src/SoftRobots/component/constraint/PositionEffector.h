/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Modules                               *
*                                                                             *
* This component is not open-source                                           *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINTSET_POSITIONEFFECTOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_POSITIONEFFECTOR_H

#include "Effector.h"
#include <SoftRobots/component/initSoftRobots.h>

namespace sofa
{

namespace component
{

namespace constraintset
{
using sofa::core::behavior::Effector ;
using sofa::core::ConstraintParams ;
using sofa::linearalgebra::BaseVector ;
using sofa::type::Vec ;
using sofa::core::visual::VisualParams ;

/**
 * The "PositionEffector" component is used to constrain one or several points of a model
 * to reach desired positions, by acting on chosen actuator(s).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class PositionEffector : public Effector<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PositionEffector,DataTypes), SOFA_TEMPLATE(Effector,DataTypes));

    typedef typename DataTypes::VecCoord            VecCoord;
    typedef typename DataTypes::VecDeriv            VecDeriv;
    typedef typename DataTypes::Coord               Coord;
    typedef typename DataTypes::Deriv               Deriv;
    typedef typename DataTypes::MatrixDeriv         MatrixDeriv;
    typedef typename Coord::value_type              Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>                          DataVecCoord;
    typedef Data<VecDeriv>                          DataVecDeriv;
    typedef Data<MatrixDeriv>                       DataMatrixDeriv;

public:
    PositionEffector(MechanicalState* object = nullptr);
    ~PositionEffector() override;

    /////////////// Inherited from BaseObject  ////////////
    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;
    //////////////////////////////////////////////////////

    /////////////// Inherited from Effector ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    ///////////////////////////////////////////////////////////////


    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(type::vector<double> &delta) override;
    ///////////////////////////////////////////////////////////////////////////

protected:
    Data<type::vector<unsigned int> >             d_indices;
    Data<VecCoord>                                  d_effectorGoalPositions;
    Data<VecDeriv>                                  d_directions;
    Data<Vec<Deriv::total_size,bool>>               d_useDirections;
    Data<type::vector<double>>                    d_delta;

    unsigned int                                    m_nbEffector;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Effector<DataTypes>::m_nbLines ;
    using Effector<DataTypes>::m_constraintId ;
    using Effector<DataTypes>::d_componentState ;
    using Effector<DataTypes>::getTarget ;
    ////////////////////////////////////////////////////////////////////////////

    void setDefaultDirections();
    void setDefaultUseDirections();
    void normalizeDirections();

private:
    void internalInit();
    void checkIndicesRegardingState();
    void setEffectorGoalDefaultValue();
    void setEffectorIndicesDefaultValue();
    void resizeIndicesRegardingState();
    void resizeEffectorData();

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Effector<DataTypes>::addAlias ;
    using Effector<DataTypes>::m_state ;
    ////////////////////////////////////////////////////////////////////////////
};



template<> SOFA_SOFTROBOTS_API
void PositionEffector<defaulttype::Rigid3Types>::normalizeDirections();

template<> SOFA_SOFTROBOTS_API
void PositionEffector<defaulttype::Rigid3Types>::draw(const VisualParams* vparams);



// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_POSITIONEFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_API PositionEffector<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API PositionEffector<sofa::defaulttype::Rigid3Types>;
#endif



} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_POSITIONEFFECTOR_H
