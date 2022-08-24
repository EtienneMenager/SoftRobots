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
#define SOFTROBOTS_POSITIONEFFECTOR_CPP

#include "PositionEffector.inl"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using namespace sofa::defaulttype;
using core::ConstraintParams;


//----------- Effector Constraint  Resolution --------------
EffectorConstraintResolution::EffectorConstraintResolution(unsigned int nbLines)
    : sofa::core::behavior::ConstraintResolution(nbLines)
    , nbLines(nbLines)
{ }

void EffectorConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(dfree);
    SOFA_UNUSED(w);
    SOFA_UNUSED(d);

    for(unsigned int i = 0, i < nbLines, i++){
      lambda[line + i ] = 0.0
    }
}




template<>
void PositionEffector<Rigid3Types>::normalizeDirections()
{
    VecDeriv directions;
    directions.resize(6);
    for(unsigned int i=0; i<6; i++)
    {
        directions[i] = d_directions.getValue()[i];
        Vec<3, Real> vector1 = Vec<3, Real>(directions[i][0],directions[i][1],directions[i][2]);
        Vec<3, Real> vector2 = Vec<3, Real>(directions[i][3],directions[i][4],directions[i][5]);
        vector1.normalize();
        vector2.normalize();
        directions[i] = Deriv(vector1,vector2);
    }
    d_directions.setValue(directions);
}


template<>
void PositionEffector<Rigid3Types>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    if (!vparams->displayFlags().getShowInteractionForceFields())
        return;

    vector<Vector3> points;
    VecCoord positions = m_state->read(core::ConstVecCoordId::position())->getValue();
    for (unsigned int i=0; i<d_indices.getValue().size(); i++)
    {
        points.push_back(positions[d_indices.getValue()[i]].getCenter());
        points.push_back(d_effectorGoalPositions.getValue()[i].getCenter());
    }
    vparams->drawTool()->drawPoints(points,10.0f,RGBAColor(0.,1.,0.,1.));
}



////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
using namespace sofa::helper;

// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

volatile int PositionEffectorClass = core::RegisterObject("This component is used to describe one or several desired positions "
                                                 "of points of a model, that will be reached by acting on chosen actuator(s).")
                .add< PositionEffector<Vec3Types> >(true)
                .add< PositionEffector<Rigid3Types> >()

        ;

////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_SOFTROBOTS_API PositionEffector<sofa::defaulttype::Vec3Types>;
template class SOFA_SOFTROBOTS_API PositionEffector<sofa::defaulttype::Rigid3Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa
