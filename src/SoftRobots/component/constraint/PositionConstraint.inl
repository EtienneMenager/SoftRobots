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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_POSITIONCONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINTSET_POSITIONCONSTRAINT_INL

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/logging/Messaging.h>

#include "PositionConstraint.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::VecCoordId;
using sofa::core::ConstVecCoordId ;
using sofa::helper::WriteAccessor ;
using sofa::helper::ReadAccessor ;
using sofa::type::vector ;
using type::Vec;
using type::Vector3;
using sofa::type::RGBAColor;
using sofa::helper::OptionsGroup;

template<class DataTypes>
PositionConstraint<DataTypes>::PositionConstraint(MechanicalState* mm)
    : SoftRobotsConstraint<DataTypes>(mm)

    , d_limitShiftToTarget(initData(&d_limitShiftToTarget, false, "limitShiftToTarget", "If true will limit the effector goal to be at \n"
                                                                                            "maxShiftToTarget."))
    , d_maxShiftToTarget(initData(&d_maxShiftToTarget, Real(1.), "maxShiftToTarget", "Maximum shift to effector goal if limitShiftToTarget \n"
                                                                                         "is set to true."))
    , d_indices(initData(&d_indices, "indices",
                                 "If indices size is lower than effectorGoal size, \n"
                                 "some effectorGoal will not be considered"))

    , d_effectorGoalPositions(initData(&d_effectorGoalPositions,"effectorGoal",
                                       "Desired positions for the effectors. \n"
                                       "If the size does not match with the size of effector indices, \n"
                                       "one will resize considerering the smallest one."))

    , d_directions(initData(&d_directions,"directions",
                          "The parameter directions allows to specify the directions in \n"
                          "which you want to solve the effector."))

    , d_useDirections(initData(&d_useDirections,"useDirections",
                              "The parameter useDirections allows to select the directions in \n"
                              "which you want to solve the effector. If unspecified, the default \n"
                              "values are all true."))

    , d_delta(initData(&d_delta, "delta","Distance to target"))
    , d_value(initData(&d_value,"imposedValue",
                          "Parameter to impose the force (with value_type = force) or to impose relative displacement (with value_type = displacement)."))

    , d_valueType(initData(&d_valueType, OptionsGroup(2,"displacement","force"), "valueType",
                               "displacement = the contstraint will impose the relative displacement provided in data value[valueIndex] \n"
                               "force = the contstraint will impose the force provided in data value[valueIndex] \n"
                               "If unspecified, the default value is displacement"))

    , m_nbEffector(0)
{
    d_indices.setGroup("Vector");
    d_effectorGoalPositions.setGroup("Vector");
    d_delta.setGroup("Vector");

    d_delta.setReadOnly(true);
    m_constraintType = ACTUATOR;
}


template<class DataTypes>
PositionConstraint<DataTypes>::~PositionConstraint()
{
}


template<class DataTypes>
std::string PositionConstraint<DataTypes>::getTemplateName() const
{
    return templateName(this);
}

template<class DataTypes>
std::string PositionConstraint<DataTypes>::templateName(const PositionConstraint<DataTypes>*)
{
    return DataTypes::Name();
}


template<class DataTypes>
SReal PositionConstraint<DataTypes>:: getTarget(const Real& target, const Real& current)
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
typename DataTypes::Coord PositionConstraint<DataTypes>:: getTarget(const Coord& target, const Coord& current)
{
    Coord newTarget = target;
    for(int i=0; i<DataTypes::Coord::total_size; i++)
        newTarget[i]=getTarget(target[i], current[i]);

    return newTarget;
}



template<class DataTypes>
void PositionConstraint<DataTypes>::init()
{
    d_componentState = ComponentState::Valid;
    Inherit1::init();

    if(m_state==nullptr)
    {
        msg_error() << "There is no mechanical state associated with this node. "
                        "the object is deactivated. "
                        "To remove this error message fix your scene possibly by "
                        "adding a MechanicalObject." ;
        d_componentState = ComponentState::Invalid;
        return;
    }

    internalInit();
}


template<class DataTypes>
void PositionConstraint<DataTypes>::reinit()
{
    internalInit();
}


template<class DataTypes>
void PositionConstraint<DataTypes>::internalInit()
{
    if(!d_directions.isSet())
        setDefaultDirections();
    else
        normalizeDirections();


    if(!d_useDirections.isSet())
    {
        setDefaultUseDirections();
    }
    else
    {
        int count = 0;
        for(int i=0; i<Deriv::total_size; i++)
            if(d_useDirections.getValue()[i])
                count++;

        if(count==0)
        {
            setDefaultUseDirections();
            msg_warning(this) << "No direction given in useDirection. Set default all.";
        }
    }

    if(!d_effectorGoalPositions.isSet())
    {
        msg_warning(this) <<"EffectorGoal not defined. Default value assigned  ("<<Coord()<<").";
        setEffectorGoalDefaultValue();
    }

    if(!d_indices.isSet())
    {
        msg_warning(this) <<"Indices not defined. Default value assigned 0.";
        setEffectorIndicesDefaultValue();
    }

    if(d_indices.getValue().size() > m_state->getSize())
    {
        msg_warning(this) <<"Indices size can not be larger than the number of point in the context. Launch resize process.";
        resizeIndicesRegardingState();
    }

    if(d_indices.getValue().size() != d_effectorGoalPositions.getValue().size())
        resizeEffectorData();

    if(d_indices.getValue().size() == 0)
    {
        msg_error(this) <<"Indices size is zero. The component will not work.";
        d_componentState = ComponentState::Invalid;
        return;
    }

    m_nbEffector = d_indices.getValue().size();

    checkIndicesRegardingState();

    // Save initial positions
    ReadAccessor<Data<VecCoord>> restPositions = m_state->readRestPositions();
    const SetIndexArray &indices = d_indices.getValue();
    m_initialPositions.resize(indices.size());
    for(int i=0; i< m_nbEffector; i++)
    {
        m_initialPositions[i] = restPositions[indices[i]];
    }
  

}


template<class DataTypes>
void PositionConstraint<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();

    if(d_indices.getValue().size() > positions.size())
    {
        msg_error(this) << "Indices size is larger than mechanicalState size" ;
        d_componentState = ComponentState::Invalid;
        return;
    }

    for(unsigned int i=0; i<d_indices.getValue().size(); i++)
    {
        if (positions.size() <= d_indices.getValue()[i])
        {
            msg_error(this) << "Indices at index " << i << " is to large regarding mechanicalState [position] size" ;
            d_componentState = ComponentState::Invalid;
            return;
        }
        if (d_indices.getValue()[i] < 0)
        {
            msg_error(this) << "Indices at index " << i << " is negative" ;
            d_componentState = ComponentState::Invalid;
            return;
        }
    }
}


template<class DataTypes>
void PositionConstraint<DataTypes>::setEffectorGoalDefaultValue()
{
    WriteAccessor<Data<VecCoord> > defaultEffectorGoal = d_effectorGoalPositions;
    defaultEffectorGoal.resize(1);
    defaultEffectorGoal[0] = Coord();
}


template<class DataTypes>
void PositionConstraint<DataTypes>::setEffectorIndicesDefaultValue()
{
    WriteAccessor<Data<vector<unsigned int> > > defaultEffectorIndices = d_indices;
    defaultEffectorIndices.resize(1, 0);
}

template<class DataTypes>
void PositionConstraint<DataTypes>::resizeIndicesRegardingState()
{
    WriteAccessor<Data<vector<unsigned int>>> effectorIndices = d_indices;
    effectorIndices.resize(m_state->getSize());
}

template<class DataTypes>
void PositionConstraint<DataTypes>::resizeEffectorData()
{
    if(d_indices.getValue().size() < d_effectorGoalPositions.getValue().size())
    {
        msg_warning(this)<<"Indices size is lower than EffectorGoal size, some effectorGoal will not be considered.";
    }
    else
    {
        msg_warning(this) <<"Indices size is larger than EffectorGoal size. Launch resize process.";
        WriteAccessor<Data<vector<unsigned int> > > effectorIndices = d_indices;
        effectorIndices.resize(d_effectorGoalPositions.getValue().size());
    }
}




template<class DataTypes>
void PositionConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                        DataMatrixDeriv &cMatrix,
                                                        unsigned int &cIndex,
                                                        const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    m_constraintId = cIndex;
    MatrixDeriv& column = *cMatrix.beginEdit();

    unsigned int index = 0;
    for (unsigned int i=0; i<m_nbEffector; i++)
    {
        for(unsigned int j=0; j<Deriv::total_size; j++)
        {
            if(d_useDirections.getValue()[j])
            {
                MatrixDerivRowIterator rowIterator = column.writeLine(m_constraintId+index);
                rowIterator.setCol(d_indices.getValue()[i],  d_directions.getValue()[j]);
                index++;
            }
        }
    }

    cIndex += index;
    cMatrix.endEdit();

    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void PositionConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                         BaseVector *resV,
                                                         const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);

    ReadAccessor<Data<VecCoord> > x = m_state->readPositions();

    int index = 0;
    for (unsigned int i=0; i<m_nbEffector; i++)
    {
        Coord pos = x[d_indices.getValue()[i]];
        Coord effectorGoalPos = getTarget(d_effectorGoalPositions.getValue()[i],pos);

        Deriv d = DataTypes::coordDifference(pos,effectorGoalPos);

        for(int j=0; j<DataTypes::Deriv::total_size; j++)
            if(d_useDirections.getValue()[j])
            {
                Real dfree = Jdx->element(index) + d*d_directions.getValue()[j];
                resV->set(m_constraintId+index, dfree);
                index++;
            }
    }
}


template<class DataTypes>
void PositionConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParam,
                                                         std::vector<ConstraintResolution*>& resTab,
                                                         unsigned int& offset)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParam);

    if(d_valueType.getValue().getSelectedItem() == "displacement") // displacement
    {   
        /**/
        double x = m_initialPositions[0][0] + d_value.getValue()[0];
        double y = m_initialPositions[0][1] + d_value.getValue()[1];
        double z = m_initialPositions[0][2] + d_value.getValue()[2];
        /**/
        /*
        double x = d_value.getValue()[0];
        double y = d_value.getValue()[1];
        double z = d_value.getValue()[2];
        */

        PositionDisplacementConstraintResolution *cr=  new PositionDisplacementConstraintResolution(x, y, z, m_nbLines);
        resTab[offset] =cr;
        offset+=3;
    }
    else // force
    {
        double fx =d_value.getValue()[0];
        double fy =d_value.getValue()[1];
        double fz =d_value.getValue()[2];

        PositionForceConstraintResolution *cr=  new PositionForceConstraintResolution(fx, fy, fz, m_nbLines);
        resTab[offset] =cr;
        offset+=3;
    }
    

}


template<class DataTypes>
void PositionConstraint<DataTypes>::storeResults(vector<double> &delta)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    d_delta.setValue(delta);
}


template<class DataTypes>
void PositionConstraint<DataTypes>::setDefaultDirections()
{
    VecDeriv directions;
    directions.resize(Deriv::total_size);
    for(int i=0; i<Deriv::total_size; i++)
        directions[i][i] = 1.;
    d_directions.setValue(directions);
}


template<class DataTypes>
void PositionConstraint<DataTypes>::setDefaultUseDirections()
{
    Vec<Deriv::total_size,bool> useDirections;
    for(int i=0; i<Deriv::total_size; i++)
        useDirections[i] = true;
    d_useDirections.setValue(useDirections);
}


template<class DataTypes>
void PositionConstraint<DataTypes>::normalizeDirections()
{
    WriteAccessor<Data<VecDeriv>> directions = d_directions;
    directions.resize(Deriv::total_size);
    for(unsigned int i=0; i<Deriv::total_size; i++)
        directions[i].normalize();
}


template<class DataTypes>
void PositionConstraint<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    if (!vparams->displayFlags().getShowInteractionForceFields())
        return;

    vector<Vector3> points;
    ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();
    for (unsigned int i=0; i<d_indices.getValue().size(); i++)
    {
        points.push_back(positions[d_indices.getValue()[i]]);
        points.push_back(d_effectorGoalPositions.getValue()[i]);
    }
    vparams->drawTool()->drawPoints(points,10.0f,RGBAColor(0.,1.,0.,1.));
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
