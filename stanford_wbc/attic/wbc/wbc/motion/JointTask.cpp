/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

//===========================================================================
/**
   \file JointTask.cpp
   \author Luis Sentis
*/
//===========================================================================

#include <wbc/motion/JointTask.hpp>
#include <wbc/core/RobotControlModel.hpp>
#include <wbc/core/Kinematics.hpp>
#include <iostream>

namespace wbc {

  JointTask::JointTask(std::string const & name)
    : TaskDescription(name, 400, 40, 0.1, 0)
  {}


  void
  JointTask::robotControlModel( RobotControlModel* robModel )
    throw(std::runtime_error)
  {
    robModel_ = robModel;
  }

  void
  JointTask::addJoint( int jointID ) {

    SAIMatrix onerow(1,robModel_->branching()->numJoints()); 
    onerow.zero();
    onerow.elementAt(0,jointID) = 1.0;
    selection_matrix_.appendVertically(onerow);
    globalJacobian_ = selection_matrix_;
  }

  void
  JointTask::onUpdate() {

    config_ = selection_matrix_ * robModel_->kinematics()->jointPositions();
    velocity_ = selection_matrix_ * robModel_->kinematics()->jointVelocities();
    servoUpdate();
  } 

  void 
  JointTask::servoUpdate() {

    if( goalConfig_.size() == 0 ) {
      goalConfig_.setSize( config_.size() ); 
      goalConfig_.zero();
    }

    // desDiff = -( state_ - goalState_ ) * gain_
    SAIVector desDiff = ( goalConfig_ -  config_ ) * m_propGain;
  
    //goalConfig_.display("goal posture:");
    //config_.display("actual posture");

    // Saturating upon velocity limits
    if( m_diffGain != 0 ) {
      double satFactor = desDiff.abs() / ( m_maxVel * m_diffGain );
      if( satFactor > 1 ) desDiff.multiply( 1/satFactor );
    }

    // desiredAccel_ = -m_diffGain (diffState_ - \nu desDiff_ );
    //cout << "desdiff size: " << desDiff.size() << "velocity size: " << velocity_.size() << std::endl;
    commandAccel_ = desDiff - velocity_ * m_diffGain;
  
    // Saturation of acceleration.
    double satFactor2, inv_rate;
    if( fabs(m_maxAccel) > 1e-2 ) { // this is to ensure that acceleration saturation has been set up
      inv_rate =  commandAccel_.abs() / m_maxAccel ;
      if( inv_rate > 1 ) satFactor2 = 1.0 / inv_rate;
      else satFactor2 = 1.0;
      commandAccel_ *= satFactor2;
    }
  
  }

}
