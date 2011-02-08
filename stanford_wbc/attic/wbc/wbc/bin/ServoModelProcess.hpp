/*
 * Copyright (c) 2009 Stanford University
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

/**
   \file ServoModelProcess.hpp
   \author Roland Philippsen
*/

#ifndef WBC_SERVO_MODEL_PROCESS_HPP
#define WBC_SERVO_MODEL_PROCESS_HPP

#include <wbc/bin/ServoProcess.hpp>
#include <wbc/bin/ModelProcess.hpp>

namespace wbc {
  
  class ServoModelProcess
    : public Process,
      public ServoProcessAPI
  {
  public:
    ServoModelProcess();
    ~ServoModelProcess();
    
    int HandleMessagePayload(wbcnet::unique_id_t msg_id);
    
    bool Step() throw(std::exception);
    
    void Init(ServoImplementation * servo_imp,
	      /** if true, the servo_imp will be deleted in our destructor */
	      bool own_servo_imp,
	      ModelImplementation * model_imp,
	      /** if true, the model_imp will be deleted in our destructor */
	      bool own_model_imp,
	      /** only for connecting to the user process, no explicit
		  servo/model communication */
	      wbcnet::NetConfig const & netconf,
	      uint8_t npos, uint8_t nvel,
	      uint8_t force_nrows, uint8_t force_ncols) throw(std::exception); 
    
    virtual wbcnet::srv_result_t BeginBehaviorTransition(int behaviorID);
    virtual BranchingRepresentation * GetBranching();
    virtual Kinematics * GetKinematics();
    virtual SAIVector const & GetCommandTorques();
    virtual BehaviorDescription * GetCurrentBehavior();
    
  protected:
    friend class ModelServoTest;
    
    typedef enum {
      READY_STATE,	       /**< initialized, but no behavior chosen yet */
      RUNNING_STATE,	       /**< running a behavior */
      ERROR_STATE	       /**< placeholder for later extension */
    } state_t;
    
    DirectoryCmdServer * m_directory_cmd_server;
    
    ServoImplementation * m_servo_imp;
    bool m_own_servo_imp;
    ModelImplementation * m_model_imp;
    bool m_own_model_imp;
    
    state_t m_state;
    int m_behaviorID;
    bool m_have_behaviorID;
    
    // not really messages anymore, just shared between model and servo
    msg::RobotState * m_robot_state;
    
    wbcnet::Channel * m_user_channel;
    
    // incoming messages
    msg::TaskSpec m_user_task_spec;
    wbcnet::msg::Service m_user_request;
    
    // outgoing messages
    wbcnet::msg::Service m_user_reply;
  };
  
}

#endif // WBC_SERVO_MODEL_PROCESS_HPP
