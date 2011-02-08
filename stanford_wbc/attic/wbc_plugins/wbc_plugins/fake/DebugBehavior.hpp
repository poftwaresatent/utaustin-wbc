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

/**
   \file DebugBehavior.hpp
   \author Roland Philippsen
*/

#ifndef FAKE_DEBUG_BEHAVIOR_HPP
#define FAKE_DEBUG_BEHAVIOR_HPP

#ifndef WIN32
#include <stdint.h>
#else
#include "wbcnet/win32/win32_compat.hpp"
#endif

#include <wbc/core/BehaviorDescription.hpp>
#include <wbc/core/TaskSet.hpp>
#include <wbc/motion/FrictionPosture.hpp>

namespace wbc_fake_plugin {
  
using namespace wbc;

  /** A "behavior" that just floats the robot and prints messages to
      stdout whenever handleKey() gets called. */
class DebugBehavior
  : public BehaviorDescription
{
public:
  DebugBehavior();
  
  virtual TaskSet * activeTaskSet();
  virtual void onUpdate();
  
  virtual int handleKey(int keycode);
  
protected:
  virtual void loadMovementPrimitives( RobotControlModel* ) throw(std::runtime_error);
  
private:
  TaskSet taskSet_;
  FrictionPosture friction_posture_;
};

}

#endif // FAKE_DEBUG_BEHAVIOR_HPP
