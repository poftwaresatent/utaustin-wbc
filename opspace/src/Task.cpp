/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen (Stanford) and Luis Sentis (UT Austin)
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
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

#include <opspace/Task.hpp>

using namespace jspace;

namespace opspace {
  
  
  Task::
  Task(std::string const & name)
    : ParameterReflection("task", name),
      sigma_threshold_(1.0e-2)
  {
    declareParameter("sigma_threshold", &sigma_threshold_);
    // these will become read-only, as soon as that is supported in
    // the Parameter interface...
    declareParameter("actual", &actual_);
    declareParameter("command", &command_);
    // overkill? declareParameter("jacobian", &jacobian_);
  }
  
  
  void Task::
  dump(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    os << prefix << "task: `" << instance_name_ << "'\n";
    ParameterReflection::dump(os, prefix + "  parameters", prefix + "    ");
    pretty_print(actual_, os, prefix + "  actual:", prefix + "    ");
    pretty_print(command_, os, prefix + "  command:", prefix + "    ");
    pretty_print(jacobian_, os, prefix + "  Jacobian:", prefix + "    ");
  }
  
  
  void Task::
  dbg(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    // too noisy...
    // os << prefix << "task " << instance_name_ << "\n";
    // ParameterReflection::dump(os, prefix + "  parameters", prefix + "    ");
  }
  
}
