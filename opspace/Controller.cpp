/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen and Luis Sentis
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <opspace/Controller.hpp>
#include <opspace/opspace.hpp>

namespace opspace {
  
  
  Controller::
  Controller()
    : initialized_(false)
  {
  }
  
  Controller::
  ~Controller()
  {
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      if (task_table_[ii]->controller_owned) {
	delete task_table_[ii]->task;
      }
      delete task_table_[ii];
    }
  }
  
  
  Controller::task_info_s const * Controller::
  appendTask(Task * task, bool controller_owned)
  {
    if (initialized_) {
      return 0;
    }
    task_info_s * task_info(new task_info_s(task, controller_owned, task_table_.size()));
    task_table_.push_back(task_info);
    return task_info;
  }
  
  
  Status Controller::
  init(Model const & model)
  {
    if (initialized_) {
      return Status(false, "already initialized");
    }
    if (task_table_.empty()) {
      return Status(false, "no tasks to initialize");
    }
    std::ostringstream msg;
    bool ok(true);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Task * task(task_table_[ii]->task);
      Status const st(task->init(model));
      if ( ! st) {
	msg << "  task[" << ii << "] `" << task->getName() << "': " << st.errstr << "\n";
	ok = false;
      }
    }
    if ( ! ok) {
      return Status(false, "failures during init:\n" + msg.str());
    }
    initialized_ = true;
    
    Status st;
    return st;
  }
  
  
  Status Controller::
  computeCommand(Model const & model, Vector & gamma)
  {
    if ( ! initialized_) {
      return Status(false, "not initialized");
    }
    Matrix ainv;
    if ( ! model.getInverseMassInertia(ainv)) {
      return Status(false, "failed to retrieve inverse mass inertia");
    }
    Vector grav;
    if ( ! model.getGravity(grav)) {
      return Status(false, "failed to retrieve gravity torques");
    }
    
    std::ostringstream msg;
    bool ok(true);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      Task * task(task_table_[ii]->task);
      Status const st(task->update(model));
      if ( ! st) {
	msg << "  task[" << ii << "] `" << task->getName() << "': " << st.errstr << "\n";
	ok = false;
      }
    }
    if ( ! ok) {
      return Status(false, "failures during task updates:\n" + msg.str());
    }
    
    Matrix rangespace;
    size_t ndof(model.getNDOF());
    size_t n_minus_1(task_table_.size() - 1);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      task_info_s * tinfo(task_table_[ii]);
      Matrix const & jac(tinfo->task->getJacobian());
      pseudoInverse(jac * ainv * jac.transpose(), 1e-3, tinfo->lambda);
      tinfo->jbar = ainv * jac.transpose() * tinfo->lambda;
      Matrix jtjbt(jac.transpose() * tinfo->jbar.transpose());
      tinfo->tau_full
	= jac.transpose() * tinfo->lambda * tinfo->task->getCommand() + jtjbt * grav;
      if (0 == ii) {
	tinfo->nullspace
	  = Matrix::Identity(ndof, ndof); // for this task, waste of ram and cycles...
	tinfo->tau_projected = tinfo->tau_full;		 // waste of ram and cycles...
	gamma = tinfo->tau_full;
	rangespace = jtjbt;	// for next task
      }
      else {
	tinfo->nullspace = Matrix::Identity(ndof, ndof) - rangespace;
	tinfo->tau_projected = tinfo->nullspace * tinfo->tau_full;
	gamma += tinfo->tau_projected;
	if (ii != n_minus_1) {
	  rangespace *= jtjbt;	// for next task
	}
      }
    }
    
    Status st;
    return st;
  }

}
