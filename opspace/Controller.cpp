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

using jspace::pretty_print;

namespace opspace {
  
  
  Controller::
  Controller(std::string const & name, std::ostream * dbg)
    : name_(name),
      dbg_(dbg),
      initialized_(false)
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
    task_info_s * task_info(new task_info_s(task, controller_owned));
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
  
  
  SController::
  SController(std::string const & name, std::ostream * dbg)
    : Controller(name, dbg)
  {
  }
  
  
  Status SController::
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
    
    if (dbg_) {
      pretty_print(model.getState().position_, *dbg_, "DEBUG opspace::SController::computeCommand\n  jpos", "    ");
      pretty_print(model.getState().velocity_, *dbg_, "  jvel", "    ");
      pretty_print(grav, *dbg_, "  grav", "    ");
      pretty_print(ainv, *dbg_, "  ainv", "    ");
    }
    
    Matrix rangespace;
    size_t const ndof(model.getNDOF());
    size_t const n_minus_1(task_table_.size() - 1);
    
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      
      Task const * task(task_table_[ii]->task);
      Matrix const & jac(task->getJacobian());
      
      Matrix lambda, jbar;
      Vector tau;
      
      pseudoInverse(jac * ainv * jac.transpose(), 1e-3, lambda);
      jbar = ainv * jac.transpose() * lambda;
      Matrix const jtjbt(jac.transpose() * jbar.transpose());
      tau = jac.transpose() * lambda * task->getCommand() + jtjbt * grav;
      
      if (dbg_) {
	Vector const taunog(tau - jtjbt * grav);
	*dbg_ << "  task [" << ii << "] " << task->getName() << "\n";
	pretty_print(jac,    *dbg_, "    jac", "      ");
	pretty_print(lambda, *dbg_, "    lambda", "      ");
	pretty_print(jbar,   *dbg_, "    jbar", "      ");
	pretty_print(jtjbt,  *dbg_, "    jac.t * jbar.t", "      ");
	pretty_print(taunog, *dbg_, "    tau w/o gravity", "      ");
	pretty_print(tau,    *dbg_, "    tau", "      ");
      }
      
      if (0 == ii) {
	gamma = tau;
	rangespace = jtjbt;	// for next task
      }
      else {
	Matrix const nullspace(Matrix::Identity(ndof, ndof) - rangespace);
	gamma += nullspace * tau;
	if (ii != n_minus_1) {
	  rangespace *= jtjbt;	// for next task
	}
	if (dbg_) {
	  Vector const tauproj(nullspace * tau);
	  pretty_print(nullspace, *dbg_, "    nullspace", "      ");
	  pretty_print(tauproj,   *dbg_, "    tau projected", "      ");
	}
      }
    }
    
    if (dbg_) {
      pretty_print(gamma, *dbg_, "  gamma", "    ");
    }
    
    Status st;
    return st;
  }
  
  
  LController::
  LController(std::string const & name, std::ostream * dbg)
    : Controller(name, dbg)
  {
  }
  
  
  Status LController::
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
    
    if (dbg_) {
      pretty_print(model.getState().position_, *dbg_, "DEBUG opspace::LController::computeCommand\n  jpos", "    ");
      pretty_print(model.getState().velocity_, *dbg_, "  jvel", "    ");
      pretty_print(grav, *dbg_, "  grav", "    ");
      pretty_print(ainv, *dbg_, "  ainv", "    ");
    }
    
    size_t const ndof(model.getNDOF());
    size_t const n_minus_1(task_table_.size() - 1);
    Matrix nullspace(Matrix::Identity(ndof, ndof));
    
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      
      Task const * task(task_table_[ii]->task);
      Matrix const & jac(task->getJacobian());

      if (dbg_) {
	*dbg_ << "  task [" << ii << "] " << task->getName() << "\n";
	pretty_print(jac, *dbg_, "    jac", "      ");
      }
      
      Matrix jstar;
      if (ii == 0) {
	jstar = jac;
      }
      else {
	jstar = jac * nullspace;
	if (dbg_) {
	  pretty_print(nullspace, *dbg_, "    nullspace", "      ");
	  pretty_print(jstar,     *dbg_, "    jstar", "      ");
	}
      }
      
      Matrix lstar;
      pseudoInverse(jstar * ainv * jstar.transpose(), 1e-3, lstar);
      Vector pstar;
      pstar = lstar * jstar * ainv * grav; // same would go for coriolis-centrifugal...
      
      if (dbg_) {
	pretty_print(lstar, *dbg_, "    lstar", "      ");
	pretty_print(pstar, *dbg_, "    pstar", "      ");
      }
      
      // could add coriolis-centrifugal just like pstar...
      if (ii == 0) {
	gamma += jstar.transpose() * (lstar * task->getCommand() + pstar);
	if (dbg_) {
	  Vector const taunog(jstar.transpose() * (lstar * task->getCommand()));
	  Vector const tau(jstar.transpose() * (lstar * task->getCommand() + pstar));
	  pretty_print(taunog, *dbg_, "    tau w/o gravity", "      ");
	  pretty_print(tau,    *dbg_, "    tau", "      ");
	}
      }
      else {
	Vector fcomp;
	// here, gamma is still at the previous iteration's value
	fcomp = lstar * jstar * ainv * gamma;
	gamma += jstar.transpose() * (lstar * task->getCommand() + pstar - fcomp);
	if (dbg_) {
	  Vector const taunog(jstar.transpose() * (lstar * task->getCommand()));
	  Vector const tau(jstar.transpose() * (lstar * task->getCommand() + pstar - fcomp));
	  pretty_print(fcomp,  *dbg_, "    fcomp", "      ");
	  pretty_print(taunog, *dbg_, "    tau w/o (grav, comp)", "      ");
	  pretty_print(tau,    *dbg_, "    tau", "      ");
	}
	
      }
      
      if (ii != n_minus_1) {
	// not sure whether Eigen2 correctly handles the case where a
	// matrix gets updated by left-multiplication...
	Matrix const nnext((Matrix::Identity(ndof, ndof) - ainv * jstar.transpose() * lstar * jstar) * nullspace);
	nullspace = nnext;
      }
    }
    
    if (dbg_) {
      pretty_print(gamma, *dbg_, "  gamma", "    ");
    }
    
    Status st;
    return st;
  }
  
}
