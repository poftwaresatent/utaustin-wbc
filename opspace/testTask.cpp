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

#include <opspace/task_library.hpp>
//#include <opspace/opspace.hpp>
#include <opspace/Controller.hpp>
#include <jspace/test/model_library.hpp>
#include <err.h>

using jspace::Model;
using jspace::State;
using jspace::pretty_print;
using namespace opspace;
using namespace std;


int main(int argc, char ** argv)
{
  Model * puma(0);
  
  try {
    warnx("creating Puma model");
    puma = jspace::test::create_puma_model();
    size_t const ndof(puma->getNDOF());
    State state(ndof, ndof, 0);
    for (size_t ii(0); ii < ndof; ++ii) {
      state.position_[ii] = 0.1 * ii + 0.8;
      state.velocity_[ii] = 0.2 - 0.05 * ii;
    }
    puma->update(state);
    
    warnx("creating odd SelectedJointPostureTask");
    SelectedJointPostureTask odd("odd");
    Status st;
    odd.dump(cout, "freshly created odd task", "  ");
    
    warnx("testing update before init");
    st = odd.update(*puma);
    if (st) {
      throw runtime_error("odd task update() should have failed before init");
    }
    
    warnx("testing init before selection setting");
    st = odd.init(*puma);
    if (st) {
      throw runtime_error("odd task init() should have failed before setting selection");
    }
    
    warnx("retrieving selection parameter");
    Parameter * selection(odd.lookupParameter("selection", TASK_PARAM_TYPE_VECTOR));
    if ( ! selection) {
      throw runtime_error("failed to retrieve selection parameter");
    }
    
    warnx("trying to set selection");
    Vector sel(Vector::Zero(ndof));
    for (size_t ii(0); ii < ndof; ii += 2) {
      sel[ii] = 1.0;
    }
    st = selection->set(sel);
    if ( ! st) {
      throw runtime_error("failed to set selection: " + st.errstr);
    }
    odd.dump(cout, "odd task after setting selection", "  ");
    
    warnx("testing init");
    st = odd.init(*puma);
    if ( ! st) {
      throw runtime_error("odd task init() failed: " + st.errstr);
    }
    odd.dump(cout, "freshly initialized odd task", "  ");
    
    warnx("testing update");
    st = odd.update(*puma);
    if ( ! st) {
      throw runtime_error("odd task update() failed: " + st.errstr);
    }
    odd.dump(cout, "odd task after update", "  ");
    
    warnx("creating, initializing, and updating even task");
    SelectedJointPostureTask even("even");
    selection = even.lookupParameter("selection", TASK_PARAM_TYPE_VECTOR);
    sel = Vector::Zero(ndof);
    for (size_t ii(1); ii < ndof; ii += 2) {
      sel[ii] = 1.0;
    }
    selection->set(sel);
    even.init(*puma);
    even.update(*puma);
    even.dump(cout, "even task after update", "  ");
    
    warnx("creating and initializing ctrl");
    Controller ctrl;
    ctrl.appendTask(&odd, false);
    ctrl.appendTask(&even, false);
    st = ctrl.init(*puma);
    if ( ! st) {
      throw runtime_error("ctrl init() failed: " + st.errstr);
    }
    
    warnx("computing command");
    Vector gamma;
    st = ctrl.computeCommand(*puma, gamma);
    if ( ! st) {
      throw runtime_error("ctrl computeCommand() failed: " + st.errstr);
    }
    
    cout << "task matrices:\n";
    {
      Controller::task_table_t const & task_table(ctrl.getTaskTable());
      for (size_t ii(0); ii < task_table.size(); ++ii) {
	cout << "  level " << ii << ": task `" << task_table[ii]->task->getName() << "'\n";
	pretty_print(task_table[ii]->lambda, cout, "    lambda", "      ");
	pretty_print(task_table[ii]->jbar, cout, "    jbar", "      ");
	pretty_print(task_table[ii]->nullspace, cout, "    nullspace", "      ");
	pretty_print(task_table[ii]->tau_full, cout, "    tau_full", "      ");
	pretty_print(task_table[ii]->tau_projected, cout, "    tau_projected", "      ");
      }
    }
    pretty_print(gamma, cout, "final command", "  ");
    
    warnx("comparing against full joint posture task");
    SelectedJointPostureTask all("all");
    selection = all.lookupParameter("selection", TASK_PARAM_TYPE_VECTOR);
    sel = Vector::Ones(ndof);
    selection->set(sel);
    all.dump(cout, "all task before init", "  ");
    all.init(*puma);
    all.dump(cout, "all task after init", "  ");
    all.update(*puma);
    all.dump(cout, "all task after update", "  ");
    
    warnx("computing verification torque");
    Matrix aa;
    if ( ! puma->getMassInertia(aa)) {
      throw runtime_error("failed to get mass inertia");
    }
    Vector gg;
    if ( ! puma->getGravity(gg)) {
      throw runtime_error("failed to get gravity");
    }
    Vector tau_check_one;
    tau_check_one = aa * all.getCommand() + gg;
    pretty_print(tau_check_one, cout, "verification command one", "  ");

    warnx("creating and initializing verification controller");
    Controller ctrl_two;
    ctrl_two.appendTask(&all, false);
    st = ctrl_two.init(*puma);
    if ( ! st) {
      throw runtime_error("ctrl_two init() failed: " + st.errstr);
    }
    
    warnx("computing second verification torque");
    Vector gamma_two;
    st = ctrl_two.computeCommand(*puma, gamma_two);
    if ( ! st) {
      throw runtime_error("ctrl_two computeCommand() failed: " + st.errstr);
    }
    {
      Controller::task_table_t const & task_table(ctrl_two.getTaskTable());
      cout << "verification task matrices:\n";
      for (size_t ii(0); ii < task_table.size(); ++ii) {
	cout << "  level " << ii << ": task `" << task_table[ii]->task->getName() << "'\n";
	pretty_print(task_table[ii]->lambda, cout, "    lambda", "      ");
	pretty_print(task_table[ii]->jbar, cout, "    jbar", "      ");
	pretty_print(task_table[ii]->nullspace, cout, "    nullspace", "      ");
	pretty_print(task_table[ii]->tau_full, cout, "    tau_full", "      ");
	pretty_print(task_table[ii]->tau_projected, cout, "    tau_projected", "      ");
      }
      pretty_print(gamma_two, cout, "final command", "  ");
    }
    
    cout << "final comparisons:\n";
    pretty_print(gamma, cout, "  command of two disjoint tasks", "    ");
    pretty_print(gamma_two, cout, "  command of one full-rank task", "    ");
    pretty_print(tau_check_one, cout, "  command of \"manual\" dynamics compensation", "    ");
  }
  
  catch (runtime_error const & ee) {
    delete puma;
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  warnx("done with all tests");
  delete puma;
}
