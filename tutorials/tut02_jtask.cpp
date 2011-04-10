/*
 * Copyright (c) 2011 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include "tutsim.hpp"
#include <opspace/Task.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <err.h>


namespace tut02 {
  
  class JTask : public opspace::Task {
  public:
    JTask() : opspace::Task("tut02::JTask") {}
    
    virtual jspace::Status init(jspace::Model const & model)
    {
      //////////////////////////////////////////////////
      // The Jacobian is not really required for pure jspace control,
      // but will become very important for operational-space control.
      
      jacobian_ = jspace::Matrix::Identity(model.getNDOF(), model.getNDOF());
      
      //////////////////////////////////////////////////
      // Initialize our PD parameters.
      
      kp_ = 100.0;
      kd_ = 20.0;
      
      //////////////////////////////////////////////////
      // Initialize our goal to a configuration that is the current
      // one +/- 45 degrees on each joint.
      
      goal_ = model.getState().position_;
      for (int ii(0); ii < goal_.rows(); ++ii) {
	if (0 == (ii % 2)) {
	  goal_[ii] += M_PI / 4.0;
	}
	else {
	  goal_[ii] -= M_PI / 4.0;
	}
      }
      
      //////////////////////////////////////////////////
      // No initialization problems to report: the default constructor
      // of jspace::Status yields an instance that signifies success.
      
      jspace::Status ok;
      return ok;
    }
    
    
    virtual jspace::Status update(jspace::Model const & model)
    {
      //////////////////////////////////////////////////
      // Update the state of our task. Again, this is not critical
      // here, but important later when we want to integrate several
      // operational space tasks into a hierarchy.
      
      actual_ = model.getState().position_;
      
      //////////////////////////////////////////////////
      // Compute PD control torques and store them in command_ for
      // later retrieval.

      command_ = kp_ * (goal_ - actual_) - kd_ * model.getState().velocity_;
      
      jspace::Status ok;
      return ok;
    }
    
    double kp_, kd_;
    jspace::Vector goal_;
  };
  
}


static std::string model_filename("/rolo/soft/utaustin-wbc/tutorials/tutrob.xml");
static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<tut02::JTask> jtask;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  static size_t prev_toggle(1234);
  
  if (0 == (toggle_count % 2)) {
    
    //////////////////////////////////////////////////
    // Send torques that make the robot sway around.
    
    command = jspace::Vector::Zero(state.position_.rows());
    command[0] = 1e-3 * sin(1e-3 * sim_time_ms);
    command[3] = 0.5 * command[0];
    command[6] = command[3];
    
  }
  else {
    
    model->update(state);
    
    //////////////////////////////////////////////////
    // Use our JTask to compute the command, re-initializing it
    // whenever we switch here from the "swaying" mode, and setting
    // the goal to zero every other time.
    
    if (prev_toggle != toggle_count) {
      jtask->init(*model);
      if (3 == (toggle_count % 4)) {
	jtask->goal_ = jspace::Vector::Zero(state.position_.rows());
      }
    }
    jtask->update(*model);
    command = jtask->getCommand();
    
  }
  
  prev_toggle = toggle_count;
  
  //////////////////////////////////////////////////
  // Saturate command torques.
  
  static jspace::Vector cmax;
  if (0 == cmax.rows()) {
    cmax.resize(9);
    cmax << 100, 80, 80, 50, 30, 20, 50, 30, 20;
  }
  for (int idx(0); idx < command.rows(); ++idx) {
    if (command[idx] > cmax[idx]) {
      command[idx] = cmax[idx];
    }
    else if (command[idx] < -cmax[idx]) {
      command[idx] = -cmax[idx];
    }
  }
  
  //////////////////////////////////////////////////
  // Print debug info from time to time.
  
  static size_t iteration(0);
  if (0 == (iteration % 100)) {
    std::cerr << "toggle: " << toggle_count << "  sim_time_ms: " << sim_time_ms << "\n";
    if (0 != (toggle_count % 2)) {
      jspace::pretty_print(jtask->goal_, std::cerr, "goal", "  ");
    }
    jspace::pretty_print(state.position_, std::cerr, "jpos", "  ");
    jspace::pretty_print(state.velocity_, std::cerr, "jvel", "  ");
    jspace::pretty_print(state.force_, std::cerr, "jforce", "  ");
    jspace::pretty_print(command, std::cerr, "command", "  ");
  }
  ++iteration;
  
  return true;
}


int main(int argc, char ** argv)
{
  try {
    model.reset(jspace::test::parse_sai_xml_file(model_filename, true));
    jtask.reset(new tut02::JTask());
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  static double const gfx_rate_hz(20.0);
  static double const servo_rate_hz(400.0);
  static double const sim_rate_hz(1600.0);
  static int const win_width(300);
  static int const win_height(200);
  return tutsim::run(gfx_rate_hz, servo_rate_hz, sim_rate_hz,
		     model_filename, servo_cb, win_width, win_height,
		     "tut1_coupling");
}
