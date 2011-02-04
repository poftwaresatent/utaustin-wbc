/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2010 University of Texas at Austin. All rights reserved.
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

/**
   \file m3_servo.cpp
   \author Roland Philippsen and Aaron Edsinger
   
   Inspired by code from M3 Meka Robotics Real-Time Control System
   Copyright (c) 2010 Meka Robotics
   Author: edsinger@mekabot.com (Aaron Edsinger)
   
   M3 is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   M3 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.
   
   You should have received a copy of the GNU Lesser General Public License
   along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

// RTAI and other basic includes.
#include <rtai_sched.h>
#include <rtai_shm.h>
#include <rtai.h>
#include <rtai_sem.h>
#include <rtai_nam2num.h>
#include <rtai_registry.h>
#include <stdio.h>
#include <signal.h>
#include <err.h>

// The top-level CMakeLists.txt file tries to find the
// torque_shm_sds.h header file underneath ${M3_DIR}/src and
// ${M3_DIR}. So, for example if your M3 code lives in
// /home/meka/mekabot/m3, you should run cmake like this:
// 
//   cmake /path/to/utaustin-wbc -DM3_DIR=/home/meka/mekabot/m3
// 
#include "m3/shared_mem/torque_shm_sds.h"
#include "m3/robots/chain_name.h"
#include <m3rt/base/m3ec_def.h>
#include <m3rt/base/m3rt_def.h>

// Stanford-WBC and other "high-level" includes.
#include <jspace/Model.hpp>
#include <jspace/controller_library.hpp>
#include <jspace/test/sai_util.hpp>
#include <sstream>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

using namespace std;


#define RT_TASK_FREQUENCY_TORQUE_SHM 400

//Period of rt-timer 
#define RT_TIMER_TICKS_NS_TORQUE_SHM (1000000000 / RT_TASK_FREQUENCY_TORQUE_SHM)

#define TORQUE_SHM "TSHMM"
#define TORQUE_CMD_SEM "TSHMC"
#define TORQUE_STATUS_SEM "TSHMS"

//////////////////////////////////////////////////
// command line arguments

static std::string xml_filename("/home/meka/mekabot/wbc-utexas/robospecs/m3_with_hand.xml");
static std::string control_law("float");
static jspace::Vector goal;
static jspace::Vector gain_kp;
static jspace::Vector gain_kd;
typedef enum { NORMAL, TP, GRAVCOMP } experiment_t;
static experiment_t experiment;
static std::string g_bitmask_str;

////////////////////////////////////////////////////////////////////////////////////
// shared memory stuff

static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static M3TorqueShmSdsCommand cmd;
static M3TorqueShmSdsStatus status;
static int sds_status_size;
static int sds_cmd_size;
static void endme(int code) { end=code; }

////////////////////////////////////////////////////////////////////////////////////
////// TorqueShm API:

void SetTorque_mNm(enum M3Chain chain,unsigned int  idx, mReal tq_desired); //ToShm
void SetTimestamp(int64_t  timestamp); //ToShm
int64_t GetTimestamp(); //FromShm

void SetTorque_mNm(enum M3Chain chain,unsigned int  idx, mReal tq_desired)
{  
  if (idx > MAX_NDOF)
    return;
  
  switch(chain)
    {
    case RIGHT_ARM:
      cmd.right_arm.tq_desired[idx] = tq_desired;
      return;
    case LEFT_ARM:
      cmd.left_arm.tq_desired[idx] = tq_desired;
      return;
    case TORSO:	
      cmd.torso.tq_desired[idx] = tq_desired;
      return;
    case HEAD:	
      cmd.head.tq_desired[idx] = tq_desired;
      return;
    }
  return; 
}

void SetTimestamp(int64_t  timestamp)
{  
  cmd.timestamp = timestamp;
  return; 
}

int64_t GetTimestamp()
{  
  return status.timestamp; 
}

////////////////////////// MAIN COMPUTATION METHOD /////////////////////////////

//////////////////////////////////////////////////
// WBC stuff

typedef map<string, boost::shared_ptr<jspace::Controller> > controller_lib_t;
typedef map<string, string> controller_doc_t;

static controller_lib_t controller_lib;
static controller_doc_t controller_doc;
static jspace::Controller * controller;
static std::string controller_errstr;
static boost::shared_ptr<jspace::Model> model;

static jspace::Matrix dbg_invLambda_t, dbg_Lambda_t;
static jspace::Matrix dbg_invLambda_p, dbg_Lambda_p;

/** Called from within get_controller(), so probably you do not need
    to call it yourself. */
static void _load_controllers();

/** Look up a controller by name and return it, or NULL if no matching
    entry was found. */
static jspace::Controller * get_controller(string const & name);


void _load_controllers()
{
  if (experiment != NORMAL) {
    return;
  }
  
  if ( ! controller_lib.empty()) {
    return;
  }
  
  if (0 == gain_kp.size()) {
    gain_kp = 60 * jspace::Vector::Ones(7);
  }
  else if (1 == gain_kp.size()) {
    gain_kp = gain_kp[0] * jspace::Vector::Ones(7);
  }
  else if (7 != gain_kp.size()) {
    errx(EXIT_FAILURE, "invalid number of kp gains (expected 7, got %d)", gain_kp.size());
  }
  
  if (0 == gain_kd.size()) {
    gain_kd = jspace::Vector::Zero(7);
  }
  else if (1 == gain_kd.size()) {
    gain_kd = gain_kd[0] * jspace::Vector::Ones(7);
  }
  else if (7 != gain_kd.size()) {
    errx(EXIT_FAILURE, "invalid number of kd gains (expected 7, got %d)", gain_kd.size());
  }
  
  boost::shared_ptr<jspace::Controller> ctrl;
  
  ctrl.reset(new jspace::FloatController());
  jspace::Status st(ctrl->setGains(jspace::Vector::Zero(7), gain_kd));
  if ( ! st) {
    errx(EXIT_FAILURE, "jspace::FloatController::setGains() failed: %s", st.errstr.c_str());
  }
  controller_lib.insert(make_pair("float", ctrl));
  controller_doc.insert(make_pair("float", "gravity compensation"));
  
  ctrl.reset(new jspace::JointGoalController(jspace::COMP_GRAVITY, gain_kp, gain_kd));
  controller_lib.insert(make_pair("pos_g", ctrl));
  controller_doc.insert(make_pair("pos_g", "joint position control with gravity compensation"));
  
  ctrl.reset(new jspace::JointGoalController(jspace::COMP_GRAVITY |
					     jspace::COMP_MASS_INERTIA,
					     gain_kp, gain_kd));
  controller_lib.insert(make_pair("pos_Ag", ctrl));
  controller_doc.insert(make_pair("pos_Ag", "joint pos with gravity and mass-inertia compensation"));
}


jspace::Controller * get_controller(string const & name)
{
  if (experiment != NORMAL) {
    return 0;
  }
  
  if (controller_lib.empty()) {
    _load_controllers();
  }
  
  controller_lib_t::iterator ic;
  if ("default" == name) {
    ic = controller_lib.find("float");
  }
  else {
    ic = controller_lib.find(name);
  }
  if (controller_lib.end() == ic) {
    return 0;
  }
  
  return ic->second.get();
}


// used as fall-back controller
static void StepFloat(jspace::Model const & model, jspace::Vector & tau)
{
  // this simplistic float mode can only handle a signle scalar kd
  // value...
  if (gain_kd.size() != 1) {
    tau = jspace::Vector::Zero(7);
    endme(269);
    return;
  }
  
  jspace::Vector gravity_torque;
  model.getGravity(gravity_torque);
  jspace::Matrix massInertia;
  model.getMassInertia(massInertia);
  tau = gravity_torque - gain_kd[0] * massInertia * model.getState().velocity_;
}


static void StepTaskPosture(jspace::Model const & model, jspace::Vector & tau)
{
  static taoDNode * right_hand(0);
  if ( ! right_hand) {
    right_hand = model.getNodeByName("right-hand");
    if ( ! right_hand) {
      tau = jspace::Vector::Zero(7);
      endme(378);
      return;
    }
  }
  
  //////////////////////////////////////////////////
  // task
  
  jspace::Transform eepos;
  model.computeGlobalFrame(right_hand, 0.0, -0.15, 0.0, eepos);
  
  jspace::Matrix Jfull;
  model.computeJacobian(right_hand,
			eepos.translation()[0],
			eepos.translation()[1],
			eepos.translation()[2],
			Jfull);
  jspace::Matrix Jx(Jfull.block(0, 0, 3, 7));
  jspace::Matrix invA;
  model.getInverseMassInertia(invA);
  jspace::Matrix invLambda_t(Jx * invA * Jx.transpose());

  Eigen::SVD<jspace::Matrix> svdLambda_t(invLambda_t);
  svdLambda_t.sort();
  int const nrows_t(svdLambda_t.singularValues().rows());
  jspace::Matrix Sinv_t;
  Sinv_t = jspace::Matrix::Zero(nrows_t, nrows_t);
  for (int ii(0); ii < nrows_t; ++ii) {
    if (svdLambda_t.singularValues().coeff(ii) > 1e-3) {
      Sinv_t.coeffRef(ii, ii) = 1.0 / svdLambda_t.singularValues().coeff(ii);
    }
  }
  jspace::Matrix Lambda_t(svdLambda_t.matrixU() * Sinv_t * svdLambda_t.matrixU().transpose());
  
  dbg_invLambda_t = invLambda_t;
  dbg_Lambda_t = Lambda_t;
  
  static jspace::Vector eegoal0;
  if (0 == eegoal0.size()) {
    if (goal.rows() < 3) {
      tau = jspace::Vector::Zero(7);
      endme(256);
      return;
    }
    eegoal0 = goal.block(0, 0, 3, 1);
  }
  struct timeval now;
  gettimeofday(&now, 0);
  double const alpha((1.0 * now.tv_sec + 1.0e-6 * now.tv_usec) * M_PI / 2.0);
  jspace::Vector eegoal(eegoal0);
  eegoal.coeffRef(1) += 0.2 * cos(alpha);
  eegoal.coeffRef(2) += 0.2 * sin(alpha);
  
  jspace::Vector poserror(eepos.translation() - eegoal);
  jspace::Vector g;
  model.getGravity(g);
  
  static jspace::Vector eegain_kp, eegain_kd;
  if (0 == eegain_kp.size()) {
    if (3 <= gain_kp.size()) {
      eegain_kp = gain_kp.block(0, 0, 3, 1);
    }
    else {
      eegain_kp = gain_kp[0] * jspace::Vector::Ones(3);
    }
    if (3 <= gain_kd.size()) {
      eegain_kd = gain_kd.block(0, 0, 3, 1);
    }
    else {
      eegain_kd = gain_kd[0] * jspace::Vector::Ones(3);
    }
  }

  jspace::Vector
    tau_task(Jx.transpose() * (-Lambda_t)
	     * ( eegain_kp.cwise() * poserror
		 + eegain_kd.cwise() * Jx * model.getState().velocity_));
  
  //////////////////////////////////////////////////
  // posture
  
  jspace::Matrix Jbar(invA * Jx.transpose() * Lambda_t);
  jspace::Matrix nullspace(jspace::Matrix::Identity(7, 7) - Jbar * Jx);
  
  static jspace::Vector goalposture;
  if (7 != goalposture.size()) {
    if (goal.size() >= 10) {
      goalposture = goal.block(3, 0, 7, 1);
      goalposture *= M_PI / 180;
    }
    else {
      goalposture = jspace::Vector::Zero(7);
      goalposture[1] = 30 * M_PI / 180; // shoulder "sideways" 30 degrees
      goalposture[3] = 60 * M_PI / 180; // ellbow at 60 degrees
    }
  }
  jspace::Matrix invLambda_p(nullspace * invA);
  Eigen::SVD<jspace::Matrix> svdLambda_p(invLambda_p);

  svdLambda_p.sort();
  
  int const nrows_p(svdLambda_p.singularValues().rows());
  jspace::Matrix Sinv_p;
  Sinv_p = jspace::Matrix::Zero(nrows_p, nrows_p);
  for (int ii(0); ii < nrows_p; ++ii) {
    if (svdLambda_p.singularValues().coeff(ii) > 1e-3) {
      Sinv_p.coeffRef(ii, ii) = 1.0 / svdLambda_p.singularValues().coeff(ii);
    }
  }
  jspace::Matrix Lambda_p(svdLambda_p.matrixU() * Sinv_p * svdLambda_p.matrixU().transpose());
  
  dbg_invLambda_p = invLambda_p;
  dbg_Lambda_p = Lambda_p;
  
  static jspace::Vector posturegain_kp, posturegain_kd;
  if (0 == posturegain_kp.size()) {
    if (10 <= gain_kp.size()) {
      posturegain_kp = gain_kp.block(3, 0, 7, 1);
    }
    else {
      if (1 == gain_kp.size()) {
	posturegain_kp = gain_kp[0] * jspace::Vector::Ones(7);
      }
      else {
	posturegain_kp = gain_kp[1] * jspace::Vector::Ones(7);
      }
    }
    if (10 <= gain_kd.size()) {
      posturegain_kd = gain_kd.block(3, 0, 7, 1);
    }
    else {
      if (1 == gain_kd.size()) {
	posturegain_kd = gain_kd[0] * jspace::Vector::Ones(7);
      }
      else {
	posturegain_kd = gain_kd[1] * jspace::Vector::Ones(7);
      }
    }
  }

  jspace::Vector tau_posture(nullspace.transpose() * (-Lambda_p)
			     * (posturegain_kp.cwise() * (model.getState().position_ - goalposture)
				+ posturegain_kd.cwise() * model.getState().velocity_));
  
  //////////////////////////////////////////////////
  // sum it up...
  
  tau = tau_task + tau_posture + g;
}


//opspace-force// // // Compute Jacobian at hand - Using a point in the middle of the
//opspace-force// // // hand - Following RT8 frame convention
//opspace-force// // SAIVector localHandCoord(3); localHandCoord[0] = 0.0; localHandCoord[1] = 0.0; localHandCoord[2] = 0.1;

//opspace-force// // // Rotation from RT8 (tool) to RT7 (wrist yaw)
//opspace-force// // SAIMatrix R87(3,3);
//opspace-force// // SAIVector xrt8 = R87 * localHandCoord;

//opspace-force// // // Rotation from RT7 to T0 (global)
//opspace-force// // //SAIMatrix R70(3,3);
//opspace-force// // SAIVector xt0 = R70 * xrt8;

//opspace-force// // Jacobian
//opspace-force// SAIMatrix J = robmodel->kinematics()->JacobianAtPoint( HSCBranchingRepresentation::Right_Hand, 
//opspace-force// 							 SAIVector(0)).submatrix(0,0,3,7);
//opspace-force// SAIVector ans = J.transpose() * force + robmodel->dynamics()->gravityForce();

//opspace-force// for (unsigned int ii(0); ii < 7; ++ii) {
//opspace-force//   SetTorque_mNm(RIGHT_ARM, ii, 1e3 * ans[ii]);
//opspace-force// }



////////////////////////// RTAI PROCESS /////////////////////////////

static void* rt_system_thread(void * arg)
{	
  SEM * status_sem;
  SEM * command_sem;
  RT_TASK *task;
	
  M3Sds * sds = (M3Sds *)arg;
  printf("Starting real-time thread\n");
		
	
  sds_status_size = sizeof(M3TorqueShmSdsStatus);
  sds_cmd_size = sizeof(M3TorqueShmSdsCommand);
	
  task = rt_task_init_schmod(nam2num("TSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
  rt_allow_nonroot_hrt();
  if (task==NULL)
    {
      printf("Failed to create RT-TASK TSHMP\n");
      return 0;
    }
  status_sem=(SEM*)rt_get_adr(nam2num(TORQUE_STATUS_SEM));
  command_sem=(SEM*)rt_get_adr(nam2num(TORQUE_CMD_SEM));
  if (!status_sem)
    {
      printf("Unable to find the %s semaphore.\n",TORQUE_STATUS_SEM);
      rt_task_delete(task);
      return 0;
    }
  if (!command_sem)
    {
      printf("Unable to find the %s semaphore.\n",TORQUE_CMD_SEM);
      rt_task_delete(task);
      return 0;
    }
	
	
  RTIME tick_period = nano2count(RT_TIMER_TICKS_NS_TORQUE_SHM); 
  RTIME now = rt_get_time();
  rt_task_make_periodic(task, now + tick_period, tick_period); 
  mlockall(MCL_CURRENT | MCL_FUTURE);
  rt_make_hard_real_time();
  long long start_time, end_time, dt;
  long long step_cnt = 0;
  sys_thread_active=1;
  
  bool first_iteration(true);
  while(!sys_thread_end)
    {
      start_time = nano2count(rt_get_cpu_time_ns());
      rt_sem_wait(status_sem);
      memcpy(&status, sds->status, sds_status_size);		
      rt_sem_signal(status_sem);

      //////////////////////////////////////////////////
      // BEGIN control algo
      
      SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat
      
      jspace::State state(7, 7, 0);
      
      for (unsigned int ii(0); ii < 7; ++ii) {
	state.position_[ii] = M_PI * status.right_arm.theta[ii] / 180.0;
	state.velocity_[ii] = M_PI * status.right_arm.thetadot[ii] / 180.0;
      }
      
      model->update(state);
      
      if (first_iteration) {
	if (controller) {
	  bool ok(true);
	  jspace::Status status(controller->init(*model));
	  if ( ! status) {
	    warnx("ERROR: controller failed to initialize: %s", status.errstr.c_str());
	    ok = false;
	  }
	  if (ok && (0 != goal.size())) {
	    status = controller->setGoal(goal);
	    if ( ! status) {
	      warnx("ERROR: controller did not accept goal: %s", status.errstr.c_str());
	      ok = false;
	    }
	  }
	  
	  if ( ! ok) {
	    endme(42);
	    sys_thread_end=1;
	    break;
	  }
	}
	first_iteration = false;
      }
      
      jspace::Vector tau;
      tau.setZero(7);
      if (experiment == GRAVCOMP) {
	StepFloat(*model, tau);
      }
      else if (experiment == TP) {
	StepTaskPosture(*model, tau);
      }
      else {
	if ( ! controller) {
	  endme(87);
	  sys_thread_end=1;
	}
	else {
	  jspace::Status status(controller->computeCommand(*model, tau));
	  if ( ! status) {
	    // Go to float mode on error...
	    controller_errstr = status.errstr;
	    StepFloat(*model, tau);
	  }
	}
      }
      
      for (unsigned int ii(0); ii < 7; ++ii) {
	SetTorque_mNm(RIGHT_ARM, ii, 1e3 * tau[ii]);
      }
      
      // END control algo
      //////////////////////////////////////////////////
		
      rt_sem_wait(command_sem);
      memcpy(sds->cmd, &cmd, sds_cmd_size);		
      rt_sem_signal(command_sem);
				
      end_time = nano2count(rt_get_cpu_time_ns());
      dt=end_time-start_time;
      /*
	Check the time it takes to run components, and if it takes longer
	than our period, make us run slower. Otherwise this task locks
	up the CPU.*/
      if (dt > tick_period && step_cnt>10) 
	{
	  printf("Step %lld: Computation time of components is too long. Forcing all components to state SafeOp.\n",step_cnt);
	  printf("Previous period: %f. New period: %f\n", (double)count2nano(tick_period),(double)count2nano(dt));
	  tick_period=dt;
	  rt_task_make_periodic(task, end + tick_period,tick_period);			
	}
      step_cnt++;
      rt_task_wait_period();
    }	
  printf("Exiting RealTime Thread...\n");
  rt_make_soft_real_time();
  rt_task_delete(task);
  sys_thread_active=0;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////

static void prettyPrint(double value)
{
  if (isinf(value)) {
    printf(" inf    ");
  }
  else if (isnan(value)) {
    printf(" nan    ");
  }
  else if (fabs(fmod(value, 1)) < 1e-6) {
    printf("%- 7d  ", static_cast<int>(rint(value)));
  }
  else {
    printf("% 6.4f  ", value);
  }
}


static void usage(ostream & os)
{
  os << "options:\n"
     << "   -h             help (this message)\n"
     << "   -t             go to TaskPosture mode (testing)\n"
     << "   -G             go to GRAVCOMP mode (testing)\n"
     << "   -C spec        gravity compensation bitmask (as a string)\n"
     << "   -f filename    specify SAI XML file\n"
     << "   -l control_law specify control law by name\n"
     << "   -g goal        set goal (space-separated degrees)\n"
     << "   -p kp          set proportional control gain\n"
     << "   -d kd          set differential control gain\n"
     << "available control laws:\n";
  _load_controllers();
  for (controller_doc_t::const_iterator ii(controller_doc.begin());
       ii != controller_doc.end(); ++ii) {
    os << "  " << ii->first << "\t" << ii->second << "\n";
  }
}


void parse_options(int argc, char ** argv)
{
  experiment = NORMAL;
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      cerr << argv[0] << ": problem with option '" << argv[ii] << "'\n";
      usage(cerr);
      exit(EXIT_FAILURE);
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(cout);
	exit(EXIT_SUCCESS);
	
      case 't':
	experiment = TP;
	break;
	
      case 'G':
	experiment = GRAVCOMP;
	break;
	
      case 'C':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -C requires parameter\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	g_bitmask_str = argv[ii];
	break;
	
      case 'f':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -f requires parameter\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	xml_filename = argv[ii];
	warnx("xml_filename: %s", xml_filename.c_str());
 	break;
	
      case 'l':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -l requires parameter\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	control_law = argv[ii];
	warnx("control_law: %s", control_law.c_str());
 	break;
	
      case 'g':
	++ii;
	if (ii >= argc)
	  errx(EXIT_FAILURE, "-g requires parameters");
	{
	  std::istringstream is(argv[ii]);
	  std::vector<double> arg_goal;
	  fprintf(stderr, "goal: ");
	  while (is) {
	    double bar;
	    is >> bar;
	    if (is) {
	      arg_goal.push_back(bar);
	      fprintf(stderr, " %g ", bar);
	    }
	    fprintf(stderr, "\n");
	  }
	  if (arg_goal.empty()) {
	    cerr << argv[0] << ": error reading goal from `" << argv[ii] << "'\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	  jspace::convert(arg_goal, goal);
	}
	break;
	
      case 'p':
	++ii;
	if (ii >= argc)
	  errx(EXIT_FAILURE, "-p requires parameter");
	{
	  std::istringstream is(argv[ii]);
	  std::vector<double> arg_kp;
	  fprintf(stderr, "kp: ");
	  while (is) {
	    double bar;
	    is >> bar;
	    if (is) {
	      arg_kp.push_back(bar);
	      fprintf(stderr, " %g ", bar);
	    }
	    fprintf(stderr, "\n");
	  }
	  if (arg_kp.empty()) {
	    cerr << argv[0] << ": error reading kp from `" << argv[ii] << "'\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	  jspace::convert(arg_kp, gain_kp);
	}
	break;
	
      case 'd':
	++ii;
	if (ii >= argc)
	  errx(EXIT_FAILURE, "-d requires parameter");
	{
	  std::istringstream is(argv[ii]);
	  std::vector<double> arg_kd;
	  fprintf(stderr, "kd: ");
	  while (is) {
	    double bar;
	    is >> bar;
	    if (is) {
	      arg_kd.push_back(bar);
	      fprintf(stderr, " %g ", bar);
	    }
	    fprintf(stderr, "\n");
	  }
	  if (arg_kd.empty()) {
	    cerr << argv[0] << ": error reading kd from `" << argv[ii] << "'\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	  jspace::convert(arg_kd, gain_kd);
	}
	break;
	
      default:
	cerr << argv[0] << ": invalid option '" << argv[ii] << "'\n";
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
  
  if (experiment != NORMAL) {
    controller = 0;
    return;
  }
  controller = get_controller(control_law);
  if ( ! controller) {
    std::ostringstream msg;
    for (controller_lib_t::const_iterator ii(controller_lib.begin());
	 ii != controller_lib.end(); ++ii) {
      msg << "  " << ii->first << "\n";
    }
    errx(EXIT_FAILURE, "invalid control mode `%s', use one of these:\n%s",
	 control_law.c_str(),
	 msg.str().c_str());
  }
}


int main (int argc, char ** argv)
{	
  RT_TASK *task;
  M3Sds * sys;
  
  parse_options(argc, argv);
  signal(SIGINT, endme);
  
  //////////////////////////////////////////////////
  // set up WBC model and dynamics
  try {
    static bool const enable_coriolis_centrifugal(false);
    model.reset(jspace::test::parse_sai_xml_file(xml_filename, enable_coriolis_centrifugal));
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  if ( ! g_bitmask_str.empty()) {
    for (size_t ii(0); ii < g_bitmask_str.size(); ++ii) {
      if (ii >= 7) {		// XXXX hardcoded 7 DOF
	warnx("more than 7 entries in g_bitmask_str");
	break;
      }
      if ('0' == g_bitmask_str[ii]) {
	model->disableGravityCompensation(ii, true);
      }
      else if ('1' != g_bitmask_str[ii]) {
	warnx("invalid character `%c'in g_bitmask_str", g_bitmask_str[ii]);
      }
    }
  }
  
  //////////////////////////////////////////////////
  // set up RT thread etc
  
  if (sys = (M3Sds*)rt_shm_alloc(nam2num(TORQUE_SHM),sizeof(M3Sds),USE_VMALLOC))
    printf("Found shared memory starting torque_shm.");
  else
    {
      printf("Rtai_malloc failure for %s\n",TORQUE_SHM);
      return 0;
    }
  
  rt_allow_nonroot_hrt();
  if (!(task = rt_task_init_schmod(nam2num("TSHM"), RT_TASK_PRIORITY, 0, 0, SCHED_FIFO, 0xF)))
    {
      rt_shm_free(nam2num(TORQUE_SHM));
      printf("Cannot init the RTAI task %s\n","TSHM");
      return 0;
    }
  
  //////////////////////////////////////////////////
  // run
  
  hst=rt_thread_create((void*)rt_system_thread, sys, 10000);
  usleep(100000); //Let start up
  if (!sys_thread_active)
    {
      rt_task_delete(task);
      rt_shm_free(nam2num(TORQUE_SHM));
      printf("Startup of thread failed.\n");
      return 0;
    }
  while (0 == end)
    {		
      usleep(1000000);
      
      printf("\ngoal          ");
      for (int ii(0); ii < goal.size(); ++ii) {
	prettyPrint(goal[ii]);
      }
      printf("\n");
      
      printf("joint angles: ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(status.right_arm.theta[ii]);
      }
      printf("\n");
      
      jspace::Matrix massInertia;
      model->getMassInertia(massInertia);
      printf("A diagonal:   ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(massInertia.coeff(ii, ii));
      }
      printf("\n");

      jspace::Vector gravity;
      model->getGravity(gravity);
      printf("gravity:      ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(gravity[ii]);
      }
      printf("\n");
      
      printf("tau desired:  ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(1e-3 * cmd.right_arm.tq_desired[ii]);
      }
      printf("\n");
      
      printf("tau actual:   ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(1e-3 * status.right_arm.torque[ii]);
      }
      printf("\n");
      
      printf("g/tau actual: ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	if (fabs(status.right_arm.torque[ii]) > 1e-3) {
	  prettyPrint(100 * 1e3 * gravity[ii] / status.right_arm.torque[ii]);
	}
	else {
	  printf(" OFL    ");
	}
      }
      printf("\n");
      
      static taoDNode * right_hand(0);
      if ( ! right_hand) {
	right_hand = model->getNodeByName("right-hand");
      }
      if (right_hand) {
	jspace::Transform eepos;
	model->getGlobalFrame(right_hand, eepos);
	printf("right-hand position:   ");
	for (unsigned int ii(0); ii < 3; ++ii) {
	  prettyPrint(eepos.translation()[ii]);
	}
	printf("\n");
	
	jspace::pretty_print(dbg_invLambda_t, cout, "invLambda_t", "  ");
	jspace::pretty_print(dbg_Lambda_t, cout, "Lambda_t", "  ");
	jspace::Matrix check_t(dbg_Lambda_t * dbg_invLambda_t);
	jspace::pretty_print(check_t, cout, "Lambda_t * invLambda_t", "  ");
	
	jspace::pretty_print(dbg_invLambda_p, cout, "invLambda_p", "  ");
	jspace::pretty_print(dbg_Lambda_p, cout, "Lambda_p", "  ");
	jspace::Matrix check_p(dbg_Lambda_p * dbg_invLambda_p);
	jspace::pretty_print(check_p, cout, "Lambda_p * invLambda_p", "  ");
	
      }

      if (controller) {
	printf("errstr (may be old): %s\n", controller_errstr.c_str());
      }
      
    }
  
  //////////////////////////////////////////////////
  // shutdown 

  printf("Removing RT thread, end code = %d.\n", end);
  sys_thread_end=1;
  rt_thread_join(hst);
  if (sys_thread_active)printf("Real-time thread did not shutdown correctly\n");
  rt_task_delete(task);
  rt_shm_free(nam2num(TORQUE_SHM));
  
  return 0;
}
