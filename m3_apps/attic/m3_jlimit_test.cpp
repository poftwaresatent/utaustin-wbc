/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Author: Roland Philippsen
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
#include <jspace/test/sai_util.hpp>
#include <opspace/TaskFactory.hpp>
#include <opspace/Controller.hpp>
#include <opspace/other_controllers.hpp>
#include <tao/dynamics/taoDNode.h>
#include <sstream>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

using jspace::State;
using jspace::convert;
using jspace::Transform;
using jspace::pretty_print;
using jspace::test::parse_sai_xml_file;
using namespace opspace;
using namespace std;

#define RT_TASK_FREQUENCY_TORQUE_SHM 400

//Period of rt-timer 
#define RT_TIMER_TICKS_NS_TORQUE_SHM (1000000000 / RT_TASK_FREQUENCY_TORQUE_SHM)

#define TORQUE_SHM "TSHMM"
#define TORQUE_CMD_SEM "TSHMC"
#define TORQUE_STATUS_SEM "TSHMS"

//////////////////////////////////////////////////
// command line arguments

static Vector jgoal;

////////////////////////////////////////////////////////////////////////////////////
// shared memory stuff

static int sys_thread_active = 0;
static int sys_thread_end=0;
static int end=0;
static int hst;
static M3TorqueShmSdsCommand shm_cmd;
static M3TorqueShmSdsStatus shm_status;
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
      shm_cmd.right_arm.tq_desired[idx] = tq_desired;
      return;
    case LEFT_ARM:
      shm_cmd.left_arm.tq_desired[idx] = tq_desired;
      return;
    case TORSO:	
      shm_cmd.torso.tq_desired[idx] = tq_desired;
      return;
    case HEAD:	
      shm_cmd.head.tq_desired[idx] = tq_desired;
      return;
    }
  return; 
}

void SetTimestamp(int64_t  timestamp)
{  
  shm_cmd.timestamp = timestamp;
  return; 
}

int64_t GetTimestamp()
{  
  return shm_status.timestamp; 
}

////////////////////////// MAIN COMPUTATION METHOD /////////////////////////////

//////////////////////////////////////////////////
// WBC stuff

static string controller_errstr;
static boost::shared_ptr<Model> model;
static boost::shared_ptr<Controller> controller;
static Parameter * jgoal_p(0);

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
  time_t trigger_goal_change;
  Vector goal[2];
  size_t next_goal;
  
  while(!sys_thread_end)
    {
      start_time = nano2count(rt_get_cpu_time_ns());
      rt_sem_wait(status_sem);
      memcpy(&shm_status, sds->status, sds_status_size);		
      rt_sem_signal(status_sem);

      //////////////////////////////////////////////////
      // BEGIN control algo
      
      SetTimestamp(GetTimestamp()); //Pass back timestamp as a heartbeat
      
      State state(7, 7, 0);
      
      for (unsigned int ii(0); ii < 7; ++ii) {
	state.position_[ii] = M_PI * shm_status.right_arm.theta[ii] / 180.0;
	state.velocity_[ii] = M_PI * shm_status.right_arm.thetadot[ii] / 180.0;
      }
      
      model->update(state);
      
      if (first_iteration) {
	bool ok(true);
	Status status(controller->init(*model));
	if ( ! status) {
	  warnx("ERROR: controller failed to initialize: %s",
		status.errstr.c_str());
	  ok = false;
	}
	if (ok) {
	  status = jgoal_p->set(jgoal);
	  if ( ! status) {
	    warnx("ERROR: jgoal setting failed (did you specify one?): %s",
		  status.errstr.c_str());
	    controller_errstr = "jgoal: " + status.errstr;
	    ok = false;
	  }
	}
	
	if ( ! ok) {
	  endme(42);
	  sys_thread_end=1;
	  break;
	}
	
	first_iteration = false;
      }
      
      Vector tau;
      tau.setZero(7);
      Status status(controller->computeCommand(*model, tau));
      if ( ! status) {
	controller_errstr = status.errstr;
	tau = - 10.0 * model->getState().velocity_;
      }
      
      for (unsigned int ii(0); ii < 7; ++ii) {
	SetTorque_mNm(RIGHT_ARM, ii, 1e3 * tau[ii]);
      }
      
      // END control algo
      //////////////////////////////////////////////////
      
      rt_sem_wait(command_sem);
      memcpy(sds->cmd, &shm_cmd, sds_cmd_size);		
      rt_sem_signal(command_sem);
      
      end_time = nano2count(rt_get_cpu_time_ns());
      dt=end_time-start_time;
      /*
	Check the time it takes to run components, and if it takes
	longer than our period, make us run slower. Otherwise this
	task locks up the CPU.*/
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
  os << "options: (-e and -j are required)\n"
     << "   -h             help (this message)\n"
     << "   -f filename    specify robot model (SAI XML file)\n"
     << "   -t filename    specify task definition (YAML file)\n"
     << "   -c name        controller type name (default LController)\n"
     << "   -j jgoal       posture goal [deg] (space-separated)\n";
}


void parse_options(int argc, char ** argv)
{
  string model_filename("/home/meka/mekabot/wbc-utexas/robospecs/m3_with_hand.xml");
  string tasks_filename("");
  string controller_typename("LController");
  static char const * tasks_default =
    "- type: opspace::PositionTask\n"
    "  name: eepos\n"
    "  control_point: [ 0.0, -0.15, 0.0 ]\n"
    "  dt_seconds: 0.002\n"
    "  kp: [ 150.0 ]\n"
    "  kd: [  20.0 ]\n"
    "  maxvel: [ 0.5 ]\n"
    "  maxacc: [ 1.5 ]\n"
    "- type: opspace::PostureTask\n"
    "  name: posture\n"
    "  dt_seconds: 0.002\n"
    "  kp: [ 100.0 ]\n"
    "  kd: [   5.0, 5.0, 5.0, 5.0, 25.0, 25.0, 25.0 ]\n"
    "  maxvel: [ 3.1416 ]\n"
    "  maxacc: [ 6.2832 ]\n";
  
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
	
      case 'f':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -f requires parameter\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	model_filename = argv[ii];
	warnx("model_filename: %s", model_filename.c_str());
 	break;
	
      case 't':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -t requires parameter\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	tasks_filename = argv[ii];
	warnx("tasks_filename: %s", tasks_filename.c_str());
 	break;

      case 'c':
 	++ii;
 	if (ii >= argc) {
 	  cerr << argv[0] << ": -c requires parameter\n";
 	  usage(cerr);
 	  exit(EXIT_FAILURE);
 	}
	controller_typename = argv[ii];
	warnx("controller_typename: %s", tasks_filename.c_str());
 	break;
	
      case 'j':
	++ii;
	if (ii >= argc)
	  errx(EXIT_FAILURE, "-j requires parameters");
	{
	  istringstream is(argv[ii]);
	  vector<double> arg_jgoal;
	  fprintf(stderr, "jgoal: ");
	  while (is) {
	    double bar;
	    is >> bar;
	    if (is) {
	      arg_jgoal.push_back(M_PI * bar / 180.0);
	      fprintf(stderr, "  %g deg = %g rad", bar, M_PI * bar / 180.0);
	    }
	    fprintf(stderr, "\n");
	  }
	  if (arg_jgoal.empty()) {
	    cerr << argv[0]
		 << ": error reading jgoal from `" << argv[ii] << "'\n";
	    usage(cerr);
	    exit(EXIT_FAILURE);
	  }
	  if (7 != arg_jgoal.size()) {
	    errx(EXIT_FAILURE, "jgoal must have seven dimensions");
	  }
	  convert(arg_jgoal, jgoal);
	}
	break;
	
      default:
	cerr << argv[0] << ": invalid option '" << argv[ii] << "'\n";
	usage(cerr);
	exit(EXIT_FAILURE);
      }
  }
  
  if ("LController" == controller_typename) {
    controller.reset(new LController(controller_typename));
  } 
  else if ("SController" == controller_typename) {
    controller.reset(new SController(controller_typename));
  }
  else {
    errx(EXIT_FAILURE,
	 "invalid controller type `%s', use one of these:\n"
	 "  LController\n"
	 "  SController\n",
	 controller_typename.c_str());
  }
  
  try {
    model.reset(parse_sai_xml_file(model_filename, false));
  }
  catch (runtime_error const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  TaskFactory tfac;
  Status st;  
  if (tasks_filename.empty()) {
    st = tfac.parseString(tasks_default);
  }
  else {
    st = tfac.parseFile(tasks_filename);
  }
  if ( ! st) {
    errx(EXIT_FAILURE, "failed to parse tasks file `%s': %s",
	 tasks_filename.c_str(), st.errstr.c_str());
  }
  tfac.dump(cout, "parsed tasks", "");
  TaskFactory::task_table_t const & ttab(tfac.getTaskTable());
  if (2 > ttab.size()) {
    errx(EXIT_FAILURE, "tasks file `%s' should define at least two tasks",
	 tasks_filename.c_str());
  }  
  for (size_t ii(0); ii < ttab.size(); ++ii) {
    if (1 == ii) {
      jgoal_p = ttab[ii]->lookupParameter("goal", PARAMETER_TYPE_VECTOR);
      if ( ! jgoal_p) {
	errx(EXIT_FAILURE,
	     "failed to retrieve `goal' parameter of second task (%s)",
	     ttab[ii]->getName().c_str());
      }
    }
    controller->appendTask(ttab[ii]);
  }
}


int main (int argc, char ** argv)
{	
  RT_TASK *task;
  M3Sds * sys;
  
  parse_options(argc, argv);
  signal(SIGINT, endme);
  
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
      usleep(200000);
      
      printf("\nposture goal: ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(jgoal_p->getVector()->coeff(ii) * 180.0 / M_PI);
      }
      printf("\njoint angles: ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(shm_status.right_arm.theta[ii]);
      }
      printf("\n");
      
      Matrix massInertia;
      model->getMassInertia(massInertia);
      printf("A diagonal:   ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(massInertia.coeff(ii, ii));
      }
      printf("\n");

      Vector gravity;
      model->getGravity(gravity);
      printf("gravity:      ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(gravity[ii]);
      }
      printf("\n");
      
      printf("tau desired:  ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(1e-3 * shm_cmd.right_arm.tq_desired[ii]);
      }
      printf("\n");
      
      printf("tau actual:   ");
      for (unsigned int ii(0); ii < 7; ++ii) {
	prettyPrint(1e-3 * shm_status.right_arm.torque[ii]);
      }
      printf("\n");
      
      controller->getTaskTable()[0]->dbg(cout, "", "");
      
      printf("errstr: %s\n", controller_errstr.c_str());
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
