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

#include "rt_util.hpp"
#include <err.h>
#include <signal.h>
#include <sys/time.h>

using namespace std;
using namespace jspace;
using namespace wbc_m3_ctrl;


static int joint_index;
static double torque;
static long long servo_rate;
static long long actual_servo_rate;
static bool keep_running;
static Vector position;
static Vector desired_torque;
static Vector actual_torque;


static void usage(int ecode, std::string msg)
{
  errx(ecode,
       "%s\n"
       "  options:\n"
       "  -h               help (this message)\n"
       "  -i  <index>      REQUIRED joint index (integer number 0..6)\n",
       "  -t  <torque>     applied torque (float number in Nm, default 0.1)\n",
       "  -f  <frequency>  servo rate (integer number in Hz, default 500Hz)\n",
       msg.c_str());
}


static void parse_options(int argc, char ** argv)
{
  joint_index = -1;
  torque = 0.1;
  servo_rate = 500;
  
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || ('-' != argv[ii][0])) {
      usage(EXIT_FAILURE, "problem with option `" + string(argv[ii]) + "'");
    }
    else
      switch (argv[ii][1]) {
	
      case 'h':
	usage(EXIT_SUCCESS, "servo [-h] [-v] [-s skillspec] -r robotspec");
	
      case 'f':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-f requires parameter");
 	}
	else {
	  istringstream is(argv[ii]);
	  is >> servo_rate;
	  if ( ! is) {
	    usage(EXIT_FAILURE, "failed to read servo rate from `" + string(argv[ii]) + "'");
	  }
	  if (0 >= servo_rate) {
	    usage(EXIT_FAILURE, "servo rate has to be positive");
	  }
	}
 	break;

      case 'i':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-i requires parameter");
 	}
	else {
	  istringstream is(argv[ii]);
	  is >> joint_index;
	  if ( ! is) {
	    usage(EXIT_FAILURE, "failed to read joint_index from `" + string(argv[ii]) + "'");
	  }
	  if ((0 > joint_index) || (7 <= joint_index)) {
	    usage(EXIT_FAILURE, "invalid joint_index");
	  }
	}
	break;

      case 't':
 	++ii;
 	if (ii >= argc) {
	  usage(EXIT_FAILURE, "-t requires parameter");
 	}
	else {
	  istringstream is(argv[ii]);
	  is >> torque;
	  if ( ! is) {
	    usage(EXIT_FAILURE, "failed to read torque from `" + string(argv[ii]) + "'");
	  }
	}
	break;
	
      default:
	usage(EXIT_FAILURE, "invalid option `" + string(argv[ii]) + "'");
      }
  }
}


static void handle(int signum)
{
  if (keep_running) {
    warnx("caught signal, requesting shutdown");
    keep_running = false;
  }
  else {
    errx(EXIT_SUCCESS, "caught signal (again?), attempting forced exit");
  }
}


namespace {
  
  
  class Servo
    : public RTUtil
  {
  public:
    virtual int init(State const & state) {
      return 0;
    }
    
    
    virtual int update(State const & state,
		       Vector & command)
    {
      position = state.position_;
      actual_torque = state.force_;
      if (0 == desired_torque.rows()) {
	desired_torque = Vector::Zero(7);
	desired_torque[joint_index] = torque;
      }
      command = desired_torque;
      return 0;
    }
    
    
    virtual int cleanup(void)
    {
      return 0;
    }
    
    
    virtual int slowdown(long long iteration,
			 long long desired_ns,
			 long long actual_ns)
    {
      actual_servo_rate = 1000000000 / actual_ns;
      return 0;
    }
  };
  
}


int main(int argc, char ** argv)
{
  struct sigaction sa;
  bzero(&sa, sizeof(sa));
  sa.sa_handler = handle;
  if (0 != sigaction(SIGINT, &sa, 0)) {
    err(EXIT_FAILURE, "sigaction");
  }
  
  keep_running = true;
  parse_options(argc, argv);
  Servo servo;
  try {
    actual_servo_rate = servo_rate;
    servo.start(servo_rate);
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "failed to start servo: %s", ee.what());
  }
  
  warnx("started servo RT thread");
  
  while (keep_running) {
    usleep(300000);
    cout << "**************************************************\n";
    pretty_print(position, cout, "position", "  ");
    pretty_print(desired_torque, cout, "desired_torque", "  ");
    pretty_print(actual_torque, cout, "actual_torque", "  ");
  }
  
  warnx("shutting down");
  servo.shutdown();
}
