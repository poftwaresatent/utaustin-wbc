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

#include <jspace/test/sai_brep_parser.hpp>
#include <jspace/test/sai_brep.hpp>

#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoVar.h>
#include <tao/matrix/TaoDeMath.h>
#include <tao/matrix/TaoDeQuaternion.h>
#include <tao/matrix/TaoDeFrame.h>

#include <boost/shared_ptr.hpp>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/fl_draw.H>

#include <err.h>
#include <sys/time.h>


namespace tutsim {
  
  
  class Simulator : public Fl_Widget {
  public:
    Simulator(int xx, int yy, int width, int height, const char * label = 0);
    virtual ~Simulator();
    
    virtual void draw();
    
    void tick();
    static void timer_cb(void * param);
    
    enum {
      LINK1, LINK2, LINK3, LINK4, NDOF
    };
    
    taoDNode const * node_[NDOF];
    size_t toggle_count_;
  };
  
  
  class Window : public Fl_Window {
  public:
    Window(int width, int height, const char * title);
    
    virtual void resize(int x, int y, int w, int h);
    
    Simulator * simulator;
    Fl_Button * toggle;
    Fl_Button * quit;
    
    static void cb_toggle(Fl_Widget * widget, void * param);
    static void cb_quit(Fl_Widget * widget, void * param);
  };
  
  
  static boost::shared_ptr<jspace::tao_tree_info_s> tao_tree;
  static bool (*servo_cb)(size_t toggle_count,
			  double wall_time_ms,
			  double sim_time_ms,
			  jspace::State & state,
			  jspace::Vector & command);
  static jspace::State state;
  static size_t ndof;
  static double gfx_rate_hz;
  static double servo_rate_hz;
  static double sim_rate_hz;
  
  
  static void write_state_to_tao_tree()
  {
    for (size_t ii(0); ii < ndof; ++ii) {
      taoJoint * joint(tao_tree->info[ii].joint);
      int const kk(tao_tree->info[ii].id);
      joint->setQ(&(state.position_.coeffRef(kk)));
      joint->setDQ(&(state.velocity_.coeffRef(kk)));
      joint->zeroDDQ();
      joint->zeroTau();
    }
    taoDynamics::updateTransformation(tao_tree->root);
  }
  
  
  static void read_state_from_tao_tree()
  {
    for (size_t ii(0); ii < ndof; ++ii) {
      taoJoint * joint(tao_tree->info[ii].joint);
      int const kk(tao_tree->info[ii].id);
      joint->getQ(&(state.position_.coeffRef(kk)));
      joint->getDQ(&(state.velocity_.coeffRef(kk)));
      joint->getTau(&(state.force_.coeffRef(kk)));
    }
  }
  
  
  int run(double _gfx_rate_hz,
	  double _servo_rate_hz,
	  double _sim_rate_hz,
	  std::string const & robot_filename,
	  bool (*_servo_cb)(size_t toggle_count,
			    double wall_time_ms,
			    double sim_time_ms,
			    jspace::State & state,
			    jspace::Vector & command),
	  int width, int height, char const * title)
  {
    if (_gfx_rate_hz <= 0.0) {
      errx(EXIT_FAILURE, "invalid gfx_rate_hz %g (must be > 0)", _gfx_rate_hz);
    }
    if (_servo_rate_hz <= 0.0) {
      errx(EXIT_FAILURE, "invalid servo_rate_hz %g (must be > 0)", _servo_rate_hz);
    }
    
    try {
      jspace::test::BRParser brp;
      jspace::test::BranchingRepresentation * brep(brp.parse(robot_filename));
      tao_tree.reset(brep->createTreeInfo());
    }
    catch (std::runtime_error const & ee) {
      errx(EXIT_FAILURE, "%s", ee.what());
    }
    
    gfx_rate_hz = _gfx_rate_hz;
    servo_rate_hz = _servo_rate_hz;
    sim_rate_hz = _sim_rate_hz;
    servo_cb = _servo_cb;
    ndof = tao_tree->info.size();
    state.init(ndof, ndof, ndof);
    write_state_to_tao_tree();
    Window win(width, height, title);
    return Fl::run();
  }
  
  
  Simulator::
  Simulator(int xx, int yy, int width, int height, const char * label)
    : Fl_Widget(xx, yy, width, height, label),
      toggle_count_(0)
  {
    node_[0] = 0;
  }
  
  
  Simulator::
  ~Simulator()
  {
    if (node_[0]) {
      Fl::remove_timeout(timer_cb, this);
    }
  }
  
  
  static taoDNode * find_node(std::string const & name)
  {
    for (size_t ii(0); ii < ndof; ++ii) {
      if (name == tao_tree->info[ii].link_name) {
	return  tao_tree->info[ii].node;
      }
    }
    errx(EXIT_FAILURE, "node `%s' not found", name.c_str());
  }
  
  
  void Simulator::
  draw()
  {
    if ( ! node_[0]) {
      Fl::add_timeout(0.1, timer_cb, this);
      node_[LINK1] = find_node("link1");
      node_[LINK2] = find_node("link2");
      node_[LINK3] = find_node("link3");
      node_[LINK4] = find_node("link4");
    }
    
    double scale;
    if (w() > h()) {
      scale = h() / 9.0;
    }
    else {
      scale = w() / 9.0;
    }
    double const x0(w() / 2.0);
    double const y0(h() / 2.0);
    
    fl_color(FL_BLACK);
    fl_rectf(x(), y(), w(), h());
    
    fl_color(FL_WHITE);
    fl_line_style(FL_SOLID, 3, 0);
    
    fl_line(x0 + (node_[LINK1]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[LINK1]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[LINK2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[LINK2]->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (node_[LINK2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[LINK2]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[LINK3]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[LINK3]->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (node_[LINK3]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[LINK3]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[LINK4]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[LINK4]->frameGlobal()->translation()[2] * scale));
    deFrame locframe;
    locframe.translation()[2] = -1.0;
    deFrame globframe;
    globframe.multiply(*(node_[LINK4]->frameGlobal()), locframe);
    fl_line(x0 + (node_[LINK4]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[LINK4]->frameGlobal()->translation()[2] * scale),
	    x0 + (globframe.translation()[1] * scale),
	    y0 - (globframe.translation()[2] * scale));
    
    fl_color(FL_GREEN);
    deVector3 globpoint;
    globpoint.multiply(*(node_[LINK1]->frameGlobal()), *(node_[LINK1]->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));
    globpoint.multiply(*(node_[LINK2]->frameGlobal()), *(node_[LINK2]->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));
    globpoint.multiply(*(node_[LINK3]->frameGlobal()), *(node_[LINK3]->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));
    globpoint.multiply(*(node_[LINK4]->frameGlobal()), *(node_[LINK4]->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));

    fl_line_style(FL_SOLID, 1, 0);
  }
  
  
  static double servo_dt_ms;
  
  
  void Simulator::
  tick()
  {
    static struct timeval wall_time_start;
    static size_t sim_nsteps(0);
    static double wall_time_ms, sim_time_ms, sim_dt;
    static double gfx_dt_ms, gfx_next_ms;
    
    if (sim_nsteps == 0) {	// lazy init
      if (0 != gettimeofday(&wall_time_start, 0)) {
	errx(EXIT_FAILURE, "gettimofday failed");
      }
      wall_time_ms = 0;
      sim_time_ms = 0;
      servo_dt_ms = 1e3 / servo_rate_hz;
      if (sim_rate_hz <= servo_rate_hz) {
	sim_rate_hz = servo_rate_hz;
	sim_nsteps = 1;
      }
      else {
	sim_nsteps = llrint(ceil(sim_rate_hz / servo_rate_hz));
	sim_rate_hz = sim_nsteps * servo_rate_hz;
      }
      sim_dt = 1.0 / sim_rate_hz;
      gfx_dt_ms = 1e3 / gfx_rate_hz;
      gfx_next_ms = -1;
    }
    else {
      struct timeval now;
      if (0 != gettimeofday(&now, 0)) {
	errx(EXIT_FAILURE, "gettimofday failed");
      }
      wall_time_ms = 1e3 * (now.tv_sec - wall_time_start.tv_sec)
	+ 1e-3 * (now.tv_usec - wall_time_start.tv_usec);
      sim_time_ms += servo_dt_ms;
    }
    
    jspace::Vector command;
    read_state_from_tao_tree();
    if ( ! servo_cb(toggle_count_, wall_time_ms, sim_time_ms, state, command)) {
      write_state_to_tao_tree();
    }
    else {
      if (command.rows() != ndof) {
	errx(EXIT_FAILURE, "invalid command dimension %d (should be %zu)", command.rows(), ndof);
      }
      jspace::Vector ddq(ndof);
      for (size_t ii(0); ii < sim_nsteps; ++ii) {
	static deVector3 earth_gravity(0.0, 0.0, -9.81);
	for (size_t kk(0); kk < ndof; ++kk) {
	  // is this necessary each kk?
	  tao_tree->info[kk].joint->setTau(&(command.coeffRef(tao_tree->info[kk].id)));
	}
	taoDynamics::fwdDynamics(tao_tree->root, &earth_gravity);
	for (size_t kk(0); kk < ndof; ++kk) {
	  tao_tree->info[kk].joint->getDDQ(&(ddq.coeffRef(tao_tree->info[kk].id)));
	}
	////	add_command_to_tao_tree(command);
	////	taoDynamics::integrate(tao_tree->root, sim_dt);
	////	taoDynamics::updateTransformation(tao_tree->root);
	state.velocity_ += sim_dt * ddq;
	state.position_ += sim_dt * state.velocity_;
	write_state_to_tao_tree();
      }
    }
    
    if (wall_time_ms >= gfx_next_ms) {
      redraw();      
      gfx_next_ms += gfx_dt_ms;
    }
  }
  
  
  void Simulator::
  timer_cb(void * param)
  {
    reinterpret_cast<Simulator*>(param)->tick();
    Fl::repeat_timeout(1e-3 * servo_dt_ms, // gets initialized within tick()
		       timer_cb,
		       param);
  }
  
  
  Window::
  Window(int width, int height, const char * title)
    : Fl_Window(width, height, title)
  {
    begin();
    simulator = new Simulator(0, 0, width, height - 40);
    toggle = new Fl_Button(5, height - 35, 100, 30, "&Toggle");
    toggle->callback(cb_toggle, simulator);
    quit = new Fl_Button(width - 105, height - 35, 100, 30, "&Quit");
    quit->callback(cb_quit, this);
    end();
    resizable(this);
    show();
  }
  
  
  void Window::
  resize(int x, int y, int w, int h)
  {
    Fl_Window::resize(x, y, w, h);
    simulator->resize(0, 0, w, h - 40);
    toggle->resize(5, h-35, 100, 30);
    quit->resize(w-105, h-35, 100, 30);
  }
  
  
  void Window::
  cb_toggle(Fl_Widget * widget, void * param)
  {
    Simulator * dd(reinterpret_cast<Simulator*>(param));
    ++dd->toggle_count_;
  }
  
  
  void Window::
  cb_quit(Fl_Widget * widget, void * param)
  {
    reinterpret_cast<Window*>(param)->hide();
  }
  
}
