/*
 * Copyright (c) 2011 Stanford University. All rights reserved.
 *                    Author: Roland Philippsen
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


static char const * robot_filename(TUTROB_XML_PATH_STR);
static double gfx_rate_hz(20.0);
static double servo_rate_hz(400.0);
static double sim_rate_hz(1600.0);
static int win_width(300);
static int win_height(200);
static char const * win_title("utaustin-wbc tutorial");
static boost::shared_ptr<jspace::tao_tree_info_s> sim_tree;
static boost::shared_ptr<jspace::tao_tree_info_s> scratch_tree;
static int id1(-1);
static int id2(-1);
static int id3(-1);
static int id4(-1);
static jspace::State state;
static size_t ndof;
static size_t toggle_count(0);
static double servo_dt_ms;


static bool (*servo_cb)(size_t toggle_count,
			double wall_time_ms,
			double sim_time_ms,
			jspace::State & state,
			jspace::Vector & command);

static void (*draw_cb)(double x0, double y0, double scale) = 0;


namespace {
  
  
  class Simulator : public Fl_Widget {
  public:
    Simulator(int xx, int yy, int width, int height, const char * label = 0);
    virtual ~Simulator();
    
    virtual void draw();
    
    void tick();
    static void timer_cb(void * param);
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
  
  
  static void write_state_to_tree(jspace::Vector const * jpos,
				  jspace::Vector const * jvel,
				  jspace::tao_tree_info_s & tree)
  {
    for (size_t ii(0); ii < ndof; ++ii) {
      taoJoint * joint(tree.info[ii].joint);
      int const kk(tree.info[ii].id);
      if (jpos) {
	joint->setQ(&(jpos->coeff(kk)));
      }
      if (jvel) {
	joint->setDQ(&(jvel->coeff(kk)));
      }
      joint->zeroDDQ();
      joint->zeroTau();
    }
    taoDynamics::updateTransformation(tree.root);
  }
  
  
  static void write_state_to_tree(jspace::tao_tree_info_s & tree)
  {
    write_state_to_tree(&(state.position_), &(state.velocity_), tree);
  }
  
  
  static void read_state_from_tree(jspace::tao_tree_info_s const & tree)
  {
    for (size_t ii(0); ii < ndof; ++ii) {
      taoJoint const * joint(tree.info[ii].joint);
      int const kk(tree.info[ii].id);
      joint->getQ(&(state.position_.coeffRef(kk)));
      joint->getDQ(&(state.velocity_.coeffRef(kk)));
      joint->getTau(&(state.force_.coeffRef(kk)));
    }
  }
  
  
  Simulator::
  Simulator(int xx, int yy, int width, int height, const char * label)
    : Fl_Widget(xx, yy, width, height, label)
  {
    Fl::add_timeout(0.1, timer_cb, this);
  }
  
  
  Simulator::
  ~Simulator()
  {
    Fl::remove_timeout(timer_cb, this);
  }
  
  
  static taoDNode * find_node(jspace::tao_tree_info_s & tree, std::string const & name)
  {
    for (size_t ii(0); ii < ndof; ++ii) {
      if (name == tree.info[ii].link_name) {
	return  tree.info[ii].node;
      }
    }
    errx(EXIT_FAILURE, "node `%s' not found", name.c_str());
  }
  
  
  static void raw_draw_tree(jspace::tao_tree_info_s & tree, double x0, double y0, double scale)
  {
    if (0 > id1) {
      id1 = find_node(*sim_tree, "link1")->getID();
      id2 = find_node(*sim_tree, "link2")->getID();
      id3 = find_node(*sim_tree, "link3")->getID();
      id4 = find_node(*sim_tree, "link4")->getID();
    }
    
    fl_line(x0 + (tree.info[id1].node->frameGlobal()->translation()[1] * scale),
	    y0 - (tree.info[id1].node->frameGlobal()->translation()[2] * scale),
	    x0 + (tree.info[id2].node->frameGlobal()->translation()[1] * scale),
	    y0 - (tree.info[id2].node->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (tree.info[id2].node->frameGlobal()->translation()[1] * scale),
	    y0 - (tree.info[id2].node->frameGlobal()->translation()[2] * scale),
	    x0 + (tree.info[id3].node->frameGlobal()->translation()[1] * scale),
	    y0 - (tree.info[id3].node->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (tree.info[id3].node->frameGlobal()->translation()[1] * scale),
	    y0 - (tree.info[id3].node->frameGlobal()->translation()[2] * scale),
	    x0 + (tree.info[id4].node->frameGlobal()->translation()[1] * scale),
	    y0 - (tree.info[id4].node->frameGlobal()->translation()[2] * scale));
    deFrame locframe;
    locframe.translation()[2] = -1.0;
    deFrame globframe;
    globframe.multiply(*(tree.info[id4].node->frameGlobal()), locframe);
    fl_line(x0 + (tree.info[id4].node->frameGlobal()->translation()[1] * scale),
	    y0 - (tree.info[id4].node->frameGlobal()->translation()[2] * scale),
	    x0 + (globframe.translation()[1] * scale),
	    y0 - (globframe.translation()[2] * scale));
  }
  
  
  static void raw_draw_com(jspace::tao_tree_info_s & tree, double x0, double y0, double scale)
  {
    if ( ! id1) {
      id1 = find_node(*sim_tree, "link1")->getID();
      id2 = find_node(*sim_tree, "link2")->getID();
      id3 = find_node(*sim_tree, "link3")->getID();
      id4 = find_node(*sim_tree, "link4")->getID();
    }
    
    deVector3 globpoint;
    globpoint.multiply(*(tree.info[id1].node->frameGlobal()), *(tree.info[id1].node->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));
    globpoint.multiply(*(tree.info[id2].node->frameGlobal()), *(tree.info[id2].node->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));
    globpoint.multiply(*(tree.info[id3].node->frameGlobal()), *(tree.info[id3].node->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));
    globpoint.multiply(*(tree.info[id4].node->frameGlobal()), *(tree.info[id4].node->center()));
    fl_point(x0 + (globpoint[1] * scale), y0 - (globpoint[2] * scale));
  }
  
  
  void Simulator::
  draw()
  {
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
    raw_draw_tree(*sim_tree, x0, y0, scale);
    
    fl_color(FL_GREEN);
    raw_draw_com(*sim_tree, x0, y0, scale);
    
    if (draw_cb) {
      draw_cb(x0, y0, scale);
    }
    
    fl_line_style(0);		// back to default
  }
  
  
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
    read_state_from_tree(*sim_tree);
    if ( ! servo_cb(toggle_count, wall_time_ms, sim_time_ms, state, command)) {
      write_state_to_tree(*sim_tree);
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
	  sim_tree->info[kk].joint->setTau(&(command.coeffRef(sim_tree->info[kk].id)));
	}
	taoDynamics::fwdDynamics(sim_tree->root, &earth_gravity);
	for (size_t kk(0); kk < ndof; ++kk) {
	  sim_tree->info[kk].joint->getDDQ(&(ddq.coeffRef(sim_tree->info[kk].id)));
	}
	////	add_command_to_sim_tree(command);
	////	taoDynamics::integrate(sim_tree->root, sim_dt);
	////	taoDynamics::updateTransformation(sim_tree->root);
	state.velocity_ += sim_dt * ddq;
	state.position_ += sim_dt * state.velocity_;
	write_state_to_tree(*sim_tree);
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
    toggle->callback(cb_toggle);
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
    ++toggle_count;
  }
  
  
  void Window::
  cb_quit(Fl_Widget * widget, void * param)
  {
    reinterpret_cast<Window*>(param)->hide();
  }
  
}


void tutsim::
set_robot_filename(char const * _robot_filename)
{
  robot_filename = _robot_filename;
}


void tutsim::
set_rates(double _gfx_rate_hz,
	  double _servo_rate_hz,
	  double _sim_rate_hz)
{
  gfx_rate_hz = _gfx_rate_hz;
  servo_rate_hz = _servo_rate_hz;
  sim_rate_hz = _sim_rate_hz;
}


void tutsim::
set_window_params(int width, int height, char const * title)
{
  win_width = width;
  win_height = height;
  win_title = title;
}


void tutsim::
draw_robot(jspace::Vector const & jpos, int width,
	   unsigned char red, unsigned char green, unsigned char blue,
	   double x0, double y0, double scale)
{
  write_state_to_tree(&jpos, 0, *scratch_tree);
  fl_color(red, green, blue);
  fl_line_style(FL_SOLID, width, 0);
  raw_draw_tree(*scratch_tree, x0, y0, scale);
  fl_line_style(0);		// back to default
}


void tutsim::
set_draw_cb(void (*_draw_cb)(double x0, double y0, double scale))
{
  draw_cb = _draw_cb;
}


int tutsim::
run(bool (*_servo_cb)(size_t toggle_count,
		      double wall_time_ms,
		      double sim_time_ms,
		      jspace::State & state,
		      jspace::Vector & command))
{
  if (gfx_rate_hz <= 0.0) {
    errx(EXIT_FAILURE, "invalid gfx_rate_hz %g (must be > 0)", gfx_rate_hz);
  }
  if (servo_rate_hz <= 0.0) {
    errx(EXIT_FAILURE, "invalid servo_rate_hz %g (must be > 0)", servo_rate_hz);
  }
  
  try {
    jspace::test::BRParser brp;
    boost::shared_ptr<jspace::test::BranchingRepresentation> brep(brp.parse(robot_filename));
    sim_tree.reset(brep->createTreeInfo());
    brep.reset(brp.parse(robot_filename));
    scratch_tree.reset(brep->createTreeInfo());
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  
  servo_cb = _servo_cb;
  ndof = sim_tree->info.size();
  state.init(ndof, ndof, ndof);
  write_state_to_tree(*sim_tree);
  
  Window win(win_width, win_height, win_title);
  return Fl::run();
}