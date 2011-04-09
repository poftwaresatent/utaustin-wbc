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

#include <jspace/Model.hpp>
#include <tao/dynamics/taoDNode.h>
#include <tao/matrix/TaoDeMath.h>
#include <tao/matrix/TaoDeQuaternion.h>
#include <tao/matrix/TaoDeFrame.h>

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
      SWAY, SERVO
    } state_;
    
    enum {
      A1, A2, A3, L1, L2, L3, R1, R2, R3, NDOF
    };
    
    taoDNode const * node_[NDOF];
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
  
  
  static jspace::Model * model;
  static void (*servo_cb)(double wall_time_ms,
			  double sim_time_ms,
			  jspace::Model const & model,
			  jspace::Vector & command);
  static jspace::State state;
  static size_t ndof;
  static double gfx_rate_hz;
  static double servo_rate_hz;
  static double sim_rate_hz;
  
  
  int run(double _gfx_rate_hz,
	  double _servo_rate_hz,
	  double _sim_rate_hz,
	  jspace::Model * _model,
	  void (*_servo_cb)(double wall_time_ms,
			    double sim_time_ms,
			    jspace::Model const & model,
			    jspace::Vector & command),
	  int width, int height, char const * title)
  {
    if (_gfx_rate_hz <= 0.0) {
      errx(EXIT_FAILURE, "invalid gfx_rate_hz %g (must be > 0)", _gfx_rate_hz);
    }
    if (_servo_rate_hz <= 0.0) {
      errx(EXIT_FAILURE, "invalid servo_rate_hz %g (must be > 0)", _servo_rate_hz);
    }
    gfx_rate_hz = _gfx_rate_hz;
    servo_rate_hz = _servo_rate_hz;
    sim_rate_hz = _sim_rate_hz;
    model = _model;
    servo_cb = _servo_cb;
    ndof = model->getNDOF();
    state.init(ndof, ndof, ndof);
    Window win(width, height, title);
    return Fl::run();
  }
  
  
  Simulator::
  Simulator(int xx, int yy, int width, int height, const char * label)
    : Fl_Widget(xx, yy, width, height, label),
      state_(SWAY)
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
  
  
  void Simulator::
  draw()
  {
    if ( ! node_[0]) {
      Fl::add_timeout(0.1, timer_cb, this);
      node_[A1] = model->getNodeByName("a1");
      node_[A2] = model->getNodeByName("a2");
      node_[A3] = model->getNodeByName("a3");
      node_[L1] = model->getNodeByName("l1");
      node_[L2] = model->getNodeByName("l2");
      node_[L3] = model->getNodeByName("l3");
      node_[R1] = model->getNodeByName("r1");
      node_[R2] = model->getNodeByName("r2");
      node_[R3] = model->getNodeByName("r3");
      for (size_t ii(0); ii < NDOF; ++ii) {
	if ( ! node_[ii]) {
	  errx(EXIT_FAILURE, "tutsim::Simulator::draw(): missing node");
	}
      }
    }
    
    double scale;
    if (w() > h()) {
      scale = h() / 7.06;
    }
    else {
      scale = w() / 7.06;
    }
    double const x0(w() / 2.0);
    double const y0(h() / 2.0);
    
    fl_color(FL_BLACK);
    fl_rectf(x(), y(), w(), h());
    
    fl_color(FL_WHITE);
    fl_line(x0 + (node_[A1]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[A1]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[A2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[A2]->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (node_[A2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[A2]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[A3]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[A3]->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (node_[L1]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[L1]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[R1]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[R1]->frameGlobal()->translation()[2] * scale));
    
    fl_color(FL_RED);
    fl_line(x0 + (node_[R1]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[R1]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[R2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[R2]->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (node_[R2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[R2]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[R3]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[R3]->frameGlobal()->translation()[2] * scale));
    deFrame loc;
    loc.translation()[1] = -0.15;
    deFrame aglob;
    aglob.multiply(*(node_[R3]->frameGlobal()), loc);
    loc.translation()[1] = 0.15;
    deFrame bglob;
    bglob.multiply(*(node_[R3]->frameGlobal()), loc);
    fl_line(x0 + (aglob.translation()[1] * scale),
	    y0 - (aglob.translation()[2] * scale),
	    x0 + (bglob.translation()[1] * scale),
	    y0 - (bglob.translation()[2] * scale));
    
    fl_color(FL_GREEN);
    fl_line(x0 + (node_[L1]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[L1]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[L2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[L2]->frameGlobal()->translation()[2] * scale));
    fl_line(x0 + (node_[L2]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[L2]->frameGlobal()->translation()[2] * scale),
	    x0 + (node_[L3]->frameGlobal()->translation()[1] * scale),
	    y0 - (node_[L3]->frameGlobal()->translation()[2] * scale));
    loc.translation()[1] = -0.15;
    aglob.multiply(*(node_[L3]->frameGlobal()), loc);
    loc.translation()[1] = 0.15;
    bglob.multiply(*(node_[L3]->frameGlobal()), loc);
    fl_line(x0 + (aglob.translation()[1] * scale),
	    y0 - (aglob.translation()[2] * scale),
	    x0 + (bglob.translation()[1] * scale),
	    y0 - (bglob.translation()[2] * scale));
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
    
    if (SWAY == state_) {
      for (size_t ii(0); ii < ndof; ++ii) {
	state.position_[ii] = 0.1 * sin(ii + 1e-3 * wall_time_ms);
	state.velocity_[ii] = 0.05 * cos(ii + 1e-3 * wall_time_ms);
	state.force_[ii] = 0.0;
      }
      model->update(state);
    }
    else if (SERVO == state_) {
      servo_cb(wall_time_ms, sim_time_ms, *model, state.force_);
      if (state.force_.rows() != ndof) {
	errx(EXIT_FAILURE, "invalid command dimension %d (should be %zu)", state.force_.rows(), ndof);
      }
      jspace::Matrix ainv;
      jspace::Vector gg, bb, ddq;
      for (size_t ii(0); ii < sim_nsteps; ++ii) {
	if ( ! model->getInverseMassInertia(ainv)) {
	  errx(EXIT_FAILURE, "model->getInverseMassInertia() failed");
	}
	if ( ! model->getGravity(gg)) {
	  errx(EXIT_FAILURE, "model->getGravity() failed");
	}
	if ( ! model->getCoriolisCentrifugal(bb)) {
	  errx(EXIT_FAILURE, "model->getCoriolisCentrifugal() failed");
	}
	ddq = ainv * (state.force_ - bb - gg);
	state.velocity_ += sim_dt * ddq;
	state.position_ += sim_dt * state.velocity_;
	model->update(state);
      }
    }
    else {
      errx(EXIT_FAILURE, "invalid state %d", state_);
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
    if (Simulator::SWAY == dd->state_) {
      dd->state_ = Simulator::SERVO;
    }
    else {
      dd->state_ = Simulator::SWAY;
    }
  }
  
  
  void Window::
  cb_quit(Fl_Widget * widget, void * param)
  {
    reinterpret_cast<Window*>(param)->hide();
  }
  
}
