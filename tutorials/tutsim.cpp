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

#include <jspace/Model.hpp>
#include <jspace/test/sai_util.hpp>
#include <tao/dynamics/taoDNode.h>

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
  
  
  class Drawing : public Fl_Widget {
  public:
    Drawing(int xx, int yy, int width, int height, const char * label = 0);
    virtual ~Drawing();
    
    virtual void draw();
    
    void tick();
    static void timer_cb(void * param);
    
    enum {
      A1, A2, A3, L1, L2, L3, R1, R2, R3, NDOF
    };
    
    taoDNode const * node_[NDOF];
    struct timeval tstart_;
  };
  
  
  class Window : public Fl_Window {
  public:
    Window(int width, int height, const char * title);
    
    virtual void resize(int x, int y, int w, int h);
    
    Drawing * drawing;
    Fl_Button * quit;
    
    static void cb_quit(Fl_Widget * widget, void * param);
  };
  
}


static std::string model_filename("/rolo/soft/utaustin-wbc/tutorials/tutrob.xml");
static boost::shared_ptr<jspace::Model> model;
static jspace::State state;
static size_t ndof;


int main(int argc, char ** argv)
{
  try {
    model.reset(jspace::test::parse_sai_xml_file(model_filename, false));
    ndof = model->getNDOF();
    state.init(ndof, ndof, ndof);
    tutsim::Window win(300, 200, "tutsim");
    return Fl::run();
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
}


namespace tutsim {
  
  Drawing::
  Drawing(int xx, int yy, int width, int height, const char * label)
    : Fl_Widget(xx, yy, width, height, label)
  {
    node_[0] = 0;
  }
  
  
  Drawing::
  ~Drawing()
  {
    if (node_[0]) {
      Fl::remove_timeout(timer_cb, this);
    }
  }
  
  
  void Drawing::
  draw()
  {
    if ( ! node_[0]) {
      if (0 != gettimeofday(&tstart_, 0)) {
	throw std::runtime_error("gettimofday failed");
      }
      Fl::add_timeout(0.5, timer_cb, this);
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
	  throw std::runtime_error("tutsim::Drawing::draw(): missing node");
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
  
  
  void Drawing::
  tick()
  {
    struct timeval now;
    if (0 != gettimeofday(&now, 0)) {
      throw std::runtime_error("gettimofday failed");
    }
    double const tt((tstart_.tv_sec - now.tv_sec) + 1e-6 * (tstart_.tv_usec - now.tv_usec));
    for (size_t ii(0); ii < ndof; ++ii) {
      state.position_[ii] = 0.1 * sin(ii + tt);
      state.velocity_[ii] = 0.05 * cos(ii + tt);
      state.force_[ii] = 0.0;
    }
    ////    jspace::pretty_print(state.position_, std::cerr, "jpos", "  ");
    model->update(state);
    ////    damage(FL_DAMAGE_USER1);
    redraw();
  }
  
  
  void Drawing::
  timer_cb(void * param)
  {
    reinterpret_cast<Drawing*>(param)->tick();
    Fl::repeat_timeout(0.05, timer_cb, param);
  }
  
  
  Window::
  Window(int width, int height, const char * title)
    : Fl_Window(width, height, title)
  {
    begin();
    drawing = new Drawing(0, 0, width, height - 40);
    quit = new Fl_Button(width / 2 - 40, height - 35, 100, 30, "&Quit");
    quit->callback(cb_quit, this);
    end();
    resizable(this);
    show();
  }
  
  
  void Window::
  resize(int x, int y, int w, int h)
  {
    Fl_Window::resize(x, y, w, h);
    drawing->resize(0, 0, w, h - 40);
    ////    drawing->redraw();
    quit->resize(w/2 - 40, h-35, 100, 30);
    ////    quit->redraw();
  }
  
  
  void Window::
  cb_quit(Fl_Widget * widget, void * param)
  {
    reinterpret_cast<Window*>(param)->hide();
  }
  
}
