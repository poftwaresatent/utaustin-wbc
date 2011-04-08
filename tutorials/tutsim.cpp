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

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/fl_draw.H>
//#include <limits>

#include <err.h>
// #include <stdlib.h>
// #include <string.h>
// #include <math.h>


namespace tutsim {
  
  class Drawing
    : public Fl_Widget
  {
  public:
    Drawing(int xx, int yy, int width, int height, const char * label = 0)
      : Fl_Widget(xx, yy, width, height, label)
    {
    }
    
  protected:
    virtual void draw()
    {
      fl_color(FL_BLACK);
      fl_rectf(x(), y(), w() - 1, h() - 1);
      fl_color(FL_WHITE);
      fl_line(x() + 1, y() + 1, x() + w() - 2, y() + h() - 2);
      fl_color(FL_YELLOW);
      fl_line(x() + 1, y() + h() - 2, x() + w() - 2, y() + 1);
      fl_color(FL_RED);
      fl_point(x() + w() / 2 - 5, y() + h() / 2 - 5);
      fl_color(FL_GREEN);
      fl_point(x() + w() / 2 - 5, y() + h() / 2 + 5);
      fl_color(FL_BLUE);
      fl_point(x() + w() / 2 + 5, y() + h() / 2 + 5);
      fl_color(FL_CYAN);
      fl_point(x() + w() / 2 + 5, y() + h() / 2 - 5);
    }
  };
  
  
  class Window
    : public Fl_Window
  {
  public:
    Window(int width, int height, const char * title)
      : Fl_Window(width, height, title)
    {
      begin();
      drawing = new Drawing(0, 0, width, height);
      quit = new Fl_Button(width / 2 - 50, width + 50, 100, 30, "&Quit");
      quit->callback(cb_quit, this);
      end();
      resizable(this);
      show();
    }
    
    Drawing * drawing;
    Fl_Button * quit;
    
  private:
    static void cb_quit(Fl_Widget * widget, void * param)
    {
      reinterpret_cast<Window*>(param)->hide();
    }
  };
  
}


int main(int argc, char ** argv)
{
  tutsim::Window win(300, 200, "tutsim");
  return Fl::run();
}
