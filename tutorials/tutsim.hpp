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

#ifndef WBC_TUTORIAL_SIMULATOR_HPP
#define WBC_TUTORIAL_SIMULATOR_HPP

// The CMake setup "should" set the TUTROB_XML_PATH_STR
// correctly. Otherwise, fall back to a default that will produce
// sensible error messages in case we fail to find the file.

#ifndef TUTROB_XML_PATH_STR
#define TUTROB_XML_PATH_STR "tutrob.xml"
#endif // TUTROB_XML_PATH_STR

#include <jspace/Model.hpp>

/**
   Collection of static methods for running the planar tutorial
   simulator. We use a struct instead of a namespace in order to
   profit from compile-time method signature checks.
*/
struct tutsim {
  
  /**
     Override the default robot filename with something else. The
     default should be correctly set up by the CMake build system to
     point to the file tutrob.xml in the source directory of the
     tutorials. In case that goes wrong somehow, you can always
     override it here at runtime.
     
     \note The provided string is assumed to remain valid after you've
     passed its pointer here. E.g. it is okay to pass a pointer to one
     of the command line arguments, but don't pass some temporary
     std::string instance's c_str() return value.
  */
  static void set_robot_filename(char const * robot_filename);
  
  /**
     Override the default update rates.  At the time of writing, these
     were gfx_rate_hz=20, servo_rate_hz=400, and sim_rate_hz=1600.
  */
  static void set_rates(double gfx_rate_hz,
			double servo_rate_hz,
			double sim_rate_hz);
  
  /**
     Override the default parameters that get passed to the fltk
     window. At the time of writing, these were width=300, height=200,
     title="utaustin-wbc tutorial".
     
     \note The provided title must reside in static memory, in order
     to comply with requirements of FLTK.
  */
  static void set_window_params(int width, int height, char const * title);
  
  /**
     Draw the robot as if it were at the given joint
     configuration. Uses the specified line width and color. To be
     used from inside a function that you register with set_draw_cb(),
     which gets the correct x0, y0, and scale passed to it..
  */
  static void draw_robot(jspace::Vector const & jpos, int width,
			 unsigned char red, unsigned char green, unsigned char blue,
			 double x0, double y0, double scale);
  
  /**
     Draw the difference between the current simulated robot
     configuration and the given jpos.
  */
  static void draw_delta_jpos(jspace::Vector const & jpos, int width,
			      unsigned char red, unsigned char green, unsigned char blue,
			      double x0, double y0, double scale);
  
  /**
     Set a custom function for drawing additional information. Use
     e.g. draw_robot(), or you can emit raw fltk drawing commands. In
     the latter case, you need to take into account the x0, y0, and
     scale parameters, and keep in mind that fltk uses screen
     coordinates where the Y-axis is pointing down.  For instance, if
     you want to draw a line from (ax, ay) to (bx, by) you have to
     say:
     
     \code
     fl_line(x0 + ax * scale, y0 - ay * scale, x0 + bx * scale, y0 - by * scale);
     \endcode
  */
  static void set_draw_cb(void (*draw_cb)(double x0, double y0, double scale));
  
  /**
     Sets up the planar tutorial simulator and runs it, calling the
     supplied function whenever the servo needs to be updated. Inside
     your callback, you can either override the state and return
     false, in which case the simulator state gets re-initialized to
     whatever you specified; or you can set the command vector and
     return true, in which case the simulator will be used to update
     the robot state based on its rigid body dynamics and your
     supplied command torques. The toggle_count gets incremented each
     time the "Toggle" button gets pressed in the GUI, allowing very
     simple interaction with your callback.
  */
  static int run(bool (*servo_cb)(size_t toggle_count,
				  double wall_time_ms,
				  double sim_time_ms,
				  jspace::State & state,
				  jspace::Vector & command));
  
};

#endif // WBC_TUTORIAL_SIMULATOR_HPP
