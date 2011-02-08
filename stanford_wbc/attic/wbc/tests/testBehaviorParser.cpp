/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file testBehaviorParser.cpp
   \author Roland Philippsen
*/

#include <wbc/parse/BehaviorParser.hpp>
#include <wbcnet/log.hpp>
#include <err.h>


int main(int argc, char ** argv)
{
  if (2 > argc) {
    errx(EXIT_FAILURE, "XML file name expected");
  }
  
  wbcnet::configure_logging();
  wbcnet::manual_logging_verbosity(2);
  
  try {
    wbc::BehaviorParser bp;
    bp.Parse(argv[1], wbc::DebugBehaviorConstructionCallback());
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION %s", ee.what());
  }
}
