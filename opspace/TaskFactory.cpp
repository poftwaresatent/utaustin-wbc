/*
 * Whole-Body Control for Human-Centered Robotics http://www.me.utexas.edu/~hcrl/
 *
 * Copyright (c) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen
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

#include "TaskFactory.hpp"
#include "Task.hpp"
#include <fstream>
#include <stdexcept>


namespace opspace {
  
  
  static void operator >> (YAML::Node const & node, Vector & vector)
  {
    vector.resize(node.size());
    for (size_t ii(0); ii < node.size(); ++ii) {
      node[ii] >> vector.coeffRef(ii);
    }
  }


  static Task * createTask(std::string const & type, std::string const & name)
  {
    return 0;
  }
  
  
  Status TaskFactory::
  parseString(std::string const & yaml_string)
  {
    std::istringstream is(yaml_string);
    return parseStream(is);
  }
  
  
  Status TaskFactory::
  parseFile(std::string const & yaml_filename)
  {
    std::ifstream is(yaml_filename.c_str());
    if ( ! is) {
      return Status(false, "could not open file `" + yaml_filename + "' for reading");
    }
    return parseStream(is);
  }
  

  Status TaskFactory::
  parseStream(std::istream & yaml_istream)
  {
    Status st;
    Task * task(0);
    
    try {
      YAML::Parser parser(yaml_istream);
      YAML::Node doc;
      while (parser.GetNextDocument(doc)) {
	for (size_t ii(0); ii < doc.size(); ++ii) {
	  YAML::Node const & node(doc[ii]);
	  std::string type, name;
	  node["type"] >> type;
	  node["name"] >> name;
	  task = createTask(type, name);
	  if ( ! task) {
	    throw std::runtime_error("opspace::TaskFactory::parseStream(): createTask(`"
				     + type + "', `" + name + "') failed");
	  }
	  if (dbg_) {
	    *dbg_ << "created task:  type = " << type << "  name = " << name << "\n"
		  << "  parsing parameters:\n";
	  }
	  for (YAML::Iterator it(node.begin()); it != node.end(); ++it) {
	    std::string key;
	    it.first() >> key;
	    Parameter * param(task->lookupParameter(key));
	    if (param) {
	      switch (param->type_) {
		
	      case TASK_PARAM_TYPE_INTEGER:
		if (YAML::CT_SCALAR != node.GetType()) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): parameter `" + key
					   + "' of task '" + name + "` should be scalar (integer)");
		}
		{
		  int value;
		  it.second() >> value;
		  st = param->set(value);
		  if ( ! st) {
		    throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					     + key + "' of task '" + name + "` failed: " + st.errstr);
		  }
		}
		break;
		
	      case TASK_PARAM_TYPE_REAL:
		if (YAML::CT_SCALAR != node.GetType()) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): parameter `" + key
					   + "' of task '" + name + "` should be scalar (real)");
		}
		{
		  double value;
		  it.second() >> value;
		  st = param->set(value);
		  if ( ! st) {
		    throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					     + key + "' of task '" + name + "` failed: " + st.errstr);
		  }
		}
		break;
		
	      case TASK_PARAM_TYPE_VECTOR:
		if (YAML::CT_SEQUENCE != node.GetType()) {
		  throw std::runtime_error("opspace::TaskFactory::parseStream(): parameter `" + key
					   + "' of task '" + name + "` should be sequence (vector)");
		}
		{
		  Vector value;
		  it.second() >> value;
		  st = param->set(value);
		  if ( ! st) {
		    throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
					     + key + "' of task '" + name + "` failed: " + st.errstr);
		  }
		}
		break;
		
	      case TASK_PARAM_TYPE_MATRIX:
		throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
                                           + key + "' of task '" + name
					 + "` requires MATRIX type which is not (yet) supported");
		break;
		
	      case TASK_PARAM_TYPE_VOID:
	      default:
		throw std::runtime_error("opspace::TaskFactory::parseStream(): setting parameter `"
                                           + key + "' of task '" + name
					 + "` invalid or VOID type cannot be set");
		
	      }
	      if (dbg_) {
		param->dump(*dbg_, "  ");
	      }
	    }
	    else if (dbg_) {
	      *dbg_ << "  " << key << " not found\n";
	    }
	    
	  }
	}

      }
    }
    catch (YAML::Exception const & ee) {
      delete task;
      st.ok = false;
      st.errstr = ee.what();
    }
    catch (std::runtime_error const & ee) {
      delete task;
      st.ok = false;
      st.errstr = ee.what();
    }
      
    return st;
  }

  
  TaskFactory::task_table_t const & TaskFactory::
  getTaskTable() const
  {
    return task_table_;
  }

}
