/* 
 * Copyright (C) 2008 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#ifndef WBCNET_NET_CONFIG_HPP
#define WBCNET_NET_CONFIG_HPP

#include <stdexcept>
#include <map>

namespace wbcnet {

  class Channel;
  
  
  /**
     Runtime-configurable creation of communication channels. You pass
     a string \c comspec to the static Create() method, out comes
     (some subclass of a) NetConfig, which you can then use to
     CreateChannel() either based on a pair of deprecated process_t
     tags, or a string-based \c connection_spec. Very useful for
     choosing the communication mode from the command line!
  */
  class NetConfig
  {
  public:
    typedef enum {
      SERVO,
      MODEL,
      USER,
      DISTANCE,
      MOTOR
    } process_t;
    
    virtual ~NetConfig() {}
    
    /**
       \note throws an exception if \c comspec is invalid. For a list of
       possible \c comspec values, call Help().
       
       \return A NetConfig subclass instance that you can use to
       create channels.
    */
    static NetConfig * Create(std::string const & comspec) throw(std::runtime_error);
  
    /**
       Print the list of known NetConfig subclasses and how to pass
       parameters to them.
    */
    static void Help(std::string const & prefix, std::ostream & os);
    
    
    /** Deprecated. Please use the string-based alternative. */
    virtual wbcnet::Channel * CreateChannel(process_t from_process,
					    process_t to_process) const
      throw(std::runtime_error) = 0;

    /** Replacement for the deprecated
	CreateChannel(process_t,process_t) method. Subclasses must
	implement it for providing a more generic creation patter. If
	such is not available, they should throw a runtime_error. */
    virtual wbcnet::Channel * CreateChannel(std::string const & connection_spec) const
          throw(std::runtime_error) = 0;
  };
  
}

#endif // WBCNET_NET_CONFIG_HPP
