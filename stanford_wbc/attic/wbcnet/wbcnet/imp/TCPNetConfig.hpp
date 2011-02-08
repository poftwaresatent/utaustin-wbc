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

#ifndef WBCNET_TCP_NET_CONFIG_HPP
#define WBCNET_TCP_NET_CONFIG_HPP

#include <wbcnet/NetConfig.hpp>

namespace wbcnet {
  
  
  /**
     A NetConfig that creates an SoServer for you. This allows you to
     communicate via TCP/IP, if that is available on your platform.
  */
  class TCPServerNetConfig
    : public NetConfig
  {
  public:
    typedef NetConfig::process_t process_t;
  
    std::string const bind_ip;
  
    explicit TCPServerNetConfig(/** You probably want to say "*" here. */
				std::string const & bind_ip);
    
    virtual wbcnet::Channel * CreateChannel(process_t from_process,
					    process_t to_process) const
      throw(std::runtime_error);
    
    virtual wbcnet::Channel * CreateChannel(std::string const & connection_spec) const
      throw(std::runtime_error);
  };
  
  
  /**
     A NetConfig that creates an SoClient for you. This allows you to
     communicate via TCP/IP, if that is available on your platform.
  */
  class TCPClientNetConfig
    : public NetConfig
  {
  public:
    typedef NetConfig::process_t process_t;
  
    std::string const server_ip;
  
    explicit TCPClientNetConfig(/** For local communication, just use
				    "127.0.0.1". */
				std::string const & server_ip);
    
    virtual wbcnet::Channel * CreateChannel(process_t from_process,
					    process_t to_process) const
      throw(std::runtime_error);
    
    virtual wbcnet::Channel * CreateChannel(std::string const & connection_spec) const
      throw(std::runtime_error);
  };
  
}

#endif // WBCNET_TCP_NET_CONFIG_HPP
