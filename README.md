Whole-Body Control for Human-Centered Robotics, UT Austin
=========================================================

**UNDER CONSTRUCTION**

This project builds on top of the joint-space dynamics library from
the Stanford Whole-Body Control Framework http://stanford-wbc.sf.net/
in order to provide operational-space control for robots interacting
physically with everyday environments. The main developers are Luis
Sentis and Roland Philippsen.

Getting Started
---------------

Grab the code like this (note: we ditched the `git submodule` setup in favor of `git-subtree`):

    git clone git://github.com/poftwaresatent/utaustin-wbc.git
    cd utaustin-wbc

And then build it like so (unless you have a Meka arm, see below):

    mkdir build
    cd build
    cmake ..
    make

You probably want to point CMake to your installation of [gtest][],
for instance if you have [ROS][] installed do something like this:

[gtest]: http://code.google.com/p/googletest/
[ROS]: http://ros.org/

    cmake .. -DGTEST_DIR=/opt/ros/cturtle/ros/3rdparty/gtest/gtest

Finally, see if you can run the test:

    ./opspace/testTask

Running the RTAI shmem controller on the Meka arm
-------------------------------------------------

Make sure you have correctly configured your M3_ROBOT environment
variable in all shells, e.g. by placing something like this into your
~/.bashrc:

    export M3_ROBOT=/home/meka/mekabot/m3uta

Build the entire utaustin-wbc in release mode, pointing it to the
Meka/RTAI sources:

    mkdir release
    cd release
    cmake .. -DCMAKE_BUILD_TYPE=Release -DM3_DIR=/home/meka/mekabot/m3
    make

Again, you might want to add
`-DGTEST_DIR=/opt/ros/cturtle/ros/3rdparty/gtest/gtest` or similar as
well.

Start the Meka/RTAI controller (you may first need to `su -l` to the
account under which M3_ROBOT resides due to log file ownership
issues):

    m3rt_server_run

Start the script which switches the controller to shared memory mode:

    /path/to/opspace/apps/m3_torque_shm.py
   
This should create some messages from the Meka/RTAI controller as
well. Hit ENTER to continue in the `m3_torque_shm.py` terminal.

Start the `m3_servo` program, for instance in gravity compensation
mode (which is the default) with a bit of damping. Use the `-h` option
for more details:

    /path/to/opspace/release/apps/m3_servo -l float -d '2 2 2 2  1 1 1'

If this complains about a missing SAI XML file, add `-f
/path/to/opspace/robospecs/m3_with_hand.xml` to the command line.

Make sure the robot's safety button is OFF. Back in the
`m3_torque_shm.py` terminal, hit the `a` key. Hold on to the robot and
switch the safety button on.

The following **shutdown sequence** needs to be strictly followed in
order to avoid creating zombie RTAI processes:

 1. Hit `q` in the `m3_torque_shm.py` terminal.
 2. Hit Ctrl-C in the `m3_servo` terminal.
 3. Hit Ctrl-C in the `m3rt_server_run` terminal.
 4. Run `m3rt_server_kill` if that fails (or run it all the time
    anyway just for good measure).

Run the following to get the "vertical wall sweeping example" (see
http://www.youtube.com/watch?v=bU7Ocphhifg):

    ./apps/m3_servo -t -g "0.4 -0.2 -0.2    0 45 0 60    0 0 0" -p "150 150 150     100 100 100 100    100 100 100 " -d "20 20 20   5 5 5 5   25 25 25"

License
-------

The `utaustin-wbc` project is released under a new-style BSD
license. Most of it is Copyright (C) University of Texas at Austin.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of
       contributors to this software may be used to endorse or promote
       products derived from this software without specific prior written
       permission.
    
    THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
