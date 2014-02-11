/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HUBORT_HPP
#define HUBORT_HPP

#include <string>

namespace HuboRT {

class Daemonizer
{
public:

    Daemonizer(size_t safe_stack_size = 1024*1024);
    ~Daemonizer();
    
    /*!
     * \fn begin(std::string daemon_name, int priority = 40)
     * \brief Equivalent to daemonize(daemon_name) && prioritize(priority)
     */
    bool begin(std::string daemon_name, int priority = 40);

    /*!
     * \fn daemonize(std::string daemon_name)
     * \brief Creates a daemon with the given name
     * \param daemon_name
     * \return true iff the daemonization worked properly
     *
     * Good practice dictates that the name given to the daemon matches the name
     * of the process, or is some kind of unique identifier to make sure that no
     * duplicates are accidentally launched.
     *
     * argv[0] contains the name of the current process, so this could be passed in
     * if you would like to automate things.
     */
    bool daemonize(std::string daemon_name);

    /*!
     * \fn prioritize()
     * \brief Specify the real time priority of a process
     * \param priority
     * \return
     *
     * Critical kernel processes generally operate at a value of 50. HuboCan
     * processes which critically need to operate at real time priorities run
     * with a value of 49. Good practice would be to use 30 - 40 for things
     * which should try to be real time but aren't absolutely critically real time
     */
    bool prioritize(int priority = 40);

    /*!
     * \fn good()
     * \brief Check to make sure that the program has not been told to abort
     * \return
     */
    bool good() const;

    /*!
     * \fn usr1()
     * \brief Returns true iff this process has received SIGUSR1
     * \return
     *
     * Also resets the usr1 flag to false so that the next time SIGUSR1 is sent, it will
     * switch this back to true.
     */
    bool usr1();

    /*!
     * \fn usr2()
     * \brief Returns true iff this process has received SIGUSR2
     * \return
     *
     * Also resets the usr2 flag to false so that the next time SIGUSR2 is sent, it will
     * switch this back to true.
     */
    bool usr2();

    /*!
     * \fn alarm()
     * \brief Returns how many times this process has received SIGALRM
     * \return
     */
    size_t alarm() const;

    /*!
     * \fn child_process_exited()
     * \brief Returns the number of child processes which have quit
     * \return
     *
     * Using the GNU/Linux function fork() will spawn child processes.
     * Whenever a child process quits (with or without error) it will
     * send a signal SIGCHLD to the process it spawned from. This function
     * will return a count of how many times this process received a
     * SIGCHLD which should be indicative of how many times a child
     * process has quit.
     */
    size_t child_processes_exited() const;

    /*!
     * \fn check(bool condition)
     * \brief Graceful version of assert
     * \param condition
     * \return
     *
     * If the first argument (condition) is false, this will have one of two effects
     * depending on the second condition:
     *
     * \li quit_immediately = true : Immediately cease execution while cleaning up the
     * daemon's lock file
     *
     * \li quit_immediately = false : Behave as though an abort signal has been sent, so
     * the call to good() will return false from now on
     */
    bool check(bool condition, std::string message, bool quit_immediately=false);

    size_t stack_prefault_size;

protected:

    /*!
     * \fn close()
     * \brief Cleans up the daemonization by removing the lock file
     * \return
     *
     * This function is called in Daemonizer's destructor, so there is no need to call
     * this on your own.
     *
     * However, this does mean that you need to make sure your Daemonizer
     * instance is created in either global scope or the scope of your main()
     * function, or else it might be destructed prematurely.
     */
    void close();

    std::string _daemon_name;
    std::string _lock_directory;
    std::string _log_directory;
};

} // namespace HuboRT

#endif // HUBORT_HPP
