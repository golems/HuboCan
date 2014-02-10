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
     * \brief Returns true iff this process has received SIGCHLD
     * \return
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

    std::string lock_directory;
    size_t stack_prefault_size;

protected:

    /*!
     * \fn close()
     * \brief Cleans up the daemonization by removing the lock file
     * \return
     *
     * This function is called in Daemonizer's destructor, so there is no need to call
     * this on your own.
     */
    bool close();

    std::string _daemon_name;


};

} // namespace HuboRT

#endif // HUBORT_HPP
