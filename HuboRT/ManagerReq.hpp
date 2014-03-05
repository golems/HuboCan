#ifndef MANAGERREQ_HPP
#define MANAGERREQ_HPP

#include <string>
#include <vector>

extern "C" {
#include "manager_msg.h"
#include "HuboCan/AchIncludes.h"
}

namespace HuboRT {

typedef std::vector<std::string> NameArray;

class ManagerReq
{
public:
    
    ManagerReq();
    
    double timeout;
    
    /*!
     * \fn list_registered_processes()
     * \brief Gives back a list of the registered processes
     * \param timeout
     * \return 
     * 
     * The return is a NameArray where each entry contains
     * the name of a process which is registered with the Manager
     * 
     * A timeout is given in case the manager is locked up (or not operating)
     * and cannot respond.
     */
    NameArray list_registered_processes();
    
    
    /*!
     * \fn list_locked_processes()
     * \brief Gives back a list of all the currently locked processes
     * \param timeout
     * \return 
     * 
     * The return is a NameArray where each entry contains the name of
     * a process which was daemonized using the Daemonizer class and
     * which currently has an active lock file. An active lock file can
     * mean one of two things:
     * 
     * \li The process is currently running
     * \li The process was somehow terminated forcibly
     * 
     * In either case, a new instance of that process will not be able 
     * to launch until the original lockfile is removed.
     * 
     * To instruct a currently running process to terminate gracefully,
     * use stop_process() or stop_all_processes().
     * 
     * To forcibly clobber a process which is stuck (or to remove a lockfile
     * of a process which did not quit gracefully) use either kill_process()
     * or kill_all_processes()
     */
    NameArray list_locked_processes();
    
    
    /*!
     * \fn list_channels(double timeout)
     * \brief Gives back a list of all registered channels
     * \param timeout
     * \return 
     * 
     * The return is a NameArray where each entry contains the name of
     * an ach channel which is registered with the Manager.
     */
    NameArray list_channels();
    
    // TODO:
//    NameArray list_open_channels();
    
    /*!
     * \fn run_process()
     * \brief Instructs the manager to launch a process with the given args
     * \param name
     * \param args
     * \return 
     * 
     * The name of the process must match one of the entries in the list given by
     * list_registered_processes(). If args is empty, the manager will use the default
     * arguments provided when the process was last registered with the Manager. If
     * you want the process to be launched with exactly no arguments, simply set args
     * to a single space " " instead of leaving it empty.
     * 
     * To add a process to the roster, use register_new_process()
     */
    manager_err_t run_process(const std::string& name, const std::string& args = "");
    
    /*!
     * \fn run_all_processes()
     * \brief Instructs the manager to launch all registered processes
     * \return 
     * 
     * Equivalent to running run_process() for each process in the Manager's roster.
     */
    manager_err_t run_all_processes();
    
    /*!
     * \fn stop_process()
     * \brief Gracefully stops the specified process
     * \param name
     * \return 
     * 
     * The name of the process must match one of the entries in the list given
     * by list_locked_processes(). NOTE: This is NOT necessarily the same as
     * list_registered_processes(), because a process might decide to lock itself
     * with a different name than it was registered with.
     * 
     * This is the function which should be used for gracefully terminating a 
     * process. If the process is stuck or non-responsive (or if the lockfile
     * continues to exist even though the process has quit), then you must use
     * kill_process() instead.
     */
    manager_err_t stop_process(const std::string& name);
    
    /*!
     * \fn stop_all_processes()
     * \brief Equivalent to running stop_process() for each entry of list_locked_processes()
     * \return 
     */
    manager_err_t stop_all_processes();
    
    /*!
     * \fn kill_process()
     * \brief Forcibly kills the specified process and removes its lock file
     * \param name
     * \return 
     * 
     * This function should be used only if stop_process() failed to stop the
     * specified process or failed to remove its lockfile.
     * 
     * In the event that a lockfile exists when it shouldn't, this function can be
     * used to remove it.
     */
    manager_err_t kill_process(const std::string& name);
    
    /*!
     * \fn kill_all_processes()
     * \brief Equivalent to running kill_process() for each entry of list_locked_processes()
     * \return 
     */
    manager_err_t kill_all_processes();
    
    /*!
     * \fn create_ach_channel()
     * \brief Instructs the manager to create the specified channel
     * \param name
     * \return 
     * 
     * The name of the channel must be one of the entries in list_channels().
     * To add a channel to the roster, use register_new_channel().
     */
    manager_err_t create_ach_channel(const std::string& name);
    
    /*!
     * \fn create_all_ach_channels()
     * \brief Equivalent to running create_ach_channel() for each entry of list_channels()
     * \return 
     */
    manager_err_t create_all_ach_channels();
    
    /*!
     * \fn close_ach_channel()
     * \brief Closes the specified ach channel
     * \param name
     * \return 
     * 
     * Ach channels should be closed and then recreated in order to clear out any
     * residual messages that might be sitting on them.
     * 
     * WARNING: Closing a channel will incapacitate any process which was using
     * that channel, even if the channel gets recreated afterwards. Only close a
     * channel if there are not supposed to be any processes using it.
     */
    manager_err_t close_ach_channel(const std::string& name);
    
    /*!
     * \fn close_all_ach_channels()
     * \brief Equivalent to calling close_ach_channel() for every registered channel
     * \return 
     */
    manager_err_t close_all_ach_channels();
    
    /*!
     * \fn register_new_process()
     * \brief Adds a new process to the roster
     * \param list_name indicates how you want the process to be named in the list
     * \param full_process_path represents what you would type into the terminal to launch the process
     * \param default_args represents what default arguments should be passed to the process
     * \return 
     * 
     * The roster represents the list of processes which you want the Manager to be aware of.
     * These processes will get spawned by the Manager whenever run_all_processes() or
     * start_up() is called.
     * 
     * If you use a list_name which is already taken, this will replace the old version with the
     * new specification.
     * 
     * To save a roster, use the function save_current_config()
     */
    manager_err_t register_new_process(const std::string& list_name,
                                       const std::string& full_process_path,
                                       const std::string& default_args = "");
    
    /*!
     * \fn unregister_old_process()
     * \brief Removes a process from the roster
     * \param list_name
     * \return 
     * 
     * This is a permanent removal, so you might want to consider using save_current_config()
     * before using this function.
     */
    manager_err_t unregister_old_process(const std::string& list_name);
    
    /*!
     * \fn register_new_channel()
     * \brief Adds a new Ach channel to the roster
     * \param list_name indicates how you want the channel to be named in the list
     * \param ach_channel_name represents the literal name of the ach channel which is to be managed
     * \param message_count represents how many messages you want the channel to be able to queue
     * \param nominal_size represents a rough estimate of how large each message is (in bytes)
     * \return 
     */
    manager_err_t register_new_channel(const std::string& list_name,
                                       const std::string& ach_channel_name,
                                       size_t message_count = 10,
                                       size_t nominal_size = 4096);
    
    /*!
     * \fn unregister_old_channel()
     * \brief Removes the specified Ach channel from the roster
     * \param list_name
     * \return 
     */
    manager_err_t unregister_old_channel(const std::string& list_name);
    
    /*!
     * \fn reset_rosters()
     * \brief Loads up the default roster configuration which is installed with the HuboCan package
     * \return 
     */
    manager_err_t reset_rosters();
    
    /*!
     * \fn start_up()
     * \brief Initializes the hardware interface and opens all channels and processes in the rosters
     * \return 
     */
    manager_err_t start_up();
    
    /*!
     * \fn shut_down()
     * \brief Gracefully stops all processes, closes all channels, and puts down the hardware interface
     * \return 
     */
    manager_err_t shut_down();
    
    /*!
     * \fn list_configs()
     * \brief Provides a list of the roster configurations which have been saved
     * \return 
     */
    NameArray list_configs();
    
    /*!
     * \fn save_current_config()
     * \brief Saves the current roster configuration
     * \param config_name
     * \return 
     */
    manager_err_t save_current_config(const std::string& config_name);
    
    /*!
     * \fn load_config()
     * \brief Loads the specified roster configuration
     * \param config_name
     * \return 
     */
    manager_err_t load_config(const std::string& config_name);
    
protected:
    
    void _initialize();
    
    manager_err_t _send_request(manager_cmd_t cmd, const std::string& desc="");
    manager_err_t _send_request(manager_cmd_t cmd, NameArray& reply, const std::string& desc = "");
    
    ach_channel_t _req_chan;
    ach_channel_t _reply_chan;
    
};

} // namespace HuboRT

#endif // MANAGERREQ_HPP
