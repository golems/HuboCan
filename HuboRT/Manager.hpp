#ifndef MANAGER_HPP
#define MANAGER_HPP

#include <string>
#include <vector>
#include "Daemonizer.hpp"

extern "C" {
#include "HuboCan/AchIncludes.h"
}

#include "HuboRT/manager_msg.hpp"

#include "HuboCan/InfoTypes.hpp"

namespace HuboRT {

const std::string opt_directory = "/opt";
const std::string hubo_directory = "/opt/hubo";
const std::string manager_directory = "/opt/hubo/mgr";
const std::string proc_roster_directory = "/opt/hubo/mgr/proc";
const std::string chan_roster_directory = "/opt/hubo/mgr/chan";
const std::string config_directory = "/opt/hubo/mgr/configs";

class MgrDaemon : public Daemonizer
{
public:
    inline MgrDaemon() :
        Daemonizer()
    {
        _lock_directory = "/opt/hubo/mgr/lock";
    }
};

class Manager
{
public:
    
    Manager();
    
    void launch();
    void run();
    
    
    // Begin: Callback functions
    void list_processes();
    void list_locked_processes();
    void list_channels();
//    void list_open_channels();
    
    void run_process(const std::string& name);
    void run_all_processes();
    
    void stop_process(const std::string& name);
    void stop_all_processes();
    
    void kill_process(const std::string& name);
    void kill_all_processes();
    
    void create_ach_chan(const std::string& name);
    void create_all_ach_chans();
    
    void close_ach_chan(const std::string& name);
    void close_all_ach_chans();
    
    void register_new_proc(const std::string& name);
    void unregister_old_proc(const std::string& name);
//    void reset_proc_roster();
    
    void register_new_chan(const std::string& name);
    void unregister_old_chan(const std::string& name);
//    void reset_chan_roster();
    
    void reset_rosters();
    
    void start_up();
    void shut_down();
    
    void list_configs();
    void save_current_config(const std::string& name);
    void load_config(const std::string& name);
    // End: Callback functions
    
    
protected:
    
    // Begin: Utility functions
    virtual void _initialize();
    virtual void _create_channel(const std::string& channel_name,
                                 size_t message_count,
                                 size_t nominal_size);
//    size_t _split_components(const std::string& name, StringArray& array);
    StringArray _grab_files_in_dir(const std::string& directory);
    bool _fork_process(const std::string& proc_name, const std::string& args);
    void _fork_process_raw(const std::string& proc_name, std::string args);
    void _stop_process_raw(const std::string &name);
    void _kill_process_raw(const std::string &name);
    std::string _create_ach_channel_raw(const std::string& name);
    bool _close_ach_channel_raw(const std::string& name);
    bool _register(const std::string& directory, const std::string& description, size_t minimum_size);
    void _unregister(const std::string& directory, const std::string& name);
    std::string _stringify_contents(const std::string& directory, const std::string& name);
    bool _load_config_raw(const std::string& name);
    void _save_config_raw(const std::string& name);
    void _clear_current_config();
    // End: Utility functions
    
    // Begin: Report functions    
    void _report_ach_error(const std::string& error_description);
    void _report_malformed_error(const std::string& error_description);
    void _report_error(manager_err_t error, const std::string& description);
    void _report_no_existence(manager_cmd_t original_req);
    void _report_no_error(manager_cmd_t original_req);
    void _relay_string_array(manager_cmd_t original_req, const StringArray& array);
    // End: Report functions
    
    
    MgrDaemon _rt;
    std::string _rt_lock_dir;
    std::string _rt_log_dir;
    
    std::string _proc_roster;
    std::string _chan_roster;
    std::string _config_roster;
    
    ach_channel_t _msg_chan;
    ach_channel_t _reply_chan;
    
};

} // namespace HuboRT

#endif // MANAGER_HPP
