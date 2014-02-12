#ifndef MANAGERREQ_HPP
#define MANAGERREQ_HPP

#include <string>
#include <vector>
#include "manager_msg.h"

namespace HuboRT {

typedef std::vector<std::string> NameArray;

class ManagerReq
{
public:
    
    NameArray list_processes(double timeout = 2);
    NameArray list_locked_processes(double timeout = 2);
    NameArray list_channels(double timeout = 2);
//    NameArray list_open_channels(double timeout = 2);
    
    manager_err_t run_process(const std::string& name, const std::string& args = "");
    manager_err_t run_all_processes();
    
    manager_err_t stop_process(const std::string& name);
    manager_err_t stop_all_processes();
    
    manager_err_t create_ach_channel(const std::string& name);
    manager_err_t create_all_ach_channels();
    
    manager_err_t close_ach_channel(const std::string& name);
    manager_err_t close_all_ach_channels();
    
    manager_err_t register_new_process(const std::string& list_name,
                                       const std::string& full_process_path,
                                       const std::string& default_args = "");
    manager_err_t unregister_old_process(const std::string& list_name);
//    manager_err_t reset_process_roster();
    
    manager_err_t register_new_channel(const std::string& list_name,
                                       const std::string& ach_channel_name,
                                       size_t message_count = 10,
                                       size_t nominal_size = 4096);
    manager_err_t unregister_old_channel(const std::string& list_name);
//    manager_err_t reset_channel_roster();
    
    manager_err_t reset_rosters();
    
    manager_err_t start_up();
    manager_err_t shut_down();
    
    NameArray list_configs();
    manager_err_t save_current_config(const std::string& config_name);
    manager_err_t load_config(const std::string& config_name);
    
protected:
    
};

} // namespace HuboRT

#endif // MANAGERREQ_HPP
