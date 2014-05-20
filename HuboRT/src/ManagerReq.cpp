
#include "../ManagerReq.hpp"
#include <sstream>
#include <iostream>

using namespace HuboRT;

ManagerReq::ManagerReq()
{
    _initialized = false;
    timeout = 1;
    initialize();
}

bool ManagerReq::initialize()
{
    ach_status_t r = ach_open(&_req_chan, hubo_rt_mgr_req_chan, NULL);
    if( ACH_OK != r )
    {
        std::cerr << "Error trying to open Manager Request Channel:\n"
                     << " -- (" << (int)r << ") " << ach_result_to_string(r) << std::endl;
        _initialized = false;
        return false;
    }
    ach_flush(&_req_chan);
    
    r = ach_open(&_reply_chan, hubo_rt_mgr_reply_chan, NULL);
    if( ACH_OK != r )
    {
        std::cerr << "Error trying to open Manager Reply Channel:\n"
                     << " -- (" << (int)r << ") " << ach_result_to_string(r) << std::endl;
        _initialized = false;
        return false;
    }
    ach_flush(&_reply_chan);
    
    _initialized = true;
    return true;
}

bool ManagerReq::is_initialized()
{
    return _initialized;
}

manager_err_t ManagerReq::list_registered_processes(StringArray& reply)
{
    return _send_request(LIST_PROCS, reply);
}

manager_err_t ManagerReq::list_locked_processes(StringArray& reply)
{
    return _send_request(LIST_LOCKED_PROCS, reply);
}

manager_err_t ManagerReq::list_channels(StringArray& reply)
{
    return _send_request(LIST_CHANS, reply);
}

manager_err_t ManagerReq::run_process(const std::string &name, const std::string &args)
{
    std::stringstream stream;
    stream << name << ":" << args << ":";
    return _send_request(RUN_PROC, stream.str());
}

manager_err_t ManagerReq::run_all_processes()
{
    return _send_request(RUN_ALL_PROCS);
}

manager_err_t ManagerReq::stop_process(const std::string &name)
{
    return _send_request(STOP_PROC, name);
}

manager_err_t ManagerReq::stop_all_processes()
{
    return _send_request(STOP_ALL_PROCS);
}

manager_err_t ManagerReq::kill_process(const std::string &name)
{
    return _send_request(KILL_PROC, name);
}

manager_err_t ManagerReq::kill_all_processes()
{
    return _send_request(KILL_ALL_PROCS);
}

manager_err_t ManagerReq::create_ach_channel(const std::string &name, StringArray& achd_type)
{
    return _send_request(CREATE_ACH_CHAN, achd_type, name);
}

manager_err_t ManagerReq::create_all_ach_channels(StringArray& achd_types)
{
    return _send_request(CREATE_ALL_ACH_CHANS, achd_types);
}

manager_err_t ManagerReq::close_ach_channel(const std::string &name)
{
    return _send_request(CLOSE_ACH_CHAN, name);
}

manager_err_t ManagerReq::close_all_ach_channels()
{
    return _send_request(CLOSE_ALL_ACH_CHANS);
}

manager_err_t ManagerReq::register_new_process(const std::string &list_name,
                                               const std::string &full_process_path,
                                               const std::string &default_args)
{
    std::stringstream stream;
    stream << list_name << ":" << full_process_path << ":" << default_args << ":";
    return _send_request(REGISTER_NEW_PROC, stream.str());
}

manager_err_t ManagerReq::unregister_old_process(const std::string &list_name)
{
    return _send_request(UNREGISTER_OLD_PROC, list_name);
}

manager_err_t ManagerReq::register_new_channel(const std::string &list_name,
                                               const std::string &ach_channel_name,
                                               achd_network_t push_or_pull,
                                               size_t message_count, size_t nominal_size)
{
    std::stringstream stream;
    stream << list_name << ":" << ach_channel_name << ":" << message_count << ":" << nominal_size << ":";
    switch(push_or_pull)
    {
        case ACHD_PULL_FROM_ROBOT:
            stream << "PULL"; break;
        case ACHD_PUSH_TO_ROBOT:
            stream << "PUSH"; break;
        default:
            stream << "NOTHING"; break;
    }
    stream << ":";
    
    return _send_request(REGISTER_NEW_CHAN, stream.str());
}

manager_err_t ManagerReq::unregister_old_channel(const std::string &list_name)
{
    return _send_request(UNREGISTER_OLD_CHAN, list_name);
}

manager_err_t ManagerReq::reset_rosters()
{
    return _send_request(RESET_ROSTERS);
}

manager_err_t ManagerReq::start_up(StringArray& achd_types)
{
    return _send_request(START_UP, achd_types);
}

manager_err_t ManagerReq::shut_down()
{
    return _send_request(SHUT_DOWN);
}

manager_err_t ManagerReq::list_configs(StringArray& configs)
{
    return _send_request(LIST_CONFIGS, configs);
}

manager_err_t ManagerReq::save_current_config(const std::string &config_name)
{
    return _send_request(SAVE_CONFIG, config_name);
}

manager_err_t ManagerReq::load_config(const std::string &config_name)
{
    return _send_request(LOAD_CONFIG, config_name);
}

manager_err_t ManagerReq::delete_config(const std::string &config_name)
{
    return _send_request(DELETE_CONFIG, config_name);
}

manager_err_t ManagerReq::_send_request(manager_cmd_t cmd, const std::string &desc)
{
    StringArray empty;
    return _send_request(cmd, empty, desc);
}

manager_err_t ManagerReq::_send_request(manager_cmd_t cmd, StringArray &reply, const std::string &desc)
{
    ach_flush(&_reply_chan);
    reply.clear();
    
    manager_req_t request;
    request.request_type = cmd;
    strncpy(request.details, desc.c_str(), desc.size());
    request.details[desc.size()] = '\0';
    
    ach_status_t r = ach_put(&_req_chan, &request, sizeof(manager_req_t));
    if( ACH_OK != r )
    {
        std::cerr << "There was an Ach error while trying to send a request to the manager:"
                     << " -- (" << (int)r << ") " << ach_result_to_string(r) << std::endl;
        return ACH_ERROR;
    }
    
    manager_reply_t raw_reply;
    size_t count=0, expected=1;
    
    while(count < expected)
    {
        size_t fs;
        struct timespec wait_time;
        clock_gettime( ACH_DEFAULT_CLOCK, &wait_time);
        long nano_wait = wait_time.tv_nsec + (long)(timeout*1E9);
        wait_time.tv_sec += (long)(nano_wait/1E9);
        wait_time.tv_nsec = (long)(nano_wait%((long)1E9));
        r = ach_get(&_reply_chan, &raw_reply, sizeof(manager_reply_t),
                                 &fs, &wait_time, ACH_O_WAIT);
        
        if( ACH_TIMEOUT == r )
        {
            std::cerr << "The reply from the Manager timed out! ("
                      << timeout << " sec)" << std::endl;
            return MGR_TIMEOUT;
        }
        
        if( ACH_OK != r )
        {
            std::cerr << "There was an Ach error while trying to receive a reply from the manager:\n"
                         << " -- (" << (int)r << ") " << ach_result_to_string(r) << std::endl;
            return ACH_ERROR;
        }
        
        if( NO_ERROR != raw_reply.err )
        {
            return raw_reply.err;
        }
        
        if( raw_reply.original_req != cmd )
        {
            std::cerr << "The reply from the Manager was not consistent with our request!"
                      << std::endl;
            return MGR_RACE_CONDITION;
        }
        
        count++;
        expected = raw_reply.numReplies;
        reply.push_back(raw_reply.reply);
    }
    
    return raw_reply.err;
}

size_t ManagerReq::_split_components(const std::string &name, StringArray &array)
{
    array.resize(0);
    size_t pos = 0, last_pos=0, count=0;
    while(std::string::npos != (pos = name.find(":", last_pos)))
    {
        ++count;
        array.push_back(name.substr(last_pos, pos-last_pos));
        last_pos = pos+1;
    }
    
    return count;
}
