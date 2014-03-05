
#include "HuboRT/ManagerReq.hpp"
#include <sstream>
#include <iostream>

using namespace HuboRT;

ManagerReq::ManagerReq()
{
    _initialize();
}

void ManagerReq::_initialize()
{
    ach_status_t r = ach_open(&_req_chan, hubo_rt_mgr_req_chan, NULL);
    if( ACH_OK != r )
    {
        std::cerr << "Error trying to open Manager Request Channel:\n"
                     << " -- (" << (int)r << ") " << ach_result_to_string(r) << std::endl;
    }
    
    r = ach_open(&_reply_chan, hubo_rt_mgr_reply_chan, NULL);
    if( ACH_OK != r )
    {
        std::cerr << "Error trying to open Manager Reply Channel:\n"
                     << " -- (" << (int)r << ") " << ach_result_to_string(r) << std::endl;
    }
    
    timeout = 2;
}

NameArray ManagerReq::list_registered_processes()
{
    NameArray reply;
    _send_request(LIST_PROCS, reply);
    return reply;
}

NameArray ManagerReq::list_locked_processes()
{
    NameArray reply;
    _send_request(LIST_LOCKED_PROCS, reply);
    return reply;
}

NameArray ManagerReq::list_channels()
{
    NameArray reply;
    _send_request(LIST_CHANS, reply);
    return reply;
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

manager_err_t ManagerReq::create_ach_channel(const std::string &name)
{
    return _send_request(CREATE_ACH_CHAN, name);
}

manager_err_t ManagerReq::create_all_ach_channels()
{
    return _send_request(CREATE_ALL_ACH_CHANS);
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
                                               size_t message_count, size_t nominal_size)
{
    std::stringstream stream;
    stream << list_name << ":" << ach_channel_name << ":" << message_count << ":" << nominal_size << ":";
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

manager_err_t ManagerReq::start_up()
{
    return _send_request(START_UP);
}

manager_err_t ManagerReq::shut_down()
{
    return _send_request(SHUT_DOWN);
}

NameArray ManagerReq::list_configs()
{
    NameArray reply;
    _send_request(LIST_CONFIGS, reply);
    return reply;
}

manager_err_t ManagerReq::save_current_config(const std::string &config_name)
{
    return _send_request(SAVE_CONFIG, config_name);
}

manager_err_t ManagerReq::load_config(const std::string &config_name)
{
    return _send_request(LOAD_CONFIG, config_name);
}

manager_err_t ManagerReq::_send_request(manager_cmd_t cmd, const std::string &desc)
{
    NameArray empty;
    return _send_request(cmd, empty, desc);
}

manager_err_t ManagerReq::_send_request(manager_cmd_t cmd, NameArray &reply, const std::string &desc)
{
    ach_flush(&_reply_chan);
    
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
        int nano_wait = wait_time.tv_nsec + (int)(timeout*1E9);
        wait_time.tv_sec += (int)(nano_wait/1E9);
        wait_time.tv_nsec = (int)(nano_wait%((int)1E9));
        r = ach_get(&_reply_chan, &raw_reply, sizeof(manager_reply_t),
                                 &fs, &wait_time, ACH_O_WAIT);
        
        if( ACH_TIMEOUT == r )
        {
            std::cerr << "The reply time from the Manager timed out!" << std::endl;
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
            std::cerr << "The reply from the Manager was not consistent with our request!" << std::endl;
            return MGR_RACE_CONDITION;
        }
        
        count++;
        expected = raw_reply.numReplies;
        reply.push_back(raw_reply.reply);
    }
    
    return raw_reply.err;
}
