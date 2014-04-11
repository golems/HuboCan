

extern "C" {
#include "HuboRT/manager_msg.h"
#include "HuboRT/Daemonizer_C.h"
#include "HuboCan/AchIncludes.h"
}

#include "HuboRT/Manager.hpp"
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <dirent.h>
#include <fstream>

using namespace HuboRT;

Manager::Manager()
{
    _initialize();
}

void Manager::_initialize()
{
    _rt_lock_dir = hubo_rt_default_lock_dir;
    _rt_log_dir = hubo_rt_default_log_dir;
    
    hubo_rt_safe_make_directory(opt_directory.c_str());
    hubo_rt_safe_make_directory(hubo_directory.c_str());
    hubo_rt_safe_make_directory(manager_directory.c_str());
    
    _proc_roster = proc_roster_directory;
    hubo_rt_safe_make_directory(_proc_roster.c_str());
    
    _chan_roster = chan_roster_directory;
    hubo_rt_safe_make_directory(_chan_roster.c_str());
    
    _config_roster = config_directory;
    hubo_rt_safe_make_directory(_config_roster.c_str());
    
    _create_channel(hubo_rt_mgr_req_chan, 20, 2048);
    _create_channel(hubo_rt_mgr_reply_chan, 20, 2048);
    
    ach_status_t r = ach_open(&_msg_chan, hubo_rt_mgr_req_chan, NULL);
    _rt.check(ACH_OK == r, "Could not open the Hubo Manager request channel (" + std::string(hubo_rt_mgr_req_chan) + ")", true);
    r = ach_open(&_reply_chan, hubo_rt_mgr_reply_chan, NULL);
    _rt.check(ACH_OK == r, "Could not open the Hubo Manager reply channel (" + std::string(hubo_rt_mgr_reply_chan) + ")", true);
}

void Manager::_create_channel(const std::string &channel_name,
                              size_t message_count,
                              size_t nominal_size)
{
    // TODO: Consider doing this with ach_create instead of a system call
    std::stringstream command_stream;
    command_stream << "ach mk " << channel_name << " -1"
                    << " -m " << message_count
                    << " -n " << nominal_size
                    << " -o 666";
    
    int exit_code = system(command_stream.str().c_str());
    if(exit_code != 0)
    {
        std::cout << "Unexpected exit code from 'ach mk': " << exit_code << std::endl;
    }
}

void Manager::launch()
{
    if(_rt.daemonize("hubo-manager"))
        run();
    else
        std::cerr << "Failed to daemonize! Check if a manager is already running!" << std::endl;
}

void Manager::run()
{
    manager_req_t incoming_msg;
    while(_rt.good())
    {
        size_t fs;
        double quit_check = 1;
        struct timespec wait_time;
        clock_gettime( ACH_DEFAULT_CLOCK, &wait_time );
        long nano_wait = wait_time.tv_nsec + (long)(quit_check*1E9);
        wait_time.tv_sec += (long)(nano_wait/1E9);
        wait_time.tv_nsec = (long)(nano_wait%((long)1E9));
        ach_status_t r = ach_get(&_msg_chan, &incoming_msg, sizeof(manager_req_t),
                                 &fs, &wait_time, ACH_O_WAIT);
        if( ACH_TIMEOUT == r )
            continue;
        
        if( ACH_OK != r )
        {
            std::cerr << "Ach error: (" << (int)r << ")" << ach_result_to_string(r) << std::endl;
            _report_ach_error(ach_result_to_string(r));
            continue;
        }
        
        if( sizeof(manager_req_t) != fs )
        {
            std::cerr << "Incoming message has a malformed size of " << fs
                         << "\n -- Should be size " << sizeof(manager_req_t) << std::endl;
            _report_malformed_error("");
            continue;
        }
        
        switch(incoming_msg.request_type)
        {
            case LIST_PROCS: list_processes();                                  break;
            case LIST_LOCKED_PROCS: list_locked_processes();                    break;
            case LIST_CHANS: list_channels();                                   break;
//            case LIST_OPEN_CHANS: list_open_channels();                         break;
                
            case RUN_PROC: run_process(incoming_msg.details);                      break;
            case RUN_ALL_PROCS: run_all_processes();                            break;
            
            case STOP_PROC: stop_process(incoming_msg.details);                    break;
            case STOP_ALL_PROCS: stop_all_processes();                          break;
                
            case KILL_PROC: kill_process(incoming_msg.details);                    break;
            case KILL_ALL_PROCS: kill_all_processes();                          break;
                
            case CREATE_ACH_CHAN: create_ach_chan(incoming_msg.details);           break;
            case CREATE_ALL_ACH_CHANS: create_all_ach_chans();                  break;
            
            case CLOSE_ACH_CHAN: close_ach_chan(incoming_msg.details);             break;
            case CLOSE_ALL_ACH_CHANS: close_all_ach_chans();                    break;
                
            case REGISTER_NEW_PROC: register_new_proc(incoming_msg.details);       break;
            case UNREGISTER_OLD_PROC: unregister_old_proc(incoming_msg.details);   break;
                
            case REGISTER_NEW_CHAN: register_new_chan(incoming_msg.details);       break;
            case UNREGISTER_OLD_CHAN: unregister_old_chan(incoming_msg.details);   break;
                
            case RESET_ROSTERS: reset_rosters();                                break;
                
            case START_UP: start_up();                                          break;
            case SHUT_DOWN: shut_down();                                        break;
                
            case LIST_CONFIGS: list_configs();                                  break;
            case SAVE_CONFIG: save_current_config(incoming_msg.details);           break;
            case LOAD_CONFIG: load_config(incoming_msg.details);                   break;
                
            default: _report_malformed_error("Unknown command type");           break;
        }
    }
}

StringArray Manager::_grab_files_in_dir(const std::string &directory)
{
    StringArray result;
    
    DIR* dptr = opendir(directory.c_str());
    if(dptr == NULL)
    {
        return result;
    }
    
    struct dirent* entry;
    while( NULL != (entry = readdir(dptr)) )
    {
        if(DT_REG == entry->d_type)
        {
            result.push_back(entry->d_name);
        }
    }
    
    return result;
}

void Manager::_relay_string_array(manager_cmd_t original_req, const StringArray &array)
{
    if(array.size() == 0)
    {
        manager_reply_t empty_reply;
        memset(&empty_reply, 0, sizeof(manager_reply_t));
        
        strcpy(empty_reply.reply, "");
        empty_reply.err = EMPTY_LIST;
        empty_reply.original_req = original_req;
        empty_reply.replyID = 0;
        empty_reply.numReplies = 1;
        
        ach_put(&_reply_chan, &empty_reply, sizeof(manager_reply_t));
    }
    else
    {
        manager_reply_t replies;
        memset(&replies, 0, sizeof(manager_reply_t));
        
        replies.err = NO_ERROR;
        replies.original_req = original_req;
        replies.numReplies = array.size();
        for(size_t i=0; i < array.size(); ++i)
        {
            replies.replyID = i;
            strcpy(replies.reply, array[i].c_str());
            
//            std::cout << array[i] << std::endl;
            
            ach_put(&_reply_chan, &replies, sizeof(manager_reply_t));
        }
    }
}

void Manager::list_processes()
{
    _relay_string_array(LIST_PROCS, _grab_files_in_dir(_proc_roster));
}

void Manager::list_locked_processes()
{
    _relay_string_array(LIST_LOCKED_PROCS, _grab_files_in_dir(_rt_lock_dir));
}

void Manager::list_channels()
{
    _relay_string_array(LIST_CHANS, _grab_files_in_dir(_chan_roster));
}

void Manager::run_process(const std::string &name)
{
    StringArray components;
    if(_split_components(name,components) == 2)
    {
        if(_fork_process(components[0], components[1]))
            _report_no_error(RUN_PROC);
        else
            _report_no_existence(RUN_PROC);
    }
    else
        _report_malformed_error("Run process should have exactly two components");
}

void Manager::run_all_processes()
{
    StringArray procs = _grab_files_in_dir(_proc_roster);
    for(size_t i=0; i < procs.size(); ++i)
    {
        _fork_process_raw(procs[i], "");
    }
    _report_no_error(RUN_ALL_PROCS);
}

void Manager::_stop_process_raw(const std::string &name)
{
    int id = 0;
    std::ifstream str;
    str.open( (_rt_lock_dir+"/"+name).c_str());
    
    str >> id;
    if( id > 0 )
    {
        std::cout << "Stopping process named '" << name << "' with id " << id << std::endl;
        pid_t processID = id;
        kill(processID, SIGINT);
    }

    // TODO: Check if process really stopped and report failure
    //          But how to do this in a decent way???
}

void Manager::_kill_process_raw(const std::string &name)
{
    int id = 0;
    std::ifstream str;
    str.open( (_rt_lock_dir+"/"+name).c_str());
    
    str >> id;
    if( id > 0 )
    {
        std::cout << "Killing process named '" << name << "' with id " << id << std::endl;
        pid_t processID = id;
        kill(processID, SIGKILL);
        
        remove( (_rt_lock_dir+"/"+name).c_str() );
    }
}

void Manager::stop_process(const std::string &name)
{
    StringArray procs = _grab_files_in_dir(_rt_lock_dir);
    bool exists = false;
    for(size_t i=0; i < procs.size(); ++i)
    {
        if( procs[i] == name )
        {
            exists = true;
            _stop_process_raw(name);
        }
    }
    
    if(exists)
    {
        _report_no_error(STOP_PROC);
    }
    else
    {
        _report_no_existence(STOP_PROC);
    }
}

void Manager::stop_all_processes()
{
    StringArray procs = _grab_files_in_dir(_rt_lock_dir);
    
    for(size_t i=0; i < procs.size(); ++i)
    {
        _stop_process_raw(procs[i]);
    }
    
    _report_no_error(STOP_ALL_PROCS);
}

void Manager::kill_process(const std::string &name)
{
    StringArray procs = _grab_files_in_dir(_rt_lock_dir);
    bool exists = false;
    for(size_t i=0; i < procs.size(); ++i)
    {
        if( procs[i] == name )
        {
            exists = true;
            _kill_process_raw(name);
        }
    }
    
    if(exists)
    {
        _report_no_error(KILL_PROC);
    }
    else
    {
        _report_no_existence(KILL_PROC);
    }
}

void Manager::kill_all_processes()
{
    StringArray procs = _grab_files_in_dir(_rt_lock_dir);
    
    for(size_t i=0; i < procs.size(); ++i)
    {
        _kill_process_raw(procs[i]);
    }
    
    _report_no_error(KILL_ALL_PROCS);
}

bool Manager::_create_ach_channel_raw(const std::string &name)
{
    std::ifstream chan_file;
    chan_file.open( (_chan_roster+"/"+name).c_str() );
    if(chan_file.good())
    {
        size_t count=0, fs=0;
        std::string chan_name;
        
        std::string chan_desc;
        std::getline(chan_file, chan_desc);
        StringArray components;
        if(_split_components(chan_desc, components) >= 3)
        {
            chan_name = components[0];
            count = atoi(components[1].c_str());
            fs = atoi(components[2].c_str());
            _create_channel(chan_name, count, fs);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool Manager::_close_ach_channel_raw(const std::string &name)
{
    std::ifstream chan_file;
    chan_file.open( (_chan_roster+"/"+name).c_str() );
    if(chan_file.good())
    {
        std::string chan_name;
        
        std::string chan_desc;
        std::getline(chan_file, chan_desc);
        StringArray components;
        if(_split_components(chan_desc, components) >= 3)
        {
            chan_name = components[0];
            
            int exit_code = system( ("ach rm "+chan_name).c_str());
            if(exit_code != 0)
            {
                std::cout << "Unexpected exit code from 'ach rm': " << exit_code << std::endl;
            }
            
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

void Manager::create_ach_chan(const std::string &name)
{
    if(_create_ach_channel_raw(name))
    {
        _report_no_error(CREATE_ACH_CHAN);
    }
    else
    {
        _report_no_existence(CREATE_ACH_CHAN);
    }
}

void Manager::create_all_ach_chans()
{
    StringArray chans = _grab_files_in_dir(_chan_roster);
    
    for( size_t i=0; i < chans.size(); ++i )
    {
        _create_ach_channel_raw(chans[i]);
    }
    
    _report_no_error(CREATE_ALL_ACH_CHANS);
}

void Manager::close_ach_chan(const std::string &name)
{
    if(_close_ach_channel_raw(name))
    {
        _report_no_error(CLOSE_ACH_CHAN);
    }
    else
    {
        _report_no_existence(CLOSE_ACH_CHAN);
    }
}

void Manager::close_all_ach_chans()
{
    StringArray chans = _grab_files_in_dir(_chan_roster);
    
    for(size_t i=0; i < chans.size(); ++i)
    {
        _close_ach_channel_raw(chans[i]);
    }
    
    _report_no_error(CLOSE_ALL_ACH_CHANS);
}

void Manager::register_new_proc(const std::string &name)
{   
    if(_register(_proc_roster, name, 3))
    {
        _report_no_error(REGISTER_NEW_PROC);
    }
    else
    {
        _report_malformed_error("REGISTER_PROC did not have the correct number of arguments in the description");
    }
}

void Manager::unregister_old_proc(const std::string &name)
{
    _unregister(_proc_roster, name);
    _report_no_error(UNREGISTER_OLD_PROC);
}

void Manager::register_new_chan(const std::string &name)
{
    if(_register(_chan_roster, name, 4))
    {
        _report_no_error(REGISTER_NEW_CHAN);
    }
    else
    {
        _report_malformed_error("REGISTER_CHAN did not have the correct number of arguments in the description");
    }
}

bool Manager::_register(const std::string &directory,
                        const std::string &description,
                        size_t minimum_size)
{
    StringArray components;
    if(_split_components(description, components) < minimum_size)
        return false;
    
    std::ofstream output;
    output.open( (directory+"/"+components[0]).c_str() );
    
    std::cout << "Registering " << components[0] << ": ";
    for(size_t i=1; i < components.size(); ++i)
    {
        output << components[i] << ":";
        std::cout << components[i] << " | ";
    }
    
    if(!output.good())
        std::cout << " FAILED! -- Make sure the manager is running with sudo!";
    
    std::cout << std::endl;
    
    output.close();
    
    return true;
}

void Manager::unregister_old_chan(const std::string &name)
{
    _unregister(_chan_roster, name);
    _report_no_error(UNREGISTER_OLD_CHAN);
}

void Manager::_unregister(const std::string &directory, const std::string &name)
{
    StringArray array = _grab_files_in_dir(directory);
    
    for(size_t i=0; i < array.size(); ++i)
    {
        if( array[i] == name )
        {
            remove( (directory+"/"+name).c_str() );
        }
    }
}

void Manager::reset_rosters()
{
    load_config("default");
}

void Manager::start_up()
{
    // TODO: Hardware-related things
    
    create_all_ach_chans();
    run_all_processes();
}

void Manager::shut_down()
{
    stop_all_processes();
    close_all_ach_chans();
    
    // TODO: Hardware-related things
}

void Manager::list_configs()
{
    _relay_string_array(LIST_CONFIGS, _grab_files_in_dir(_config_roster));
}

void Manager::save_current_config(const std::string &name)
{
    _save_config_raw(name);
    
    _report_no_error(SAVE_CONFIG);
}

void Manager::_save_config_raw(const std::string &name)
{
    std::ofstream output;
    output.open( (_config_roster+"/"+name).c_str() );
    
    StringArray procs = _grab_files_in_dir(_proc_roster);
    for(size_t i=0; i < procs.size(); ++i)
    {
        output << "proc:" << _stringify_contents(_proc_roster, procs[i]) << "\n";
    }
    
    StringArray chans = _grab_files_in_dir(_chan_roster);
    for(size_t i=0; i < chans.size(); ++i)
    {
        output << "chan:" << _stringify_contents(_chan_roster, chans[i]) << "\n";
    }
    
    output.close();
}

std::string Manager::_stringify_contents(const std::string &directory, const std::string &name)
{
    std::ifstream input;
    input.open( (directory+"/"+name).c_str() );
    
    std::string result;
    
    if(input.good())
    {
        std::getline(input, result);
    }
    
    result = name + ":" + result;
    
    return result;
}

void Manager::load_config(const std::string &name)
{
    StringArray configs = _grab_files_in_dir(_config_roster);
    bool config_existed = false;
    bool correctly_formed = true;
    for(size_t i=0; i < configs.size(); ++i)
    {
        if(name == configs[i])
        {
            config_existed = true;
            
            correctly_formed = _load_config_raw(name);
        }
    }
    
    if(config_existed)
    {
        if(correctly_formed)
            _report_no_error(LOAD_CONFIG);
        else
            _report_malformed_error("The config file which you requested is malformed");
    }
    else
    {
        _report_no_existence(LOAD_CONFIG);
    }
    
}

bool Manager::_load_config_raw(const std::string &name)
{
    _clear_current_config();
    
    std::ifstream config;
    config.open( (_config_roster+"/"+name).c_str() );
    
    while(config.good())
    {
        std::string next_line;
        std::getline(config, next_line);
        
        StringArray components;
        
        if(_split_components(next_line, components) < 1)
        {
            continue;
        }
        
        if(components[0] == "proc" || components[0] == "chan")
        {
            std::ofstream output;
            output.open( (manager_directory+"/"+components[0]).c_str() );
            
            for(size_t i=1; i < components.size(); ++i)
            {
                output << components[i] << ":";
            }
            
            output.close();
        }
    }
    
    return true;
}


void Manager::_clear_current_config()
{
    _save_config_raw("last_config");
    
    StringArray procs = _grab_files_in_dir(_proc_roster);
    for(size_t i=0; i < procs.size(); ++i)
    {
        remove( (_proc_roster+"/"+procs[i]).c_str() );
    }
    
    StringArray chans = _grab_files_in_dir(_chan_roster);
    for(size_t i=0; i < chans.size(); ++i)
    {
        remove( (_chan_roster+"/"+procs[i]).c_str() );
    }
}

size_t Manager::_split_components(const std::string &name, StringArray &array)
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

void Manager::_report_no_error(manager_cmd_t original_req)
{
    manager_reply_t reply;
    memset(&reply, 0, sizeof(manager_reply_t));
    
    strcpy(reply.reply, "");
    reply.err = NO_ERROR;
    reply.original_req = original_req;
    reply.replyID = 0;
    reply.numReplies = 1;
    
    ach_put(&_reply_chan, &reply, sizeof(manager_reply_t));
}

void Manager::_report_error(manager_err_t error, const std::string &description)
{
    std::cerr << "Failed request (" << (int)error << "): " << description << std::endl;
    
    manager_reply_t reply;
    memset(&reply, 0, sizeof(manager_reply_t));
    
    strncpy(reply.reply, description.c_str(), sizeof(reply.reply));
    reply.err = error;
    reply.original_req = UNKNOWN_REQUEST;
    reply.replyID = 0;
    reply.numReplies = 1;
    
    ach_put(&_reply_chan, &reply, sizeof(manager_reply_t));
}

void Manager::_report_ach_error(const std::string &error_description)
{
    _report_error(ACH_ERROR, error_description);
}

void Manager::_report_malformed_error(const std::string &error_description)
{
    _report_error(MALFORMED_REQUEST, error_description);
}

void Manager::_report_no_existence(manager_cmd_t original_req)
{
    _report_error(NONEXISTENT_ENTRY, "");
}

bool Manager::_fork_process(const std::string &proc_name, const std::string &args)
{
    StringArray procs = _grab_files_in_dir(_proc_roster);
    
    bool existed = false;
    for(size_t i=0; i < procs.size(); ++i)
    {
        if(procs[i] == proc_name)
        {
            existed = true;
            
            _fork_process_raw(proc_name, args);
        }
    }
    
    return existed;
}

void Manager::_fork_process_raw(const std::string &proc_name, std::string args)
{
    std::ifstream proc_file;
    proc_file.open( (_proc_roster+"/"+proc_name).c_str() );
    
    if(proc_file.good())
    {
        std::string proc_desc;
        std::getline(proc_file, proc_desc);
        StringArray components;
        if(_split_components(proc_desc, components) < 2)
        {
            std::cerr << "Proc named " << proc_name
                      << " has incorrect number of parameters: " << components.size()
                      << "\n -- This should equal 2" << std::endl;
            return;
        }
        
        if(args == "")
        {
            args = components[1];
        }
        
        pid_t child = fork();
        if(child == 0)
        {
            int exit_code = system( (components[0] + " " + args).c_str() );
            std::cout << "Process " << proc_name << " exited with code " << exit_code << std::endl;
            exit(0);
        }
    }
}
