

#include "AchIncludes.h"
#include "Manager.hpp"
#include "manager_msg.h"
#include "Daemonizer_C.h"
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
    
    _proc_roster = proc_roster_directory;
    hubo_rt_safe_make_directory(_proc_roster.c_str());
    
    _chan_roster = chan_roster_directory;
    hubo_rt_safe_make_directory(_chan_roster.c_str());
    
    _config_roster = config_directory;
    hubo_rt_safe_make_directory(_config_roster.c_str());
    
    _create_channel(hubo_rt_mgr_cmd_chan, 20, 2048);
    _create_channel(hubo_rt_mgr_reply_chan, 20, 2048);
    
    ach_status_t r = ach_open(&_msg_chan, hubo_rt_mgr_cmd_chan, NULL);
    _rt.check(ACH_OK == r, "Could not open the Hubo Manager request channel (" + std::string(hubo_rt_mgr_cmd_chan) + ")", true);
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
    
    system(command_stream.str().c_str());
}

void Manager::launch()
{
    _rt.daemonize("hubo-manager");

    run();
}

void Manager::run()
{
    manager_msg_t incoming_msg;
    while(_rt.good())
    {
        size_t fs;
        ach_status_t r = ach_get(&_msg_chan, &incoming_msg, sizeof(manager_msg_t),
                                 &fs, NULL, ACH_O_WAIT);
        if( ACH_OK != r )
        {
            std::cerr << "Ach error: (" << (int)r << ")" << ach_result_to_string(r) << std::endl;
            _report_ach_error(ach_result_to_string(r));
            continue;
        }
        
        if( sizeof(manager_msg_t) != fs )
        {
            std::cerr << "Incoming message has a malformed size of " << fs
                         << "\n -- Should be size " << sizeof(manager_msg_t) << std::endl;
            _report_malformed_error("");
            continue;
        }
        
        switch(incoming_msg.request_type)
        {
            case LIST_PROCS: list_processes();                                  break;
            case LIST_LOCKED_PROCS: list_locked_processes();                    break;
            case LIST_CHANS: list_channels();                                   break;
//            case LIST_OPEN_CHANS: list_open_channels();                         break;
                
            case RUN_PROC: run_process(incoming_msg.name);                      break;
            case RUN_ALL_PROCS: run_all_processes();                            break;
            
            case STOP_PROC: stop_process(incoming_msg.name);                    break;
            case STOP_ALL_PROCS: stop_all_processes();                          break;
                
            case KILL_PROC: kill_process(incoming_msg.name);                    break;
            case KILL_ALL_PROCS: kill_all_processes();                          break;
                
            case CREATE_ACH_CHAN: create_ach_chan(incoming_msg.name);           break;
            case CREATE_ALL_ACH_CHANS: create_all_ach_chans();                  break;
            
            case CLOSE_ACH_CHAN: close_ach_chan(incoming_msg.name);             break;
            case CLOSE_ALL_ACH_CHANS: close_all_ach_chans();                    break;
                
            case REGISTER_NEW_PROC: register_new_proc(incoming_msg.name);       break;
            case UNREGISTER_OLD_PROC: unregister_old_proc(incoming_msg.name);   break;
//            case RESET_PROC_ROSTER: reset_proc_roster();                        break;
                
            case REGISTER_NEW_CHAN: register_new_chan(incoming_msg.name);       break;
            case UNREGISTER_OLD_CHAN: unregister_old_chan(incoming_msg.name);   break;
//            case RESET_CHAN_ROSTER: reset_chan_roster();                        break;
                
            case RESET_ROSTERS: reset_rosters();                                break;
                
            case START_UP: start_up();                                          break;
            case SHUT_DOWN: shut_down();                                        break;
                
            case LIST_CONFIGS: list_configs();                                  break;
            case SAVE_CONFIG: save_current_config(incoming_msg.name);           break;
            case LOAD_CONFIG: load_config(incoming_msg.name);                   break;
                
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
        _fork_process(components[0], components[1]);
        _report_no_error(RUN_PROC);
    }
    else
        _report_malformed_error("Run process should have exactly two components");
}

void Manager::run_all_processes()
{
    StringArray procs = _grab_files_in_dir(_proc_roster);
    for(size_t i=0; i < procs.size(); ++i)
    {
        _fork_process(procs[i], "");
    }
    _report_no_error(RUN_ALL_PROCS);
}

void Manager::_stop_process_raw(const std::string &name)
{
    int id = 0;
    std::ifstream str;
//            if(!str.good())
//            {
//                // Handle error here? Can this ever not be good?
//            }
    str.open( (_rt_lock_dir+"/"+name).c_str());
    
    str >> id;
    if( id > 0 )
    {
        std::cout << "Stopping process named '" << name << "' with id " << id << std::endl;
        pid_t processID = id;
        kill(processID, SIGINT);
    }
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
    std::ifstream chan_desc;
    chan_desc.open( (_chan_roster+"/"+name).c_str() );
    if(chan_desc.good())
    {
        size_t count=0, fs=0;
        std::string chan_name;
        chan_desc >> chan_name >> count >> fs;
        _create_channel(chan_name, count, fs);
        
        return true;
    }
    else
    {
        return false;
    }
}

bool Manager::_close_ach_channel_raw(const std::string &name)
{
    std::ifstream chan_desc;
    chan_desc.open( (_chan_roster+"/"+name).c_str() );
    if(chan_desc.good())
    {
        size_t count=0, fs=0;
        std::string channel_name;
        chan_desc >> channel_name >> count >> fs;
        
        std::stringstream command_stream;
        command_stream << "ach rm " << channel_name;
        
        system(command_stream.str().c_str());
        std::cout << "Closing ach channel " << name << " (" << channel_name << ")" << std::endl;
        
        return true;
    }
    else
    {
        std::cout << "Could not find a channel associated with '" << name << "' in the roster" << std::endl;
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

void Manager::register_new_proc(const std::string &name)
{
    if(_register(_proc_roster, name) == 3)
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
    if(_register(_chan_roster, name) == 4)
    {
        _report_no_error(REGISTER_NEW_CHAN);
    }
    else
    {
        _report_malformed_error("REGISTER_CHAN did not have the correct number of arguments in the description");
    }
}

void Manager::unregister_old_chan(const std::string &name)
{
    _unregister(_chan_roster, name);
    _report_no_error(UNREGISTER_OLD_CHAN);
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
    std::ofstream output;
    output.open( (_config_roster+"/"+name).c_str() );
    
    // TODO: FINISH THIS
}
