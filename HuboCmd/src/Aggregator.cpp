
#include "../Aggregator.hpp"
#include <stdio.h>
#include <stdlib.h>

using namespace HuboCan;

Aggregator::Aggregator(const HuboDescription& description)
{
    _initialize();
    load_description(description);
}

void Aggregator::_initialize()
{
    _input_data = NULL;
    _output_data = NULL;
    _final_data = NULL;

    _channels_opened = false;
    open_channels();
}

bool Aggregator::open_channels()
{
    if(_channels_opened)
        return true;

    _channels_opened = true;

    ach_status_t result = ach_open(&_cmd_chan, HUBO_CMD_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        fprintf(stderr, "Error opening command channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;
        return false;
    }

    result = ach_open(&_agg_chan, HUBO_AGG_CHANNEL, NULL);
    if( ACH_OK != result )
    {
        fprintf(stderr, "Error opening aggregated channel: %s (%d)\n",
                ach_result_to_string(result), (int)result);
        _channels_opened = false;

        ach_close(&_cmd_chan);
        return false;
    }

    return true;
}

void Aggregator::_create_memory()
{
    free(_input_data);
    free(_output_data);
    free(_final_data);

    if(_desc.getJointCount() > 0)
    {
        _input_data  = hubo_cmd_init_data( _desc.getJointCount() );
        _output_data = hubo_cmd_init_data( _desc.getJointCount() );
        _final_data  = hubo_cmd_init_data( _desc.getJointCount() );
    }
    else
    {
        _input_data  = NULL;
        _output_data = NULL;
        _final_data  = NULL;
    }
}

void Aggregator::load_description(const HuboDescription &desc)
{
    _desc = desc;
    _create_memory();
}

Aggregator::~Aggregator()
{
    free(_input_data);
    free(_output_data);
    free(_final_data);

    ach_close(&_cmd_chan);
    ach_close(&_agg_chan);
}





