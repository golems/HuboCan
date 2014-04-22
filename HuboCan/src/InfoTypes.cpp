
#include "../InfoTypes.hpp"
#include <sstream>

using namespace HuboCan;

error_result_t::error_result_t()
{
    result = OKAY;
}

error_result_t::error_result_t(error_flag flag)
{
    result = flag;
}

std::string HuboCan::error_result_to_string(error_result_t error)
{
    std::stringstream stream;
    if(error.result==0)
    {
        return "OKAY";
    }
    
    bool first = true;
    for(size_t i=0; i <= max_error_flag_bit_location; ++i)
    {
        if( ((error.result>>i) & 0x01) == 1 )
        {
            if(!first)
            {
                stream << " | ";
            }
            else
            {
                first = false;
            }
            
            switch(i)
            {
                case 0: stream << "UNDEFINED_ERROR";        break;
                case 1: stream << "ARRAY_MISMATCH";         break;
                case 2: stream << "ACH_ERROR";              break;
                case 3: stream << "READ_ONLY";              break;
                case 4: stream << "INCOMPATIBLE_JOINT";     break;
                case 5: stream << "INDEX_OUT_OF_BOUNDS";    break;
                case 6: stream << "SYNCH_ERROR";            break;
                case 7: stream << "TIMEOUT";                break;
                case 8: stream << "UNINITIALIZED";          break;
                case 9: stream << "INTERRUPTED";            break;

                default:stream << "UNKNOWN";                break;
            }
        }
    }
    
    return stream.str();
}

bool error_result_t::operator ==(const error_result_t& error)
{
    if(result == error.result)
    {
        return true;
    }
    
    return false;
}

bool error_result_t::operator !=(const error_result_t& error)
{
    return !(*this == error);
}

bool error_result_t::operator ==(const error_flag& flag)
{
    if(flag == 0)
        return (result == 0);
    
    if( (result & flag) == 0 )
    {
        return false;
    }
    
    return true;
}

bool error_result_t::operator !=(const error_flag& flag)
{
    return !(*this == flag);
}

error_result_t& error_result_t::operator =(const error_flag& flag)
{
    result = flag;
    return *this;
}

error_result_t& error_result_t::operator |=(const error_flag& flag)
{
    result |= flag;
    return *this;
}

error_result_t& error_result_t::operator |=(const error_result_t& error)
{
    result |= error.result;
    return *this;
}

error_result_t& error_result_t::operator &=(const error_flag& flag)
{
    result &= flag;
    return *this;
}

error_result_t& error_result_t::operator &=(const error_result_t& error)
{
    result &= error.result;
    return *this;
}












