#ifndef ACHDHANDLE_HPP
#define ACHDHANDLE_HPP

#include <QString>
#include <QProcess>

#include "HuboRT/manager_msg.hpp"

namespace HuboQt {

class AchdHandle
{
public:
    
    AchdHandle();
    ~AchdHandle();
    
    QString robot_hostname;
    
    QString channel_description;
    QString nickname;
    QString channel_name;
    int nominal_size;
    int message_count;
    achd_network_t push_or_pull;
    
    QProcess achd_process;
    
    bool start(QString hostname);
    bool start(QString hostname, QString description);
    bool started;
    
    void stop();
    
    bool parse_description(QString description);   
};


} // namespace HuboQt

#endif // ACHDHANDLE_HPP
