#!/bin/sh

ShowUsage()
{
    echo "There are two modes of installation:"
    echo " "
    echo " -- robot -- "
    echo " This is for installing HuboCan on the physical Hubo itself."
    echo " When running the script, pass in 'robot' followed by the"
    echo " version of Hubo which you have. Ex:"
    echo "$ ./setup robot Hubo2Plus"
    echo " "
    echo " Currently supported versions include:"
    echo " - Hubo2Plus"
    echo " - DrcHubo"
    echo " "
    echo " -- workstation -- "
    echo " This is for installing HuboCan on a remote workstation"
    echo " (i.e. a computer which is for developing code or for"
    echo " an operator to use). When running the script, simply"
    echo " pass in 'workstation'. Ex:"
    echo "$ ./setup workstation"
    echo " "
}

CreateOptHubo()
{
    if [ -d "/opt/hubo" ]
    then
        echo '/opt/hubo already exists'
    else
        echo 'Creating /opt/hubo'
        sudo mkdir /opt/hubo
        sudo chmod a+rwx /opt/hubo
    fi
}

CopyDevices()
{
    echo 'Copying device files into /opt/hubo/devices'
    cp -r ../HuboCan/devices /opt/hubo/
}

AutostartManager()
{
    echo 'Adding hubomgr to the upstart list'
    sudo cp ../misc/hubomgr.conf /etc/init/hubomgr.conf
}

RawRobotInstall()
{
    echo "Performing robot installation for $2"

    # I hate bash scripts
    if [ -d "build" ]
    then
        echo 'Build directory was found'
    else
        echo 'Making build directory'
        mkdir build
    fi

    cd build
    cmake .. -DBuildHuboQt=OFF -DCMAKE_INSTALL_PREFIX=/usr
    make
    sudo make install

    CreateOptHubo
    CopyDevices
    AutostartManager

    /usr/bin/hubocan_setup_default_config "$2"
}

RobotInstall()
{
    case "$1" in

        '')
            ShowUsage
            exit 2
        ;;

        *)
            RawRobotInstall $@
        ;;

    esac
}

WorkstationInstall()
{
    if [ -d "build" ]
    then
        echo 'Build directory was found'
    else
        echo 'Making build directory'
        mkdir build
    fi

    cd build
    cmake .. -DBuildHuboQt=ON -DCMAKE_INSTALL_PREFIX=/usr
    make
    sudo make install

    CreateOptHubo
    CopyDevices
}

case "$1" in
    'robot')
        RobotInstall $@
    ;;

    'workstation')
        WorkstationInstall $@
    ;;

    *)
        ShowUsage
        exit 1
    ;;

esac

exit 0
