#!/bin/sh

ShowUsage()
{
    echo "There are two modes of installation:"
    echo " "
    echo " -- robot -- "
    echo " This is for installing HuboCan on the physical Hubo itself."
    echo " When running the script, pass in 'robot' followed by the"
    echo " version of Hubo which you have. Ex:"
    echo "   $ ./setup robot Hubo2Plus"
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
    echo "   $ ./setup workstation"
    echo " "
}

MakeBuildDirectory()
{
    if [ -d "build" ]
    then
        echo 'Build directory was found'
        new_build_dir=0
    else
        echo 'Making build directory'
        mkdir build
        new_build_dir=1
    fi
}

SetupAptList()
{
    echo 'Updating golems.list'
    sudo cp misc/golems.list /etc/apt/sources.list.d/golems.list
}

GetDependencies()
{
    sudo apt-get update
    sudo apt-get install libach-dev ach-utils
    sudo apt-get install libeigen3-dev
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
    echo 'Setting hubomgr to autolaunch after boot'
#    sudo cp ../misc/hubomgr.conf /etc/init/hubomgr.conf

    sudo cp ../misc/hubo-manager /etc/init.d/hubo-manager
    sudo chmod a+x /etc/init.d/hubo-manager
    sudo update-rc.d hubo-manager defaults
}

RemoveAutostart()
{
    sudo update-rc.d -f hubo-manager remove
}

RawRobotInstall()
{
    echo "Performing robot installation for $2"

    SetupAptList
    GetDependencies
    MakeBuildDirectory

    cd build
    cmake .. -DBuildHuboQt=OFF -DCMAKE_INSTALL_PREFIX=/usr
    make
    sudo make install

    CreateOptHubo
    CopyDevices
    AutostartManager

    /usr/bin/hubocan_setup_default_config "$2"

    echo 'robot' > /opt/hubo/install_type
}

RobotInstall()
{
    case "$2" in

        '')
            ShowUsage
            exit 2
        ;;

        'Hubo2Plus')
            RawRobotInstall $@
        ;;

        'DrcHubo')
            RawRobotInstall $@
        ;;

        *)
            ShowUsage
            exit 2
        ;;

    esac
}

WorkstationInstall()
{
    SetupAptList
    GetDependencies
    sudo apt-get install libqt4-dev

    MakeBuildDirectory

    cd build
    cmake .. -DBuildHuboQt=ON -DCMAKE_INSTALL_PREFIX=/usr
    make
    sudo make install

    CreateOptHubo
    CopyDevices

    echo 'workstation' > /opt/hubo/install_type
}

HuboUninstall()
{
    MakeBuildDirectory

    cd build

    if [ -e "/opt/hubo/install_type" ]
    then
        install_type=$(sudo cat /opt/hubo/install_type)
    else
        install_type=workstation
    fi

    if [ "$new_build_dir" -eq 1 ]
    then
        echo 'Since a build directory did not exist, we must compile and install before uninstalling'
        if [ "$install_type" = 'robot' ]
        then
            cmake .. -DBuildHuboQt=OFF -DCMAKE_INSTALL_PREFIX=/usr
        else
            cmake .. -DBuildHuboQt=ON  -DCMAKE_INSTALL_PREFIX=/usr
        fi

        make
        sudo make install
    fi

    sudo make uninstall

    if [ "$install_type" = 'robot' ]
    then
        sudo update-rc.d -f hubo-manager remove
        sudo rm -f /etc/init.d/hubo-manager
    fi
}

case "$1" in
    'robot')
        RobotInstall $@
    ;;

    'workstation')
        WorkstationInstall $@
    ;;

    'uninstall')
        HuboUninstall
    ;;

    *)
        ShowUsage
        exit 1
    ;;

esac

exit 0
