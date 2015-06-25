/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <greyxmike@gmail.com>
 *
 * Humanoid Robotics Lab
 *
 * Directed by Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>

#include "FlowLayout.h"
#include "HuboQt/JointWidget.h"

namespace HuboQt {

const QString joint_button_style = "background-color: "
        "qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dadbde, stop: 1 ";

void JointButton::_normify()
{
    QColor color(230,230,230);
    setStyleSheet(joint_button_style + color.name() + ")");
}

void JointButton::_reddify()
{
    QColor color(200,0,0);
    setStyleSheet(joint_button_style + color.name() + ")");
}

void JointButton::_purpify()
{
    QColor color(200,0,200);
    setStyleSheet(joint_button_style + color.name() + ")");
}

JointButton::JointButton(const hubo_joint_info_t& info)
{
    _degrees = false;
    _info = info;
    memset(&_state, 0, sizeof(_state));
    update();
}

void JointButton::useDegrees()
{
    _degrees = true;
    update();
}

void JointButton::useRadians()
{
    _degrees = false;
    update();
}

void JointButton::update(const hubo_joint_state_t &state)
{
    _state = state;
    update();
}

void JointButton::_setErrorFlags()
{
    QString tip = QString::fromLocal8Bit(_info.name) + ": ";
    double value = _state.position;
    if(_degrees)
        value = value*180.0/M_PI;
    tip += QString::number(value, 'f');

    hubo_joint_status& status = _state.status;
    bool homed = true;
    if(status.home_flag != 6) // TODO: Make this a part of HuboJoint so it can be overloaded?
    {
        tip += "\nNot Homed | "; homed = false;
    }
    else
    {
        tip += "\nHomed | "; homed = true;
    }

    tip += "Driver:";
    if(status.driver_on == 1)
        tip += "on";
    else
        tip += "off";

    tip += " | Control:";
    if(status.control_on == 1)
        tip += "on";
    else
        tip += "off";

    tip += " | Mode:" + QString::number(status.control_mode);
    tip += " | Switch:";
    if(status.limit_switch == 1)
        tip += "on";
    else
        tip += "off";

    _error = false;
    tip += "\nErrors:";
    hubo_joint_error_t& err = status.error;
    if(err.jam == 1)
    {
        tip += "\nJammed"; _error = true;
    }
    if(err.pwm_saturated == 1)
    {
        tip += "\nPWM Saturated"; _error = true;
    }
    if(err.big == 1)
    {
        tip += "\nBig Error"; _error = true;
    }
    if(err.encoder == 1)
    {
        tip += "\nEncoder Error"; _error = true;
    }
    if(err.driver_fault == 1)
    {
        tip += "\nDriver Fault"; _error = true;
    }
    if(err.motor_fail_0 == 1)
    {
        tip += "\nMotor Fail (Channel 0)"; _error = true;
    }
    if(err.motor_fail_1 == 1)
    {
        tip += "\nMotor Fail (Channel 1)"; _error = true;
    }
    if(err.min_position == 1)
    {
        tip += "\nMinimum Position Violated"; _error = true;
    }
    if(err.max_position == 1)
    {
        tip += "\nMaximum Position Violaed"; _error = true;
    }
    if(err.velocity == 1)
    {
        tip += "\nMax Velocity Violation"; _error = true;
    }
    if(err.acceleration == 1)
    {
        tip += "\nMax Acceleration Violation"; _error = true;
    }
    if(err.temperature == 1)
    {
        tip += "\nTemperature Error"; _error = true;
    }

    if(_error)
    {
        _reddify();
    }
    else if(homed)
    {
        _normify();
    }
    else
    {
        _purpify();
    }

    setToolTip(tip);
}

void JointButton::update()
{
    QString text = QString::fromLocal8Bit(_info.name);
    text += "\n";
    double value = _state.position;
    if(_degrees)
        value = value*180.0/M_PI;
    text += QString::number(value, 'f', 3);
    setText(text);

    _setErrorFlags();
}


JointGridWidget::JointGridWidget()
{
    setLayout(new FlowLayout);
    _initialized = false;
    _sptr = NULL;
}

HuboState::State* JointGridWidget::getStatePtr() const
{
    return _sptr;
}

void JointGridWidget::setStatePtr(HuboState::State* new_state_ptr)
{
    _sptr = new_state_ptr;

    if( NULL == _sptr )
        return;

    if(!_sptr->initialized())
    {
        if(!_sptr->receive_description(0))
        {
            std::cerr << "Cannot generate joint grid, because HuboState::State could not initialize"
                      << std::endl;
            _initialized = false;
            return;
        }
    }

    for(int i=0; i<_buttons.size(); ++i)
    {
        layout()->removeWidget(_buttons[i]);
        delete _buttons[i];
    }
    _buttons.clear();

    for(size_t i=0; i<_sptr->joints.size(); ++i)
    {
        const hubo_joint_info_t& info = _sptr->get_description().getJointInfo(i);

        JointButton* joint = new JointButton(info);
        _buttons.push_back(joint);
        layout()->addWidget(joint);
    }

    _initialized = true;
}

void JointGridWidget::update()
{
    if( NULL == _sptr )
        return;

    if(!_initialized)
    {
        setStatePtr(_sptr);
        if(!_initialized)
            return;
    }

    for(int i=0; i<_buttons.size(); ++i)
    {
        _buttons[i]->update(_sptr->joints[i]);
    }
}

JointWidget::JointWidget()
{
    setLayout(new QVBoxLayout);

    grid = new JointGridWidget;
    layout()->addWidget(grid);
}

void JointWidget::initialize()
{
    if( NULL == state )
    {
        state = new HuboState::State(0);
        grid->setStatePtr(state);
    }
}

} // namespace HuboQt
