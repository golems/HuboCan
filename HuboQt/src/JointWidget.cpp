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

#include <QRadioButton>
#include <QButtonGroup>
#include <QSpacerItem>
#include <QDoubleSpinBox>

#include "FlowLayout.h"
#include "HuboQt/JointWidget.h"

const QString no_op_text = "Nothing";
const QString home_text = "Home";
const QString ctrl_on_text = "Ctrl On";
const QString ctrl_off_text = "Ctrl Off";

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

JointButton::JointButton(const hubo_joint_info_t& info, JointGridWidget* grid)
    : _degrees(false),
      _info(info),
      _grid(grid)
{
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

void JointButton::update(const hubo_joint_state_t& state)
{
    _state = state;
    update();
}

#define REPORT_ERROR( X ) { tip += "\n" #X ; errorText += " | " #X ; _error = true; }

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
    QString errorText;
    hubo_joint_error_t& err = status.error;
    if(err.jam == 1)
        REPORT_ERROR( Jammed );

    if(err.pwm_saturated == 1)
        REPORT_ERROR( PWM Saturated );

    if(err.big == 1)
        REPORT_ERROR( Big Error );

    if(err.encoder == 1)
        REPORT_ERROR( Encoder Error );

    if(err.driver_fault == 1)
        REPORT_ERROR( Driver Fault );

    if(err.motor_fail_0 == 1)
        REPORT_ERROR( Motor Fail - Channel 0 );

    if(err.motor_fail_1 == 1)
        REPORT_ERROR( Motor Fail - Channel 1 );

    if(err.min_position == 1)
        REPORT_ERROR( Minimum Position Violated );

    if(err.max_position == 1)
        REPORT_ERROR( Maximum Position Violated );

    if(err.velocity == 1)
        REPORT_ERROR( Maximum Velocity Violated );

    if(err.acceleration == 1)
        REPORT_ERROR( Maximum Acceleration Violated );

    if(err.temperature == 1)
        REPORT_ERROR( Temperature Error );

    if(errorText.size() > 0)
    {
        errorText.prepend(_info.name);
        errorText.prepend(" :: ");
        errorText.append("\n");
        _grid->errorText += errorText;
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


JointGridWidget::JointGridWidget(QTextEdit* text)
    : errorBox(text)
{
    setLayout(new FlowLayout);
    _initialized = false;
    _sptr = NULL;
    group = new QButtonGroup;
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
            _initialized = false;
            return;
        }
    }

    for(int i=0; i<buttons.size(); ++i)
    {
        layout()->removeWidget(buttons[i]);
        delete buttons[i];
    }
    buttons.clear();

    for(size_t i=0; i<_sptr->joints.size(); ++i)
    {
        const hubo_joint_info_t& info = _sptr->get_description().getJointInfo(i);

        JointButton* joint = new JointButton(info, this);
        buttons.push_back(joint);
        layout()->addWidget(joint);
        group->addButton(joint, i);
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

    _sptr->update(0, false);

    errorText.clear();
    for(int i=0; i<buttons.size(); ++i)
    {
        buttons[i]->update(_sptr->joints[i]);
    }
    errorBox->setText(errorText);
}

JointWidget::JointWidget()
    : state(NULL)
{
    setLayout(new QVBoxLayout);

    QTextEdit* errorText = new QTextEdit;
    errorText->setReadOnly(true);

    timer = new QTimer;

    grid = new JointGridWidget(errorText);
    connect(grid->group, SIGNAL(buttonClicked(int)), this, SLOT(handleJointButtonPress(int)));

    layout()->addWidget(_createJointCommandOptions());
    layout()->addWidget(grid);
    layout()->addWidget(errorText);
    layout()->addWidget(_createBottomBar());

    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
}

void JointWidget::initialize()
{
    if( NULL == state )
    {
        reinitialize();
    }
}

void JointWidget::reinitialize()
{
    delete state;

    std::cout << "Initializing JointWidget Grid" << std::endl;
    state = new HuboState::State(0);
    grid->setStatePtr(state);

    sender = new HuboCmd::AuxSender;

    if(state->initialized() && !timer->isActive())
        timer->start(100*period->value(), false);
}

#define CREATE_CMD_BUTTON( X ) QRadioButton* X ## _button = new QRadioButton( X ); \
                               _commandGroup->addButton( X ## _button );           \
                               layout->addWidget( X ## _button );

QWidget* JointWidget::_createJointCommandOptions()
{
    QWidget* CommandWidget = new QWidget;
    FlowLayout* layout = new FlowLayout;
    CommandWidget->setLayout(layout);

    _commandGroup = new QButtonGroup;
    _commandGroup->setExclusive(true);

    CREATE_CMD_BUTTON( no_op_text );    // Use this button to ensure that nothing will happen when a
                                        // joint is clicked
    CREATE_CMD_BUTTON( home_text );
    CREATE_CMD_BUTTON( ctrl_on_text );
    CREATE_CMD_BUTTON( ctrl_off_text );

    no_op_text_button->setChecked(true);

    return CommandWidget;
}

QWidget* JointWidget::_createBottomBar()
{
    QWidget* DegRadWidget = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout;
    DegRadWidget->setLayout(layout);

    QButtonGroup* radioGroup = new QButtonGroup;
    radioGroup->setExclusive(true);

    QRadioButton* degrees = new QRadioButton("Degrees");
    radioGroup->addButton(degrees);
    layout->addWidget(degrees);
    connect(degrees, SIGNAL(clicked()), this, SLOT(useDegrees()));


    QRadioButton* radians = new QRadioButton("Radians");
    radioGroup->addButton(radians);
    layout->addWidget(radians);
    connect(radians, SIGNAL(clicked()), this, SLOT(useRadians()));

    radians->setChecked(true);

    period = new QDoubleSpinBox;
    period->setToolTip("Set the period (seconds) for refreshing joint data");
    period->setValue(0.2);
    period->setSingleStep(0.05);
    period->setMinimum(0.01);
    timer->setInterval(100*period->value());
    layout->addWidget(period);

    layout->addItem(new QSpacerItem(0,0, QSizePolicy::MinimumExpanding));

    QPushButton* reinitializer = new QPushButton("Reinitialize");
    connect(reinitializer, SIGNAL(clicked()), this, SLOT(reinitialize()));
    layout->addWidget(reinitializer);

    return DegRadWidget;
}

void JointWidget::useDegrees()
{
    for(int i=0; i<grid->buttons.size(); ++i)
        grid->buttons[i]->useDegrees();
}

void JointWidget::useRadians()
{
    for(int i=0; i<grid->buttons.size(); ++i)
        grid->buttons[i]->useRadians();
}

void JointWidget::handleJointButtonPress(int joint)
{
    if(NULL == state)
        return;

    if(_commandGroup->checkedButton() == NULL)
        return;

    if(_commandGroup->checkedButton()->text() == home_text)
        sender->home_joint(joint);
    else if(_commandGroup->checkedButton()->text() == ctrl_on_text)
        sender->joint_ctrl_on(joint);
    else if(_commandGroup->checkedButton()->text() == ctrl_off_text)
        sender->joint_ctrl_off(joint);
}

void JointWidget::update()
{
    if(NULL == state)
    {
        reinitialize();
    }
    else
    {
        grid->update();
    }

    timer->setInterval(1000*period->value());
}

} // namespace HuboQt
