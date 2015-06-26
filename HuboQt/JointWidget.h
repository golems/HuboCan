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

#ifndef HUBOQT_JOINTWIDGET_H
#define HUBOQT_JOINTWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QVector>
#include <QString>
#include <QTextEdit>
#include <QTimer>

#include "HuboState/State.hpp"
#include "HuboCmd/AuxSender.hpp"

class QDoubleSpinBox;

namespace HuboQt {

class JointGridWidget;

class JointButton : public QPushButton
{
    Q_OBJECT

public:

    JointButton(const hubo_joint_info_t& info, JointGridWidget* grid);

    void useRadians();
    void useDegrees();

    void update(const hubo_joint_state_t& state);
    void update();

protected:

    void _setErrorFlags();

    void _normify();
    void _reddify();
    void _purpify();

    bool _degrees;
    bool _error;
    hubo_joint_state_t _state;
    hubo_joint_info_t _info;

    JointGridWidget* _grid;

};

class JointGridWidget : public QWidget
{
    Q_OBJECT

public:

    explicit JointGridWidget(QTextEdit* text);

    HuboState::State* getStatePtr() const;
    void setStatePtr(HuboState::State* new_state_ptr);

    void update();

    QVector<JointButton*> buttons;

    QButtonGroup* group;

    QTextEdit* errorBox;

    QString errorText;

protected:

    HuboState::State* _sptr;
    bool _initialized;

};

class JointWidget : public QWidget
{
    Q_OBJECT

public:

    JointWidget();

    JointGridWidget* grid;

    HuboState::State* state;

    HuboCmd::AuxSender* sender;

    QDoubleSpinBox* period;

    QTimer* timer;

protected:

    QWidget* _createJointCommandOptions();

    QButtonGroup* _commandGroup;

    QWidget* _createBottomBar();

public Q_SLOTS:

    void initialize();

    void reinitialize();

    void useDegrees();

    void useRadians();

    void handleJointButtonPress(int joint);

    void update();
};

} // namespace HuboQt

#endif // HUBOQT_JOINTWIDGET_H
