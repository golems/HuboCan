#ifndef JOINTWIDGET_H
#define JOINTWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QVector>
#include <QString>

#include "HuboState/State.hpp"

namespace HuboQt {

class JointButton : public QPushButton
{
    Q_OBJECT

public:

    JointButton(const hubo_joint_info_t& info);

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

};

class JointGridWidget : public QWidget
{
    Q_OBJECT

public:

    explicit JointGridWidget(HuboState::State* state);

    HuboState::State* getStatePtr() const;
    void setStatePtr(HuboState::State* new_state_ptr);

    void update();

protected:

    HuboState::State* _sptr;
    bool _initialized;

    QVector<JointButton*> _buttons;

};

class JointWidget : public QWidget
{
    Q_OBJECT

public:

    JointWidget();

    JointGridWidget* grid;

    HuboState::State state;

};

} // namespace HuboQt


#endif // JOINTWIDGET_H
