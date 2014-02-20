#ifndef CPJOYTOTWIST_H
#define CPJOYTOTWIST_H

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>
#include <joy_to_twist/joy_to_twist_nodelet.hpp>

#include <ros/ros.h>

namespace Ui {
class CPJoyToTwistPlugin;
}

namespace control_panel
{
class CPJoyToTwistPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPJoyToTwistPlugin();
    ~CPJoyToTwistPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    
private:
    Ui::CPJoyToTwistPlugin *ui;
    float linear_scale;
    float angular_scale;
    joy_to_twist::joy_to_twist_nodelet *nodelet_priv;
};
}

#endif // CPJOYTOTWIST_H
