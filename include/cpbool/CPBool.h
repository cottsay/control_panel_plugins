#ifndef CPBOOL_H
#define CPBOOL_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <boost/shared_ptr.hpp>

namespace Ui {
class CPBoolPlugin;
}

namespace control_panel
{
class CPBoolPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPBoolPlugin();
    ~CPBoolPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);

signals:
    void changeValue(const QString &);
    void changeLabel(const QString &);
    void changeEnabled(bool);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);  
 
private:
    void boolCB(const std_msgs::BoolConstPtr &msg);

    Ui::CPBoolPlugin *ui;
    QString topic;
    CPSSN *nodelet_priv;

};
}

#endif // CPBOOL_H
