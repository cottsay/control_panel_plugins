#ifndef CPRANGE_H
#define CPRANGE_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <boost/shared_ptr.hpp>

namespace Ui {
class CPRangePlugin;
}

namespace control_panel
{
class CPRangePlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPRangePlugin();
    ~CPRangePlugin();
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
    void rangeCB(const sensor_msgs::Range::ConstPtr &msg);

    Ui::CPRangePlugin *ui;
    QString topic;
    CPSSN *nodelet_priv;

};
}

#endif // CPRANGE_H
