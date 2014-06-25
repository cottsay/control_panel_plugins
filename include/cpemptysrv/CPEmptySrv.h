#ifndef CPEMPTYSRV_H
#define CPEMPTYSRV_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <boost/shared_ptr.hpp>

namespace Ui {
class CPEmptySrvPlugin;
}

namespace control_panel
{

class CPEmptySrvNodelet : public nodelet::Nodelet
{
public:
    CPEmptySrvNodelet( ) :
        srvIsValid(false)
    {
    }

    void activate(const std::string topic, bool passive = false)
    {
        if(srv)
        {
            srv.shutdown();
            ros::NodeHandle nh = getNodeHandle();
            srv = nh.serviceClient<std_srvs::Empty>(topic);
            srvIsValid = true;
        }
        else if(!passive)
        {
            ros::NodeHandle nh = getNodeHandle();
            srv = nh.serviceClient<std_srvs::Empty>(topic);
            srvIsValid = true;
        }
    }

    void onInit()
    {
    }

    void deactivate()
    {
        srvIsValid = false;
        srv.shutdown();
    }

    bool isActive()
    {
        return (srvIsValid);
    }

    bool call( )
    {
      static std_srvs::Empty msg;
      srv.call(msg);
    }

private:
    bool srvIsValid;
    ros::ServiceClient srv;
};

class CPEmptySrvPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPEmptySrvPlugin();
    ~CPEmptySrvPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);
    void call();

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);  
 
private:
    Ui::CPEmptySrvPlugin *ui;
    QString topic;
    CPEmptySrvNodelet *nodelet_priv;

};
}

#endif // CPEMPTYSRV_H
