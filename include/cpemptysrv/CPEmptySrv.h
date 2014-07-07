#ifndef CPEMPTYSRV_H
#define CPEMPTYSRV_H

#include "cpssn/CPSSN.h"

#include <QCheckBox>
#include <QContextMenuEvent>
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>

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

    void activate(const std::string topic, const std::string joyTopic, const boost::function<void(const boost::shared_ptr<sensor_msgs::Joy>&)> cb, bool passive = false)
    {
        this->cb = cb;

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

        if(joy_sub)
            joy_sub.shutdown();

        if(srv && !joyTopic.empty())
        {
            ros::NodeHandle nh = getNodeHandle();
            joy_sub = nh.subscribe<sensor_msgs::Joy>(joyTopic, 10, cb);
        }
    }

    void onInit()
    {
    }

    void deactivate()
    {
        srvIsValid = false;
        if(srv)
            srv.shutdown();
        if(joy_sub)
            joy_sub.shutdown();
    }

    bool isActive()
    {
        return (srvIsValid);
    }

    bool call( )
    {
      static std_srvs::Empty msg;
      if(srv)
          srv.call(msg);
    }

private:
    bool srvIsValid;
    ros::ServiceClient srv;
    ros::Subscriber joy_sub;
    boost::function<void(const boost::shared_ptr<sensor_msgs::Joy>&)> cb;
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
    void JoyCB(const sensor_msgs::Joy::ConstPtr &msg);
    void setJoyEn(const bool en);
    void setJoyTopic(const QString &topic);

signals:
    void changeEnabled(bool);
    void joyDetect(int);
    void pressButton();

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);  
 
private:
    Ui::CPEmptySrvPlugin *ui;
    QString topic;
    bool joyEn;
    QString joyTopic;
    int joyIdx;
    CPEmptySrvNodelet *nodelet_priv;

};

class CPEmptySrvCfg : public QDialog
{
    Q_OBJECT

public:
    CPEmptySrvCfg(const QString &topic, const QString &label, const bool joyEn, const QString &joyTopic, unsigned int joyIdx);

    QGridLayout layout;

    QLabel topictxt;
    QLineEdit topicedit;

    QLabel labeltxt;
    QLineEdit labeledit;

    QLabel joytxt;
    QCheckBox joycheck;

    QLabel joytopictxt;
    QLineEdit joytopicedit;

    QLabel joyidxtxt;
    QGridLayout joyidxlayout;
    QLineEdit joyidxedit;
    QPushButton joyidxbutton;

    QPushButton okbutton;

signals:
    void updateJoyTopic(const QString &topic);

public slots:
    void joyChange(bool en);
    void joyAuto();
    void joyDetect(int button);

private:
    bool detecting;
};
}

#endif // CPEMPTYSRV_H
