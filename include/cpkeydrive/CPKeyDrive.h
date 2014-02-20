#ifndef CPKEYDRIVE_H
#define CPKEYDRIVE_H

#include <QContextMenuEvent>
#include <QThread>
#include <QTimer>
#include <QMutex>

#include <nodelet/nodelet.h>
#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <boost/signals2/mutex.hpp>

namespace Ui {
class CPKeyDrivePlugin;
}

namespace control_panel
{
class CPKeyDrivePlugin;

class CPKeyDriveNodelet : public nodelet::Nodelet
{
public:
    CPKeyDriveNodelet(CPKeyDrivePlugin *_parent);
    ~CPKeyDriveNodelet();
    void activate(const std::string topic, bool passive = false);
    void deactivate();
    bool isActive();
    void onInit();
    bool publishMessage(const geometry_msgs::Twist &msg);
private:
    ros::Publisher cmd_vel_pub;
    CPKeyDrivePlugin *parent;
};

class CPKeyDrivePlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPKeyDrivePlugin();
    ~CPKeyDrivePlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);
    void keyDownCB(QKeyEvent *event);
    void keyUpCB(QKeyEvent *event);
    void timerCB();

protected:
    void contextMenuEvent(QContextMenuEvent *event);

    Ui::CPKeyDrivePlugin *ui;
    QString topic;
    QString name;
    CPKeyDriveNodelet *nodelet_priv;
    double linear_x_velocity;
    double linear_y_velocity;
    double angular_velocity;
    QThread pub_thread;
    QTimer pub_timer;
    QMutex pub_mutex;
    geometry_msgs::Twist pub_twist;
    bool w;
    bool a;
    bool s;
    bool d;
    bool q;
    bool e;

friend class CPKeyDriveNodelet;
};
}

#endif // CPDOUBLE_H
