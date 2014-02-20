#ifndef CPVELOCITY_H
#define CPVELOCITY_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <boost/shared_ptr.hpp>

#include <QMenu>
#include <QGridLayout>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>

namespace Ui {
class CPVelocityPlugin;
}

namespace control_panel
{

enum MessageFlags
{
    MessageNone = 0,
    MessageStamped = (1 << 0),
    MessageWithCov = (1 << 1),
    MessageTypeTwist = (1 << 2),
    MessageTypeOdom = (1 << 3)
};

class velocityConfigDialog : public QDialog
{
    Q_OBJECT
    
public slots:
    void typeChangeCB(int idx);

public:
    velocityConfigDialog(QString topic, bool labelIsVisible, QString labelText, bool valLableIsVisible, enum MessageFlags _messageFlags);

    QGridLayout *layout;
    QLineEdit *topicedit;
    QCheckBox *slabelcheck;
    QLineEdit *labeledit;
    QCheckBox *svallabelcheck;
    QComboBox *typebox;
    QCheckBox *stampedcheck;
    QCheckBox *withcovcheck;

private:
    enum MessageFlags messageFlags;
};

class CPVelocityPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPVelocityPlugin();
    ~CPVelocityPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);

private:
    void displayValues(double xLin, double yLin, double zLin, double xAng, double yAng, double zAng);

    void twistCB(const geometry_msgs::TwistConstPtr &msg);
    void twistStampedCB(const geometry_msgs::TwistStampedConstPtr &msg);
    void twistWithCovarianceCB(const geometry_msgs::TwistWithCovarianceConstPtr &msg);
    void twistWithCovarianceStampedCB(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg);
    void odometryCB(const nav_msgs::OdometryConstPtr &msg);

    Ui::CPVelocityPlugin *ui;
    QString topic;
    enum MessageFlags messageFlags;
    CPSSN *nodelet_priv;
};
}

#endif // CPVELOCITY_H
