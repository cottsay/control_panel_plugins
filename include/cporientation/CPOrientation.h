#ifndef CPORIENTATION_H
#define CPORIENTATION_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <boost/shared_ptr.hpp>

#include <QMenu>
#include <QGridLayout>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>

namespace Ui {
class CPOrientationPlugin;
}

namespace control_panel
{

enum MessageFlags
{
    MessageNone = 0,
    MessageStamped = (1 << 0),
    MessageWithCov = (1 << 1),
    MessageTypeQuat = (1 << 2),
    MessageTypePose = (1 << 3),
    MessageTypeOdom = (1 << 4),
    MessageTypeImu = (1 << 5)
};

class orientationConfigDialog : public QDialog
{
    Q_OBJECT
    
public slots:
    void typeChangeCB(int idx);

public:
    orientationConfigDialog(QString topic, bool labelIsVisible, QString labelText, enum MessageFlags _messageFlags);

    QGridLayout *layout;
    QLineEdit *topicedit;
    QCheckBox *slabelcheck;
    QLineEdit *labeledit;
    QComboBox *typebox;
    QCheckBox *stampedcheck;
    QCheckBox *withcovcheck;

private:
    enum MessageFlags messageFlags;
};

class CPOrientationPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPOrientationPlugin();
    ~CPOrientationPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);

signals:
    void changeX(const QString &);
    void changeY(const QString &);
    void changeZ(const QString &);
    void changeLabel(const QString &);
    void changeEnabled(bool);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);

private:
    void displayValues(double x, double y, double z, double w);

    void quaternionCB(const geometry_msgs::QuaternionConstPtr &msg);
    void quaternionStampedCB(const geometry_msgs::QuaternionStampedConstPtr &msg);
    void poseCB(const geometry_msgs::PoseConstPtr &msg);
    void poseStampedCB(const geometry_msgs::PoseStampedConstPtr &msg);
    void poseWithCovarianceCB(const geometry_msgs::PoseWithCovarianceConstPtr &msg);
    void poseWithCovarianceStampedCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void odometryCB(const nav_msgs::OdometryConstPtr &msg);
    void imuCB(const sensor_msgs::ImuConstPtr &msg);

    Ui::CPOrientationPlugin *ui;
    QString topic;
    enum MessageFlags messageFlags;
    CPSSN *nodelet_priv;
};
}

#endif // CPORIENTATION_H
