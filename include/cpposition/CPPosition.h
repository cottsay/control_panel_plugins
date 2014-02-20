#ifndef CPPOSITION_H
#define CPPOSITION_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
class CPPositionPlugin;
}

namespace control_panel
{

enum MessageFlags
{
    MessageNone = 0,
    MessageStamped = (1 << 0),
    MessageWithCov = (1 << 1),
    MessageTypePoint = (1 << 2),
    MessageTypePose = (1 << 3),
    MessageTypeOdom = (1 << 4)
};

class positionConfigDialog : public QDialog
{
    Q_OBJECT
    
public slots:
    void typeChangeCB(int idx);

public:
    positionConfigDialog(QString topic, bool labelIsVisible, QString labelText, enum MessageFlags _messageFlags);

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

class CPPositionPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPPositionPlugin();
    ~CPPositionPlugin();
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
    void displayValues(double x, double y, double z);

    void pointCB(const geometry_msgs::PointConstPtr &msg);
    void pointStampedCB(const geometry_msgs::PointStampedConstPtr &msg);
    void poseCB(const geometry_msgs::PoseConstPtr &msg);
    void poseStampedCB(const geometry_msgs::PoseStampedConstPtr &msg);
    void poseWithCovarianceCB(const geometry_msgs::PoseWithCovarianceConstPtr &msg);
    void poseWithCovarianceStampedCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void odometryCB(const nav_msgs::OdometryConstPtr &msg);

    Ui::CPPositionPlugin *ui;
    QString topic;
    enum MessageFlags messageFlags;
    CPSSN *nodelet_priv;
};
}

#endif // CPPOSITION_H
