#include "cporientation/CPOrientation.h"
#include "ui_CPOrientation.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control_panel::CPOrientationPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{

orientationConfigDialog::orientationConfigDialog(QString topic, bool labelIsVisible, QString labelText, enum MessageFlags _messageFlags) : 
    layout(new QGridLayout),
    topicedit(new QLineEdit(topic)),
    slabelcheck(new QCheckBox),
    labeledit(new QLineEdit(labelText)),
    typebox(new QComboBox),
    stampedcheck(new QCheckBox),
    withcovcheck(new QCheckBox)
{
    messageFlags = _messageFlags;

    QLabel *topictxt = new QLabel(tr("Topic Name:"));
    layout->addWidget(topictxt, 0, 0);
    layout->addWidget(topicedit, 0, 1);

    QLabel *slabeltxt = new QLabel(tr("Show Label:"));
    slabelcheck->setChecked(labelIsVisible);
    layout->addWidget(slabeltxt, 1, 0);
    layout->addWidget(slabelcheck, 1, 1);

    QLabel *labeltxt = new QLabel(tr("Label:"));
    connect(slabelcheck, SIGNAL(toggled(bool)), labeledit, SLOT(setEnabled(bool)));
    labeledit->setEnabled(labelIsVisible);
    layout->addWidget(labeltxt, 2, 0);
    layout->addWidget(labeledit, 2, 1);

    QLabel *typetxt = new QLabel(tr("Type:"));
    connect(typebox, SIGNAL(currentIndexChanged(int)), this, SLOT(typeChangeCB(int)));
    typebox->addItem(tr("Quaternion"), (int)MessageTypeQuat);
    if(messageFlags & MessageTypeQuat)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("Pose"), (int)MessageTypePose);
    if(messageFlags & MessageTypePose)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("Odom"), (int)MessageTypeOdom);
    if(messageFlags & MessageTypeOdom)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("IMU"), (int)MessageTypeImu);
    if(messageFlags & MessageTypeImu)
        typebox->setCurrentIndex(typebox->count() - 1);
    layout->addWidget(typetxt, 3, 0);
    layout->addWidget(typebox, 3, 1);

    QLabel *stampedtxt = new QLabel(tr("Stamped:"));
    if(messageFlags & MessageStamped)
        stampedcheck->setChecked(true);
    else
        stampedcheck->setChecked(false);
    layout->addWidget(stampedtxt, 4, 0);
    layout->addWidget(stampedcheck, 4, 1);
    
    QLabel *withcovtxt = new QLabel(tr("Covariance:"));
    if(messageFlags & MessageWithCov)
        withcovcheck->setChecked(true);
    else
        withcovcheck->setChecked(false);
    layout->addWidget(withcovtxt, 5, 0);
    layout->addWidget(withcovcheck, 5, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 6, 1);

    this->setLayout(layout);

    this->setWindowTitle("Plugin Configuration - Orientation");

    connect(okbutton, SIGNAL(clicked()), this, SLOT(accept()));
}

void orientationConfigDialog::typeChangeCB(int idx)
{
    if(idx == 0)
    {
        stampedcheck->setEnabled(true);
        withcovcheck->setChecked(false);
        withcovcheck->setEnabled(false);
    }
    else if(idx == 1)
    {
        stampedcheck->setEnabled(true);
        withcovcheck->setEnabled(true);
    }
    else if(idx == 2)
    {
        stampedcheck->setChecked(true);
        stampedcheck->setEnabled(false);
        withcovcheck->setChecked(true);
        withcovcheck->setEnabled(false);
    }
    else if(idx == 3)
    {
        stampedcheck->setChecked(true);
        stampedcheck->setEnabled(false);
        withcovcheck->setChecked(true);
        withcovcheck->setEnabled(false);
    }
}

CPOrientationPlugin::CPOrientationPlugin() :
    ui(new Ui::CPOrientationPlugin),
    topic("cp_orientation"),
    messageFlags((enum MessageFlags)(MessageTypeOdom | MessageStamped | MessageWithCov)),
    nodelet_priv(new CPSSN())
{
    ui->setupUi(this);
    connect(this, SIGNAL(changeX(const QString &)), ui->xValue, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeY(const QString &)), ui->yValue, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeZ(const QString &)), ui->zValue, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeLabel(const QString &)), ui->label, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->xValue, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->yValue, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->zValue, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->label, SLOT(setEnabled(bool)));
    emit changeEnabled(false);
    emit changeLabel("CP Orientation:");
}

CPOrientationPlugin::~CPOrientationPlugin()
{
    delete ui;
}

void CPOrientationPlugin::start()
{
    if(!ros::ok())
    {
        ROS_WARN_STREAM("Tried to start a nodelet that wasn't ready!");
        return;
    }
    settings->setValue(uuid.toString() + "/Active", true);
    activateNodelet();
    emit changeEnabled(true);
}

void CPOrientationPlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    emit changeEnabled(false);
    nodelet_priv->deactivate();
}

void CPOrientationPlugin::setup()
{
    emit changeLabel(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    ui->label->setVisible(settings->value(uuid.toString() + "/ShowLabel", true).toBool());
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    messageFlags = (enum MessageFlags)settings->value(uuid.toString() + "/Type", (int)messageFlags).toInt();
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPOrientationPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPOrientationPlugin::displayValues(double x, double y, double z, double w)
{ 

    double roll = atan2(2.0 * (y*z + w*x), w*w - x*x - y*y + z*z) * 57.2957795;
    double pitch = asin(-2.0 * (x*z - w*y)) * 57.2957795;
    double yaw = atan2(2.0 * (x*y + w*z), w*w + x*x - y*y - z*z) * 57.2957795;

    emit changeX(QString::number(roll,'f', 4));
    emit changeY(QString::number(pitch, 'f', 4));
    emit changeZ(QString::number(yaw, 'f', 4));
}

void CPOrientationPlugin::quaternionCB(const geometry_msgs::QuaternionConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->x, msg->y, msg->z, msg->w);
}

void CPOrientationPlugin::quaternionStampedCB(const geometry_msgs::QuaternionStampedConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);
}

void CPOrientationPlugin::poseCB(const geometry_msgs::PoseConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void CPOrientationPlugin::poseStampedCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void CPOrientationPlugin::poseWithCovarianceCB(const geometry_msgs::PoseWithCovarianceConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void CPOrientationPlugin::poseWithCovarianceStampedCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void CPOrientationPlugin::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void CPOrientationPlugin::imuCB(const sensor_msgs::ImuConstPtr &msg)
{
    CPOrientationPlugin::displayValues(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void CPOrientationPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPOrientationPlugin::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu;
    menu.addAction("Enable");
    menu.actions()[0]->setCheckable(true);
    menu.actions()[0]->setChecked(nodelet_priv->isActive());
    connect(menu.actions()[0], SIGNAL(toggled(bool)), this, SLOT(setActive(bool)));
    menu.addAction("Configure", this, SLOT(configDialog()));
    menu.addAction("Delete", this, SLOT(delete_self()));
    menu.exec(event->globalPos());
}

void CPOrientationPlugin::configDialog()
{
    bool reactivate = false;
    enum MessageFlags newMessageFlags;

    orientationConfigDialog *dialog = new orientationConfigDialog(topic, ui->label->isVisible(), ui->label->text(), messageFlags);


    if(!dialog->exec())
        return;

    if(topic != dialog->topicedit->text())
    {
        topic = dialog->topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        reactivate = true;
    }

    newMessageFlags = (enum MessageFlags)(dialog->typebox->itemData(dialog->typebox->currentIndex()).toInt() | (dialog->stampedcheck->isChecked() ? MessageStamped : MessageNone ) | (dialog->withcovcheck->isChecked() ? MessageWithCov : MessageNone ));

    if(newMessageFlags != messageFlags)
        reactivate = true;

    messageFlags = newMessageFlags; 
    settings->setValue(uuid.toString() + "/Type", (int)messageFlags);

    if (reactivate)
    {
        emit changeX("N/A");
        emit changeY("N/A");
        emit changeZ("N/A");
        activateNodelet(true);
    }

    if(ui->label->isVisible() != dialog->slabelcheck->isChecked())
    {
        ui->label->setVisible(dialog->slabelcheck->isChecked());
        settings->setValue(uuid.toString() + "/ShowLabel", dialog->slabelcheck->isChecked());
    }

    if(ui->label->text() != dialog->labeledit->text())
    {
        emit changeLabel(dialog->labeledit->text());
        settings->setValue(uuid.toString() + "/Label", dialog->labeledit->text());
    }
}

void CPOrientationPlugin::activateNodelet(bool passive)
{
    if(messageFlags & MessageTypeQuat)
    {
        if(messageFlags & MessageStamped)
            nodelet_priv->activate<const geometry_msgs::QuaternionStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::quaternionStampedCB, this, _1), passive);
        else
            nodelet_priv->activate<const geometry_msgs::Quaternion_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::quaternionCB, this, _1), passive);
    }
    else if(messageFlags & MessageTypePose)
    {
        if(messageFlags & MessageStamped)
        {
            if(messageFlags & MessageWithCov)
                nodelet_priv->activate<const geometry_msgs::PoseWithCovarianceStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::poseWithCovarianceStampedCB, this, _1), passive);
            else
                nodelet_priv->activate<const geometry_msgs::PoseStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::poseStampedCB, this, _1), passive);
        }
        else if(messageFlags & MessageWithCov)
            nodelet_priv->activate<const geometry_msgs::PoseWithCovariance_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::poseWithCovarianceCB, this, _1), passive);
        else
            nodelet_priv->activate<const geometry_msgs::Pose_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::poseCB, this, _1), passive);
    }    
    else if(messageFlags & MessageTypeOdom)
            nodelet_priv->activate<const nav_msgs::Odometry_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::odometryCB, this, _1), passive);
    else if(messageFlags & MessageTypeImu)
            nodelet_priv->activate<const sensor_msgs::Imu_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPOrientationPlugin::imuCB, this, _1), passive);
}
}
