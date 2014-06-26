#include "cpposition/CPPosition.h"
#include "ui_CPPosition.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control_panel::CPPositionPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{

positionConfigDialog::positionConfigDialog(QString topic, bool labelIsVisible, QString labelText, enum MessageFlags _messageFlags) : 
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
    typebox->addItem(tr("Point"), (int)MessageTypePoint);
    if(messageFlags & MessageTypePoint)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("Pose"), (int)MessageTypePose);
    if(messageFlags & MessageTypePose)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("Odom"), (int)MessageTypeOdom);
    if(messageFlags & MessageTypeOdom)
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

    this->setWindowTitle("Plugin Configuration - Position");

    connect(okbutton, SIGNAL(clicked()), this, SLOT(accept()));
}

void positionConfigDialog::typeChangeCB(int idx)
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
}

CPPositionPlugin::CPPositionPlugin() :
    ui(new Ui::CPPositionPlugin),
    topic("cp_position"),
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
    emit changeLabel("CPPosition:");
}

CPPositionPlugin::~CPPositionPlugin()
{
    delete ui;
}

void CPPositionPlugin::start()
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

void CPPositionPlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    emit changeEnabled(false);
    nodelet_priv->deactivate();
}

void CPPositionPlugin::setup()
{
    emit changeLabel(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    ui->label->setVisible(settings->value(uuid.toString() + "/ShowLabel", true).toBool());
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    messageFlags = (enum MessageFlags)settings->value(uuid.toString() + "/Type", (int)messageFlags).toInt();
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPPositionPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPPositionPlugin::displayValues(double x, double y, double z)
{ 
    emit changeX(QString::number(x, 'f', 4));
    emit changeY(QString::number(y, 'f', 4));
    emit changeZ(QString::number(z, 'f', 4));
}

void CPPositionPlugin::pointCB(const geometry_msgs::PointConstPtr &msg)
{
    CPPositionPlugin::displayValues(msg->x, msg->y, msg->z);
}

void CPPositionPlugin::pointStampedCB(const geometry_msgs::PointStampedConstPtr &msg)
{
    CPPositionPlugin::displayValues(msg->point.x, msg->point.y, msg->point.z);
}

void CPPositionPlugin::poseCB(const geometry_msgs::PoseConstPtr &msg)
{
    CPPositionPlugin::displayValues(msg->position.x, msg->position.y, msg->position.z);
}

void CPPositionPlugin::poseStampedCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
    CPPositionPlugin::displayValues(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void CPPositionPlugin::poseWithCovarianceCB(const geometry_msgs::PoseWithCovarianceConstPtr &msg)
{
    CPPositionPlugin::displayValues(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void CPPositionPlugin::poseWithCovarianceStampedCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    CPPositionPlugin::displayValues(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void CPPositionPlugin::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
    CPPositionPlugin::displayValues(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void CPPositionPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPPositionPlugin::contextMenuEvent(QContextMenuEvent *event)
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

void CPPositionPlugin::configDialog()
{
    bool reactivate = false;
    enum MessageFlags newMessageFlags;

    positionConfigDialog *dialog = new positionConfigDialog(topic, ui->label->isVisible(), ui->label->text(), messageFlags);


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

void CPPositionPlugin::activateNodelet(bool passive)
{
    if(messageFlags & MessageTypePoint)
    {
        if(messageFlags & MessageStamped)
            nodelet_priv->activate<const geometry_msgs::PointStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPPositionPlugin::pointStampedCB, this, _1), passive);
        else
            nodelet_priv->activate<const geometry_msgs::Point_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPPositionPlugin::pointCB, this, _1), passive);
    }
    else if(messageFlags & MessageTypePose)
    {
        if(messageFlags & MessageStamped)
        {
            if(messageFlags & MessageWithCov)
                nodelet_priv->activate<const geometry_msgs::PoseWithCovarianceStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPPositionPlugin::poseWithCovarianceStampedCB, this, _1), passive);
            else
                nodelet_priv->activate<const geometry_msgs::PoseStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPPositionPlugin::poseStampedCB, this, _1), passive);
        }
        else if(messageFlags & MessageWithCov)
            nodelet_priv->activate<const geometry_msgs::PoseWithCovariance_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPPositionPlugin::poseWithCovarianceCB, this, _1), passive);
        else
            nodelet_priv->activate<const geometry_msgs::Pose_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPPositionPlugin::poseCB, this, _1), passive);
    }    
    else if(messageFlags & MessageTypeOdom)
            nodelet_priv->activate<const nav_msgs::Odometry_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPPositionPlugin::odometryCB, this, _1), passive);
}
}
