#include "cpvelocity/CPVelocity.h"
#include "ui_CPVelocity.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control_panel::CPVelocityPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{

velocityConfigDialog::velocityConfigDialog(QString topic, bool labelIsVisible, QString labelText, bool valLabelIsVisible, enum MessageFlags _messageFlags) : 
    layout(new QGridLayout),
    topicedit(new QLineEdit(topic)),
    slabelcheck(new QCheckBox),
    svallabelcheck(new QCheckBox),
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

    QLabel *svallabeltxt = new QLabel(tr("Show value Labels:"));
    svallabelcheck->setChecked(valLabelIsVisible);
    layout->addWidget(svallabeltxt, 3, 0);
    layout->addWidget(svallabelcheck, 3, 1);

    QLabel *typetxt = new QLabel(tr("Type:"));
    connect(typebox, SIGNAL(currentIndexChanged(int)), this, SLOT(typeChangeCB(int)));
    typebox->addItem(tr("Twist"), (int)MessageTypeTwist);
    if(messageFlags & MessageTypeTwist)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("Odom"), (int)MessageTypeOdom);
    if(messageFlags & MessageTypeOdom)
        typebox->setCurrentIndex(typebox->count() - 1);
    layout->addWidget(typetxt, 4, 0);
    layout->addWidget(typebox, 4, 1);

    QLabel *stampedtxt = new QLabel(tr("Stamped:"));
    if(messageFlags & MessageStamped)
        stampedcheck->setChecked(true);
    else
        stampedcheck->setChecked(false);
    layout->addWidget(stampedtxt, 5, 0);
    layout->addWidget(stampedcheck, 5, 1);
    
    QLabel *withcovtxt = new QLabel(tr("Covariance:"));
    if(messageFlags & MessageWithCov)
        withcovcheck->setChecked(true);
    else
        withcovcheck->setChecked(false);
    layout->addWidget(withcovtxt, 6, 0);
    layout->addWidget(withcovcheck, 6, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 7, 1);

    this->setLayout(layout);

    this->setWindowTitle("Plugin Configuration - Velocity");

    connect(okbutton, SIGNAL(clicked()), this, SLOT(accept()));
}

void velocityConfigDialog::typeChangeCB(int idx)
{
    if(idx == 0)
    {
        stampedcheck->setEnabled(true);
        withcovcheck->setEnabled(true);
    }
    else if(idx == 1)
    {
        stampedcheck->setChecked(true);
        stampedcheck->setEnabled(false);
        withcovcheck->setChecked(true);
        withcovcheck->setEnabled(false);
    }
}

CPVelocityPlugin::CPVelocityPlugin() :
    ui(new Ui::CPVelocityPlugin),
    topic("cp_velocity"),
    messageFlags((enum MessageFlags)(MessageTypeOdom | MessageStamped | MessageWithCov)),
    nodelet_priv(new CPSSN())
{
    ui->setupUi(this);
    connect(this, SIGNAL(changeLinearX(const QString &)), ui->xLinear, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeLinearY(const QString &)), ui->yLinear, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeLinearZ(const QString &)), ui->zLinear, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeAngularX(const QString &)), ui->xAngular, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeAngularY(const QString &)), ui->yAngular, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeAngularZ(const QString &)), ui->zAngular, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeLabel(const QString &)), ui->label, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->xLinear, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->yLinear, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->zLinear, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->xAngular, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->yAngular, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->zAngular, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->label, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->linLabel, SLOT(setEnabled(bool)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->angLabel, SLOT(setEnabled(bool)));
    emit changeEnabled(false);
    emit changeLabel("CPVelocity:");
    ui->linLabel->setText("Linear:");
    ui->angLabel->setText("Angular:");
}

CPVelocityPlugin::~CPVelocityPlugin()
{
    delete ui;
}

void CPVelocityPlugin::start()
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

void CPVelocityPlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    emit changeEnabled(false);
    nodelet_priv->deactivate();
}

void CPVelocityPlugin::setup()
{
    emit changeLabel(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    ui->label->setVisible(settings->value(uuid.toString() + "/ShowLabel", true).toBool());
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    messageFlags = (enum MessageFlags)settings->value(uuid.toString() + "/Type", (int)messageFlags).toInt();
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPVelocityPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPVelocityPlugin::displayValues(double xLin, double yLin, double zLin, double xAng, double yAng, double zAng)
{ 
    emit changeLinearX(QString::number(xLin, 'f', 4));
    emit changeLinearY(QString::number(yLin, 'f', 4));
    emit changeLinearZ(QString::number(zLin, 'f', 4));
    
    emit changeAngularX(QString::number(xAng, 'f', 4));
    emit changeAngularY(QString::number(yAng, 'f', 4));
    emit changeAngularZ(QString::number(zAng, 'f', 4));
}

void CPVelocityPlugin::twistCB(const geometry_msgs::TwistConstPtr &msg)
{
    CPVelocityPlugin::displayValues(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
}

void CPVelocityPlugin::twistStampedCB(const geometry_msgs::TwistStampedConstPtr &msg)
{
    CPVelocityPlugin::displayValues(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
}

void CPVelocityPlugin::twistWithCovarianceCB(const geometry_msgs::TwistWithCovarianceConstPtr &msg)
{
    CPVelocityPlugin::displayValues(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
}

void CPVelocityPlugin::twistWithCovarianceStampedCB(const geometry_msgs::TwistWithCovarianceStampedConstPtr &msg)
{
    CPVelocityPlugin::displayValues(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
}

void CPVelocityPlugin::odometryCB(const nav_msgs::OdometryConstPtr &msg)
{
    CPVelocityPlugin::displayValues(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
}

void CPVelocityPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPVelocityPlugin::contextMenuEvent(QContextMenuEvent *event)
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

void CPVelocityPlugin::configDialog()
{
    bool reactivate = false;
    enum MessageFlags newMessageFlags;

    velocityConfigDialog *dialog = new velocityConfigDialog(topic, ui->label->isVisible(), ui->label->text(), ui->linLabel->isVisible(), messageFlags);


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
        emit changeLinearX("N/A");
        emit changeLinearY("N/A");
        emit changeLinearZ("N/A");

        emit changeAngularX("N/A");
        emit changeAngularY("N/A");
        emit changeAngularZ("N/A");
        activateNodelet(true);
    }

    if(ui->label->isVisible() != dialog->slabelcheck->isChecked())
    {
        ui->label->setVisible(dialog->slabelcheck->isChecked());
        settings->setValue(uuid.toString() + "/ShowLabel", dialog->slabelcheck->isChecked());
    }

    if(ui->linLabel->isVisible() != dialog->svallabelcheck->isChecked())
    {
        ui->linLabel->setVisible(dialog->svallabelcheck->isChecked());
        ui->angLabel->setVisible(dialog->svallabelcheck->isChecked());
        settings->setValue(uuid.toString() + "/ShowValLabel", dialog->svallabelcheck->isChecked());
    }

    if(ui->label->text() != dialog->labeledit->text())
    {
        emit changeLabel(dialog->labeledit->text());
        settings->setValue(uuid.toString() + "/Label", dialog->labeledit->text());
    }
}

void CPVelocityPlugin::activateNodelet(bool passive)
{
    if(messageFlags & MessageTypeTwist)
    {
        if(messageFlags & MessageStamped)
        {
            if(messageFlags & MessageWithCov)
                nodelet_priv->activate<const geometry_msgs::TwistWithCovarianceStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPVelocityPlugin::twistWithCovarianceStampedCB, this, _1), passive);
            else
                nodelet_priv->activate<const geometry_msgs::TwistStamped_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPVelocityPlugin::twistStampedCB, this, _1), passive);
        }
        else if(messageFlags & MessageWithCov)
            nodelet_priv->activate<const geometry_msgs::TwistWithCovariance_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPVelocityPlugin::twistWithCovarianceCB, this, _1), passive);
        else
            nodelet_priv->activate<const geometry_msgs::Twist_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPVelocityPlugin::twistCB, this, _1), passive);
    }    
    else if(messageFlags & MessageTypeOdom)
            nodelet_priv->activate<const nav_msgs::Odometry_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPVelocityPlugin::odometryCB, this, _1), passive);
}
}
