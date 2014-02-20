#include "cpkeydrive/CPKeyDrive.h"
#include "ui_CPKeyDrive.h"
#include <pluginlib/class_list_macros.h>
#include <QMenu>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>

PLUGINLIB_EXPORT_CLASS(control_panel::CPKeyDrivePlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
CPKeyDrivePlugin::CPKeyDrivePlugin() :
    ui(new Ui::CPKeyDrivePlugin),
    topic("cmd_vel"),
    nodelet_priv(new CPKeyDriveNodelet(this)),
    linear_x_velocity(0.35),
    linear_y_velocity(0.35),
    angular_velocity(1.8),
    w(false),
    a(false),
    s(false),
    d(false),
    q(false),
    e(false)
{
    ui->setupUi(this);
    ui->label->setText("CP Key Drive");
    ui->label->setEnabled(false);
    pub_timer.setInterval(50);
    pub_timer.moveToThread(&pub_thread);
    connect(&pub_timer, SIGNAL(timeout()), this, SLOT(timerCB()));
    connect(&pub_thread, SIGNAL(started()), &pub_timer, SLOT(start()));
}

CPKeyDrivePlugin::~CPKeyDrivePlugin()
{
    pub_timer.stop();
    pub_thread.quit();
    pub_thread.wait();
    delete ui;
}

void CPKeyDrivePlugin::start()
{
    if(!ros::ok())
    {
        ROS_WARN_STREAM("Tried to start a nodelet that wasn't ready!");
        return;
    }
    settings->setValue(uuid.toString() + "/Active", true);
    nodelet_priv->activate(topic.toStdString());
    emit setKeyCB(this, true);
    pub_thread.start();
    ui->label->setEnabled(true);
}

void CPKeyDrivePlugin::stop()
{
    ui->label->setEnabled(false);
    nodelet_priv->deactivate();
    pub_timer.stop();
    pub_thread.quit();
    pub_thread.wait();
    emit setKeyCB(this, false);
    settings->setValue(uuid.toString() + "/Active", false);
}

void CPKeyDrivePlugin::setup()
{
    ui->label->setText(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    linear_x_velocity = settings->value(uuid.toString() + "/LinearXVelocity", linear_x_velocity).toDouble();
    linear_y_velocity = settings->value(uuid.toString() + "/LinearYVelocity", linear_y_velocity).toDouble();
    angular_velocity = settings->value(uuid.toString() + "/AngularVelocity", angular_velocity).toDouble();
    pub_timer.setInterval(1000.0 / settings->value(uuid.toString() + "/PublishRate", 1000.0 / pub_timer.interval()).toDouble());
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPKeyDrivePlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPKeyDrivePlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPKeyDrivePlugin::keyDownCB(QKeyEvent *event)
{
    pub_mutex.lock();
    switch(event->key())
    {
    case 'W':
        w = true;
        pub_twist.linear.x = s ? 0.0 : linear_x_velocity;
        break;
    case 'A':
        a = true;
        pub_twist.angular.z = d ? 0.0 : angular_velocity;
        break;
    case 'S':
        s = true;
        pub_twist.linear.x = w ? 0.0 : -linear_x_velocity;
        break;
    case 'D':
        d = true;
        pub_twist.angular.z = a ? 0.0 : -angular_velocity;
        break;
    case 'Q':
        q = true;
        pub_twist.linear.y = e ? 0.0 : linear_y_velocity;
        break;
    case 'E':
        e = true;
        pub_twist.linear.y = q ? 0.0 : -linear_y_velocity;
        break;
    default:
        pub_mutex.unlock();
        return;
        break;
    }
    pub_mutex.unlock();
}

void CPKeyDrivePlugin::keyUpCB(QKeyEvent *event)
{
    pub_mutex.lock();
    switch(event->key())
    {
    case 'W':
        w = false;
        pub_twist.linear.x = s ? -linear_x_velocity : 0.0;
        break;
    case 'A':
        a = false;
        pub_twist.angular.z = d ? -angular_velocity : 0.0;
        break;
    case 'S':
        s = false;
        pub_twist.linear.x = w ? linear_x_velocity : 0.0;
        break;
    case 'D':
        d = false;
        pub_twist.angular.z = a ? angular_velocity : 0.0;
        break;
    case 'Q':
        q = false;
        pub_twist.linear.y = e ? -linear_y_velocity : 0.0;
        break;
    case 'E':
        e = false;
        pub_twist.linear.y = q ? linear_y_velocity : 0.0;
        break;
    default:
        pub_mutex.unlock();
        return;
        break;
    }
    pub_mutex.unlock();
}

void CPKeyDrivePlugin::timerCB()
{
    pub_mutex.lock();
    nodelet_priv->publishMessage(pub_twist);
    pub_mutex.unlock();
}

void CPKeyDrivePlugin::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu;
    menu.addAction("Enabled");
    menu.actions()[0]->setCheckable(true);
    menu.actions()[0]->setChecked(nodelet_priv->isActive());
    connect(menu.actions()[0], SIGNAL(toggled(bool)), this, SLOT(setActive(bool)));
    menu.addAction("Configure", this, SLOT(configDialog()));
    menu.addAction("Delete", this, SLOT(delete_self()));
    menu.exec(event->globalPos());
}

void CPKeyDrivePlugin::configDialog()
{
    QDialog dialog;
    QGridLayout *layout = new QGridLayout;

    QLabel *topictxt = new QLabel(tr("Topic Name:"));
    QLineEdit *topicedit = new QLineEdit(topic);
    layout->addWidget(topictxt, 0, 0);
    layout->addWidget(topicedit, 0, 1);

    QLabel *labeltxt = new QLabel(tr("Label:"));
    QLineEdit *labeledit = new QLineEdit(ui->label->text());
    layout->addWidget(labeltxt, 1, 0);
    layout->addWidget(labeledit, 1, 1);

    QLabel *linxveltxt = new QLabel(tr("Linear X Velocity:"));
    QLineEdit *linxveledit = new QLineEdit(QString::number(linear_x_velocity));
    layout->addWidget(linxveltxt, 2, 0);
    layout->addWidget(linxveledit, 2, 1);

    QLabel *linyveltxt = new QLabel(tr("Linear Y Velocity:"));
    QLineEdit *linyveledit = new QLineEdit(QString::number(linear_y_velocity));
    layout->addWidget(linyveltxt, 3, 0);
    layout->addWidget(linyveledit, 3, 1);

    QLabel *angveltxt = new QLabel(tr("Angular Velocity:"));
    QLineEdit *angveledit = new QLineEdit(QString::number(angular_velocity));
    layout->addWidget(angveltxt, 4, 0);
    layout->addWidget(angveledit, 4, 1);

    QLabel *ratetxt = new QLabel(tr("Publish Rate (Hz):"));
    QSpinBox *rateedit = new QSpinBox();
    rateedit->setMinimum(1);
    rateedit->setMaximum(100);
    rateedit->setValue(1000.0 / pub_timer.interval());
    layout->addWidget(ratetxt, 5, 0);
    layout->addWidget(rateedit, 5, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 6, 1);

    dialog.setLayout(layout);

    dialog.setWindowTitle("Plugin Configuration - Key Drive");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(topic != topicedit->text())
    {
        topic = topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        nodelet_priv->activate(topic.toStdString(), true);
    }

    if(ui->label->text() != labeledit->text())
    {
        ui->label->setText(labeledit->text());
        settings->setValue(uuid.toString() + "/Label", labeledit->text());
    }

    if(linxveledit->text().toDouble() != linear_x_velocity)
    {
        pub_mutex.lock();
        linear_x_velocity = linxveledit->text().toDouble();
        if(w && !s)
            pub_twist.linear.x = linear_x_velocity;
        else if(!w && s)
            pub_twist.linear.x = -linear_x_velocity;
        pub_mutex.unlock();
        settings->setValue(uuid.toString() + "/LinearXVelocity", linear_x_velocity);
    }

    if(linyveledit->text().toDouble() != linear_y_velocity)
    {
        pub_mutex.lock();
        linear_y_velocity = linyveledit->text().toDouble();
        if(q && !e)
            pub_twist.linear.y = linear_y_velocity;
        else if(!q && e)
            pub_twist.linear.y = -linear_y_velocity;
        pub_mutex.unlock();
        settings->setValue(uuid.toString() + "/LinearYVelocity", linear_y_velocity);
    }

    if(angveledit->text().toDouble() != angular_velocity)
    {
        pub_mutex.lock();
        angular_velocity = angveledit->text().toDouble();
        if(a && !d)
            pub_twist.angular.z = angular_velocity;
        else if(!a && d)
            pub_twist.angular.z = -angular_velocity;
        pub_mutex.unlock();
        settings->setValue(uuid.toString() + "/AngularVelocity", angular_velocity);
    }

    if(rateedit->value() != 1000.0 / pub_timer.interval())
    {
        pub_timer.setInterval(1000.0 / rateedit->value());
        settings->setValue(uuid.toString() + "/PublishRate", rateedit->value());
    }
}

CPKeyDriveNodelet::CPKeyDriveNodelet(CPKeyDrivePlugin *_parent)
    : parent(_parent)
{
}

CPKeyDriveNodelet::~CPKeyDriveNodelet()
{
    cmd_vel_pub.shutdown();
}

void CPKeyDriveNodelet::onInit()
{
}

void CPKeyDriveNodelet::activate(const std::string topic, bool passive)
{
    if(cmd_vel_pub)
    {
        if(cmd_vel_pub.getTopic() == topic)
            return;
        cmd_vel_pub.shutdown();
        ros::NodeHandle nh = getNodeHandle();
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic, 1, false);
    }
    else if(!passive)
    {
        ros::NodeHandle nh = getNodeHandle();
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic, 1, false);
    }
}

void CPKeyDriveNodelet::deactivate()
{
    cmd_vel_pub.shutdown();
}

bool CPKeyDriveNodelet::isActive()
{
    return (cmd_vel_pub);
}

bool CPKeyDriveNodelet::publishMessage(const geometry_msgs::Twist &msg)
{
    if(!cmd_vel_pub)
        return false;
    cmd_vel_pub.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist(msg)));
    return true;
}
}
