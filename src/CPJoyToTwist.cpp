#include "cpjoy_to_twist/CPJoyToTwist.h"
#include "ui_CPJoyToTwist.h"
#include <pluginlib/class_list_macros.h>
#include <QMenu>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>

PLUGINLIB_EXPORT_CLASS(control_panel::CPJoyToTwistPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
CPJoyToTwistPlugin::CPJoyToTwistPlugin() :
    ui(new Ui::CPJoyToTwistPlugin),
    linear_scale(0.35),
    angular_scale(1.8),
    nodelet_priv(new joy_to_twist::joy_to_twist_nodelet(false))
{
    ui->setupUi(this);
    connect(this, SIGNAL(changeLabel(const QString &)), ui->label, SLOT(setText(const QString &)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->label, SLOT(setEnabled(bool)));
    emit changeLabel("CP Joy To Twist");
    emit changeEnabled(false);
}

CPJoyToTwistPlugin::~CPJoyToTwistPlugin()
{
    delete ui;
}

void CPJoyToTwistPlugin::start()
{
    if(!ros::ok())
    {
        ROS_WARN_STREAM("Tried to start a nodelet that wasn't ready!");
        return;
    }
    settings->setValue(uuid.toString() + "/Active", true);
    nodelet_priv->start();
    emit changeEnabled(true);
}

void CPJoyToTwistPlugin::stop()
{
    emit changeEnabled(false);
    nodelet_priv->stop();
    settings->setValue(uuid.toString() + "/Active", false);
}

void CPJoyToTwistPlugin::setup()
{
    emit changeLabel(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    linear_scale = settings->value(uuid.toString() + "/LinearScale", linear_scale).toFloat();
    nodelet_priv->set_linear_scale(linear_scale);
    angular_scale = settings->value(uuid.toString() + "/AngularScale", angular_scale).toFloat();
    nodelet_priv->set_angular_scale(angular_scale);
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPJoyToTwistPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPJoyToTwistPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPJoyToTwistPlugin::contextMenuEvent(QContextMenuEvent *event)
{
    QMenu menu;
    menu.addAction("Enable");
    menu.actions()[0]->setCheckable(true);
    menu.actions()[0]->setChecked(nodelet_priv->stat());
    connect(menu.actions()[0], SIGNAL(toggled(bool)), this, SLOT(setActive(bool)));
    menu.addAction("Configure", this, SLOT(configDialog()));
    menu.addAction("Delete", this, SLOT(delete_self()));
    menu.exec(event->globalPos());
}

void CPJoyToTwistPlugin::configDialog()
{
    QDialog dialog;
    QGridLayout *layout = new QGridLayout;

    QLabel *labeltxt = new QLabel(tr("Label:"));
    QLineEdit *labeledit = new QLineEdit(ui->label->text());
    layout->addWidget(labeltxt, 0, 0);
    layout->addWidget(labeledit, 0, 1);

    QLabel *lineartxt = new QLabel(tr("Linear Scale:"));
    QLineEdit *linearedit = new QLineEdit(QString::number(linear_scale));
    layout->addWidget(lineartxt, 1, 0);
    layout->addWidget(linearedit, 1, 1);

    QLabel *angulartxt = new QLabel(tr("Angular Scale:"));
    QLineEdit *angularedit = new QLineEdit(QString::number(angular_scale));
    layout->addWidget(angulartxt, 2, 0);
    layout->addWidget(angularedit, 2, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 3, 1);

    dialog.setLayout(layout);

    dialog.setWindowTitle("Plugin Configuration - CP Joy To Twist");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(ui->label->text() != labeledit->text())
    {
        emit changeLabel(labeledit->text());
        settings->setValue(uuid.toString() + "/Label", labeledit->text());
    }

    if(linearedit->text().toFloat() != linear_scale)
    {
        linear_scale = linearedit->text().toFloat();
        nodelet_priv->set_linear_scale(linear_scale);
        settings->setValue(uuid.toString() + "/LinearScale", linear_scale);
    }

    if(angularedit->text().toFloat() != angular_scale)
    {
        angular_scale = angularedit->text().toFloat();
        nodelet_priv->set_angular_scale(angular_scale);
        settings->setValue(uuid.toString() + "/AngularScale", angular_scale);
    }
}

}
