#include "cpemptysrv/CPEmptySrv.h"
#include "ui_CPEmptySrv.h"
#include <pluginlib/class_list_macros.h>
#include <QMenu>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(control_panel::CPEmptySrvPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
CPEmptySrvPlugin::CPEmptySrvPlugin() :
    ui(new Ui::CPEmptySrvPlugin),
    topic("cp_empty_srv"),
    nodelet_priv(new CPEmptySrvNodelet())
{
    ui->setupUi(this);
    connect(this, SIGNAL(changeEnabled(bool)), ui->button, SLOT(setEnabled(bool)));
    emit changeEnabled(false);
    ui->button->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    ui->button->setText("CPEmptySrv");
    connect(ui->button, SIGNAL(clicked()), this, SLOT(call()));
}

CPEmptySrvPlugin::~CPEmptySrvPlugin()
{
    delete ui;
}

void CPEmptySrvPlugin::start()
{
    if(!ros::ok())
    {
        ROS_WARN_STREAM("Tried to start a nodelet that wasn't ready!");
        return;
    }
    settings->setValue(uuid.toString() + "/Active", true);
    activateNodelet();
    emit changeEnabled(true);
    ui->button->setAttribute(Qt::WA_TransparentForMouseEvents, false);
}

void CPEmptySrvPlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    emit changeEnabled(false);
    ui->button->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    nodelet_priv->deactivate();
}

void CPEmptySrvPlugin::setup()
{
    ui->button->setText(settings->value(uuid.toString() + "/Label", ui->button->text()).toString());
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPEmptySrvPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPEmptySrvPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPEmptySrvPlugin::call()
{
    nodelet_priv->call();
}

void CPEmptySrvPlugin::contextMenuEvent(QContextMenuEvent *event)
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

void CPEmptySrvPlugin::configDialog()
{
    QDialog dialog;
    QGridLayout *layout = new QGridLayout;

    QLabel *topictxt = new QLabel(tr("Topic Name:"));
    QLineEdit *topicedit = new QLineEdit(topic);
    layout->addWidget(topictxt, 0, 0);
    layout->addWidget(topicedit, 0, 1);

    QLabel *labeltxt = new QLabel(tr("Label:"));
    QLineEdit *labeledit = new QLineEdit(ui->button->text());
    layout->addWidget(labeltxt, 1, 0);
    layout->addWidget(labeledit, 1, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 2, 1);

    dialog.setLayout(layout);

    dialog.setWindowTitle("Plugin Configuration - Empty Service");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(topic != topicedit->text())
    {
        topic = topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        activateNodelet(true);
    }

    if(ui->button->text() != labeledit->text())
    {
        ui->button->setText(labeledit->text());
        settings->setValue(uuid.toString() + "/Label", labeledit->text());
    }
}
void CPEmptySrvPlugin::activateNodelet(bool passive)
{
    nodelet_priv->activate(topic.toStdString(), passive);
}
}
