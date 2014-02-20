#include "cpbool/CPBool.h"
#include "ui_CPBool.h"
#include <pluginlib/class_list_macros.h>
#include <QMenu>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>

PLUGINLIB_EXPORT_CLASS(control_panel::CPBoolPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
CPBoolPlugin::CPBoolPlugin() :
    ui(new Ui::CPBoolPlugin),
    topic("cp_bool"),
    nodelet_priv(new CPSSN())
{
    ui->setupUi(this);
    ui->label->setEnabled(false);
    ui->value->setEnabled(false);
    ui->label->setText("CPBool: ");
}

CPBoolPlugin::~CPBoolPlugin()
{
    delete ui;
}

void CPBoolPlugin::start()
{
    if(!ros::ok())
    {
        ROS_WARN_STREAM("Tried to start a nodelet that wasn't ready!");
        return;
    }
    settings->setValue(uuid.toString() + "/Active", true);
    activateNodelet();
    ui->label->setEnabled(true);
    ui->value->setEnabled(true);
}

void CPBoolPlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    ui->label->setEnabled(false);
    ui->value->setEnabled(false);
    nodelet_priv->deactivate();
}

void CPBoolPlugin::setup()
{
    ui->label->setText(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    ui->label->setVisible(settings->value(uuid.toString() + "/ShowLabel", true).toBool());
    ui->value->setAlignment((settings->value(uuid.toString() + "/ShowLabel", true).toBool() ? Qt::AlignLeft : Qt::AlignHCenter) | Qt::AlignVCenter);
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPBoolPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPBoolPlugin::boolCB(const std_msgs::BoolConstPtr &msg)
{
    ui->value->setText(msg->data ? "True" : "False");
}

void CPBoolPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPBoolPlugin::contextMenuEvent(QContextMenuEvent *event)
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

void CPBoolPlugin::configDialog()
{
    QDialog dialog;
    QGridLayout *layout = new QGridLayout;

    QLabel *topictxt = new QLabel(tr("Topic Name:"));
    QLineEdit *topicedit = new QLineEdit(topic);
    layout->addWidget(topictxt, 0, 0);
    layout->addWidget(topicedit, 0, 1);

    QLabel *slabeltxt = new QLabel(tr("Show Label:"));
    QCheckBox *slabelcheck = new QCheckBox();
    slabelcheck->setChecked(ui->label->isVisible());
    layout->addWidget(slabeltxt, 1, 0);
    layout->addWidget(slabelcheck, 1, 1);

    QLabel *labeltxt = new QLabel(tr("Label:"));
    QLineEdit *labeledit = new QLineEdit(ui->label->text());
    connect(slabelcheck, SIGNAL(toggled(bool)), labeledit, SLOT(setEnabled(bool)));
    labeledit->setEnabled(ui->label->isVisible());
    layout->addWidget(labeltxt, 2, 0);
    layout->addWidget(labeledit, 2, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 3, 1);

    dialog.setLayout(layout);

    dialog.setWindowTitle("Plugin Configuration - Bool");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(topic != topicedit->text())
    {
        ui->value->setText("N/A");
        topic = topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        activateNodelet(true);
    }

    if(ui->label->isVisible() != slabelcheck->isChecked())
    {
        ui->label->setVisible(slabelcheck->isChecked());
        ui->value->setAlignment((slabelcheck->isChecked() ? Qt::AlignLeft : Qt::AlignHCenter) | Qt::AlignVCenter);
        settings->setValue(uuid.toString() + "/ShowLabel", slabelcheck->isChecked());
    }

    if(ui->label->text() != labeledit->text())
    {
        ui->label->setText(labeledit->text());
        settings->setValue(uuid.toString() + "/Label", labeledit->text());
    }
}
void CPBoolPlugin::activateNodelet(bool passive)
{
    nodelet_priv->activate<const std_msgs::Bool_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPBoolPlugin::boolCB, this, _1), passive);
}
}
