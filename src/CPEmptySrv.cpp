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
    joyEn(false),
    joyTopic("joy"),
    joyIdx(0),
    nodelet_priv(new CPEmptySrvNodelet())
{
    ui->setupUi(this);
    connect(this, SIGNAL(changeEnabled(bool)), ui->button, SLOT(setEnabled(bool)));
    emit changeEnabled(false);
    ui->button->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    ui->button->setText("CPEmptySrv");
    connect(ui->button, SIGNAL(clicked()), this, SLOT(call()));
    connect(this, SIGNAL(pressButton()), ui->button, SLOT(click()));
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
    joyEn = settings->value(uuid.toString() + "/JoyEnabled", joyEn).toBool();
    joyTopic = settings->value(uuid.toString() + "/JoyTopic", joyTopic).toString();
    joyIdx = settings->value(uuid.toString() + "/JoyIndex", joyIdx).toInt();
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
    bool reactivateNodelet = false;
    CPEmptySrvCfg dialog(topic, ui->button->text(), joyEn, joyTopic, joyIdx);

    // Invalidate the joyIdx to keep from calling while detecting
    joyIdx = -1;

    connect(this, SIGNAL(joyDetect(int)), &dialog, SLOT(joyDetect(int)));
    connect(&dialog.joyidxbutton, SIGNAL(toggled(bool)), this, SLOT(setJoyEn(bool)));
    connect(&dialog, SIGNAL(updateJoyTopic(const QString &)), this, SLOT(setJoyTopic(const QString &)));

    if(!dialog.exec())
        return;

    if(topic != dialog.topicedit.text())
    {
        topic = dialog.topicedit.text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        reactivateNodelet = true;
    }

    if(ui->button->text() != dialog.labeledit.text())
    {
        ui->button->setText(dialog.labeledit.text());
        settings->setValue(uuid.toString() + "/Label", ui->button->text());
    }

    if(joyEn != dialog.joycheck.isChecked())
    {
        joyEn = dialog.joycheck.isChecked();
        settings->setValue(uuid.toString() + "/JoyEnabled", joyEn);
        reactivateNodelet = true;
    }

    if(joyTopic != dialog.joytopicedit.text())
    {
        joyTopic = dialog.joytopicedit.text();
        settings->setValue(uuid.toString() + "/JoyTopic", joyTopic);
        if(joyEn)
            reactivateNodelet = true;
    }

    if(joyIdx != dialog.joyidxedit.text().toInt())
    {
        joyIdx = dialog.joyidxedit.text().toInt();
        settings->setValue(uuid.toString() + "/JoyIndex", joyIdx);
    }

    if(reactivateNodelet)
        activateNodelet(true);
}
void CPEmptySrvPlugin::activateNodelet(bool passive)
{
    nodelet_priv->activate(topic.toStdString(), joyEn ? joyTopic.toStdString() : std::string(""), boost::bind(&CPEmptySrvPlugin::JoyCB, this, _1), passive);
}

void CPEmptySrvPlugin::JoyCB(const sensor_msgs::Joy::ConstPtr &msg)
{
    static bool lastState = false;

    if(0 <= joyIdx && msg->buttons.size() > joyIdx)
    {
        if(msg->buttons[joyIdx] && !lastState)
            emit pressButton();
        lastState = msg->buttons[joyIdx];
    }

    if(receivers(SIGNAL(joyDetect(int))) > 0)
    {
        for(unsigned int i = 0; i < msg->buttons.size(); i++)
            if(msg->buttons[i])
            {
                emit joyDetect(i);
                break;
            }
    }
}

void CPEmptySrvPlugin::setJoyTopic(const QString &topic)
{
    if(joyTopic != topic)
    {
        joyTopic = topic;
        settings->setValue(uuid.toString() + "/JoyTopic", joyTopic);
    }

    if(joyEn != true)
    {
        joyEn = true;
        settings->setValue(uuid.toString() + "/JoyEnabled", joyEn);
    }

    activateNodelet(true);
}

CPEmptySrvCfg::CPEmptySrvCfg(const QString &topic, const QString &label, const bool joyEn, const QString &joyTopic, unsigned int joyIdx) :
    topictxt(tr("Topic Name:")),
    topicedit(topic),
    labeltxt(tr("Label:")),
    labeledit(label),
    joytxt(tr("Joy Binding:")),
    joycheck(),
    joytopictxt(tr("Joy Topic:")),
    joytopicedit(joyTopic),
    joyidxtxt(tr("Joy Button:")),
    joyidxedit(QString::number(joyIdx)),
    joyidxbutton(tr("Auto")),
    okbutton(tr("&OK")),
    detecting(false)
{
    joyChange(joyEn);
    joycheck.setChecked(joyEn);

    layout.addWidget(&topictxt, 0, 0);
    layout.addWidget(&topicedit, 0, 1);

    layout.addWidget(&labeltxt, 1, 0);
    layout.addWidget(&labeledit, 1, 1);

    layout.addWidget(&joytxt, 2, 0);
    layout.addWidget(&joycheck, 2, 1);

    layout.addWidget(&joytopictxt, 3, 0);
    layout.addWidget(&joytopicedit, 3, 1);

    layout.addWidget(&joyidxtxt, 4, 0);
    joyidxlayout.addWidget(&joyidxedit, 0, 0);
    joyidxlayout.addWidget(&joyidxbutton, 0, 1);
    layout.addLayout(&joyidxlayout, 4, 1);

    layout.addWidget(&okbutton, 5, 1);

    connect(&okbutton, SIGNAL(clicked()), this, SLOT(accept()));
    connect(&joycheck, SIGNAL(toggled(bool)), this, SLOT(joyChange(bool)));
    connect(&joyidxbutton, SIGNAL(clicked()), this, SLOT(joyAuto()));

    this->setLayout(&layout);
    this->setWindowTitle("Plugin Configuration - Empty Service");
}

void CPEmptySrvCfg::joyChange(bool en)
{
    joytopictxt.setEnabled(en);
    joytopicedit.setEnabled(en);

    joyidxtxt.setEnabled(en);
    joyidxedit.setEnabled(en);
    joyidxbutton.setText(tr("Auto"));
    joyidxbutton.setEnabled(en);
}

void CPEmptySrvCfg::joyAuto()
{
    if(detecting)
    {
        joyidxbutton.setText(tr("Auto"));
    }
    else
    {
        joyidxbutton.setText(tr("Listening..."));
        emit updateJoyTopic(joytopicedit.text());
    }
    detecting = !detecting;
}

void CPEmptySrvCfg::joyDetect(int button)
{
    if(detecting)
    {
        joyidxedit.setText(QString::number(button));
        joyidxbutton.setText(tr("Auto"));
        detecting = false;
    }
}

}
