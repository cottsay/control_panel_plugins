#include "cpattitude/CPAttitude.h"
#include "ui_CPAttitude.h"

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include <QCheckBox>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPushButton>

PLUGINLIB_EXPORT_CLASS(control_panel::CPAttitudePlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
CPAttitudePlugin::CPAttitudePlugin() :
    ui(new Ui::CPAttitudePlugin),
    topic("cp_imu"),
    nodelet_priv(new CPSSN())
{
    ui->setupUi(this);
    connect(this, SIGNAL(changeRoll(double)), ui->widget, SLOT(setRoll(double)));
    connect(this, SIGNAL(changePitch(double)), ui->widget, SLOT(setPitch(double)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->widget, SLOT(setEnabled(bool)));
    connect(ui->widget, SIGNAL(zoomChanged(double)), this, SLOT(zoomChanged(double)));
    connect(this, SIGNAL(setZoom(double)), ui->widget, SLOT(setZoom(double)));
    emit changeEnabled(false);
}

CPAttitudePlugin::~CPAttitudePlugin()
{
    delete ui;
}

void CPAttitudePlugin::start()
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

void CPAttitudePlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    emit changeEnabled(false);
    nodelet_priv->deactivate();
}

void CPAttitudePlugin::setup()
{
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    if(settings->contains(uuid.toString() + "/IndicatorZoom"))
        emit setZoom(settings->value(uuid.toString() + "/IndicatorZoom").toDouble());

    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPAttitudePlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPAttitudePlugin::ImuCB(const sensor_msgs::Imu::ConstPtr &msg)
{
    const double x = msg->orientation.x;
    const double y = msg->orientation.y;
    const double z = msg->orientation.z;
    const double w = msg->orientation.w;

    emit changeRoll(atan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z));
    emit changePitch(asin(-2.0 * (x * z - w * y)));
}

void CPAttitudePlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPAttitudePlugin::zoomChanged(const double zoom)
{
    settings->setValue(uuid.toString() + "/IndicatorZoom", zoom);
}

void CPAttitudePlugin::contextMenuEvent(QContextMenuEvent *event)
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

void CPAttitudePlugin::configDialog()
{
    QDialog dialog;
    QGridLayout *layout = new QGridLayout;

    QLabel *topictxt = new QLabel(tr("Topic Name:"));
    QLineEdit *topicedit = new QLineEdit(topic);
    layout->addWidget(topictxt, 0, 0);
    layout->addWidget(topicedit, 0, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 1, 1);

    dialog.setLayout(layout);

    dialog.setWindowTitle("Plugin Configuration - Attitude");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(topic != topicedit->text())
    {
        emit changeRoll(0);
        emit changePitch(0);
        topic = topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        activateNodelet(true);
    }
}
void CPAttitudePlugin::activateNodelet(bool passive)
{
    nodelet_priv->activate<const sensor_msgs::Imu_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPAttitudePlugin::ImuCB, this, _1), passive);
}

CPAttitudeGL::CPAttitudeGL(QWidget *_parent)
    : QGLWidget(_parent, &(ControlPanelPlugin::getGlobalGLWidget())),
      zoom_level(1.0),
      w(100),
      h(100),
      horizon_dl(0),
      crosshair_dl(0),
      horizon_tex(0),
      crosshair_tex(0),
      cover_tex(0),
      stencil_quad(NULL),
      roll(0),
      pitch(0)
{
}

CPAttitudeGL::~CPAttitudeGL()
{
    if(horizon_dl)
      glDeleteLists(horizon_dl, 1);
    if(crosshair_dl)
      glDeleteLists(crosshair_dl, 1);
}

void CPAttitudeGL::setRoll(double r)
{
    roll = r * 180 / M_PI;

    update();
}

void CPAttitudeGL::setPitch(double p)
{
    pitch = p * 180 / M_PI;

    update();
}

void CPAttitudeGL::setZoom(const double zoom)
{
    zoom_level = zoom;
    updateZoom();
    update();
    emit zoomChanged(zoom_level);
}

void CPAttitudeGL::initializeGL()
{
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    qglClearColor(QWidget::palette().color(QWidget::backgroundRole()));

    horizon_tex = bindTexture(QPixmap(":/images/artificial_horizon.png"));
    horizon_dl = glGenLists(1);
    paintHorizon();

    crosshair_tex = bindTexture(QPixmap(":/images/crosshair.png"));
    cover_tex = bindTexture(QPixmap(":/images/cover.png"));
    crosshair_dl = glGenLists(1);
    stencil_quad = gluNewQuadric();
    paintCrosshair();
}

void CPAttitudeGL::resizeGL(int _w, int _h)
{
    w = _w;
    h = _h;
    updateZoom();
}

void CPAttitudeGL::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glRotated(roll, 0.0, 0.0, 1.0);
    glTranslated(0.0, pitch, 0.0);
    glCallList(horizon_dl);
    glLoadIdentity();
    glCallList(crosshair_dl);
}

void CPAttitudeGL::paintHorizon()
{
    glNewList(horizon_dl, GL_COMPILE);
    glColor3f(1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, horizon_tex);
    glBegin(GL_POLYGON);
    glTexCoord2f(0, 0);
    glVertex2f(-100, -150);
    glTexCoord2f(1, 0);
    glVertex2f(100, -150);
    glTexCoord2f(1, 1);
    glVertex2f(100, 150);
    glTexCoord2f(0, 1);
    glVertex2f(-100, 150);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glEndList();
}

void CPAttitudeGL::paintCrosshair()
{
    glNewList(crosshair_dl, GL_COMPILE);
    glColor3f(1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, crosshair_tex);
    glBegin(GL_POLYGON);
    glTexCoord2f(0, 0);
    glVertex2f(-50, -50);
    glTexCoord2f(1, 0);
    glVertex2f(50, -50);
    glTexCoord2f(1, 1);
    glVertex2f(50, 50);
    glTexCoord2f(0, 1);
    glVertex2f(-50, 50);
    glEnd();
    glBindTexture(GL_TEXTURE_2D, cover_tex);
    glBegin(GL_POLYGON);
    glTexCoord2f(0, 0);
    glVertex2f(-50, -50);
    glTexCoord2f(1, 0);
    glVertex2f(50, -50);
    glTexCoord2f(1, 1);
    glVertex2f(50, 50);
    glTexCoord2f(0, 1);
    glVertex2f(-50, 50);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    qglColor(QWidget::palette().color(QWidget::backgroundRole()));
    gluDisk(stencil_quad, 49, 1000, 40, 8);
    glEndList();
}

void CPAttitudeGL::wheelEvent(QWheelEvent *event)
{
    if(event->orientation() == Qt::Vertical)
    {
        zoom_level -= event->delta() / 5000.0;
        if(zoom_level < .01)
            zoom_level = .01;
        updateZoom();
        update();
        event->accept();
        emit zoomChanged(zoom_level);
    }
}

void CPAttitudeGL::updateZoom()
{
    const double w_2 = w / 2.0 * zoom_level;
    const double h_2 = h / 2.0 * zoom_level;
    makeCurrent();
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-w_2, w_2, -h_2, h_2);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

float CPAttitudeGL::getZoom()
{
    return zoom_level;
}

}

