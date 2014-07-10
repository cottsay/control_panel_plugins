#include "cpimage/CPImage.h"
#include "ui_CPImage.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <QMenu>
#include <QDialog>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <GL/glu.h>

PLUGINLIB_EXPORT_CLASS(control_panel::CPImagePlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{
CPImagePlugin::CPImagePlugin() :
    ui(new Ui::CPImagePlugin),
    topic("cp_image"),
    nodelet_priv(new CPSSN())
{
    ui->setupUi(this);
    connect(this, SIGNAL(changeImage(const QImage &)), ui->widget, SLOT(setImage(const QImage &)));
    connect(this, SIGNAL(changeEnabled(bool)), ui->widget, SLOT(setEnabled(bool)));
    connect(ui->widget, SIGNAL(zoomChanged(double)), this, SLOT(zoomChanged(double)));
    connect(ui->widget, SIGNAL(centerChanged(double, double)), this, SLOT(centerChanged(double, double)));
    connect(this, SIGNAL(setZoom(double)), ui->widget, SLOT(setZoom(double)));
    connect(this, SIGNAL(setCenter(double, double)), ui->widget, SLOT(setCenter(double, double)));
    emit changeEnabled(false);
}

CPImagePlugin::~CPImagePlugin()
{
    delete ui;
}

void CPImagePlugin::start()
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

void CPImagePlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    emit changeEnabled(false);
    nodelet_priv->deactivate();
}

void CPImagePlugin::setup()
{
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    if(settings->contains(uuid.toString() + "/ImageZoom"))
        emit setZoom(settings->value(uuid.toString() + "/ImageZoom").toDouble());
    if(settings->contains(uuid.toString() + "/ImageCenterX") && settings->contains(uuid.toString() + "/ImageCenterY"))
        emit setCenter(settings->value(uuid.toString() + "/ImageCenterX").toDouble(), settings->value(uuid.toString() + "/ImageCenterY").toDouble());

    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPImagePlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPImagePlugin::ImageCB(const sensor_msgs::Image::ConstPtr &msg)
{
    emit changeImage(rosImageToQt(msg));
}

void CPImagePlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPImagePlugin::zoomChanged(const double zoom)
{
    settings->setValue(uuid.toString() + "/ImageZoom", zoom);
}

void CPImagePlugin::centerChanged(const double w, const double h)
{
    settings->setValue(uuid.toString() + "/ImageCenterX", w);
    settings->setValue(uuid.toString() + "/ImageCenterY", h);
}

void CPImagePlugin::contextMenuEvent(QContextMenuEvent *event)
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

void CPImagePlugin::configDialog()
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

    dialog.setWindowTitle("Plugin Configuration - Image");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(topic != topicedit->text())
    {
        emit changeImage(QImage());
        topic = topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        activateNodelet(true);
    }
}
void CPImagePlugin::activateNodelet(bool passive)
{
    nodelet_priv->activate<const sensor_msgs::Image_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPImagePlugin::ImageCB, this, _1), passive);
}

QImage CPImagePlugin::rosImageToQt(const sensor_msgs::Image::ConstPtr &msg)
{
    QImage::Format imgFormat = QImage::Format_Invalid;

    // Determine Encoding
    if(msg->encoding == sensor_msgs::image_encodings::RGB8 || msg->encoding == sensor_msgs::image_encodings::BGR8)
        imgFormat = QImage::Format_RGB888;
    else if(msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
        imgFormat = QImage::Format_Indexed8;
    else if(msg->encoding == sensor_msgs::image_encodings::RGBA8 || msg->encoding == sensor_msgs::image_encodings::BGRA8)
        imgFormat = QImage::Format_ARGB32;

    QImage img(&msg->data[0], msg->width, msg->height, msg->step, imgFormat);

    // Set up color table
    if(img.format() == QImage::Format_Indexed8)
         for(int i = 0; i < 256; i++)
             img.setColor(i, qRgb(i,i,i));

    // Perform R/B Flip and return
    if(msg->encoding == sensor_msgs::image_encodings::BGR8 || msg->encoding == sensor_msgs::image_encodings::BGRA8)
        return img.rgbSwapped().convertToFormat(QImage::Format_RGB32);
    else
        return img.convertToFormat(QImage::Format_RGB32);
}

CPImageGL::CPImageGL(QWidget *_parent)
    : QGLWidget(_parent, &(ControlPanelPlugin::getGlobalGLWidget())),
      zoom_level(1.0),
      w(100),
      h(100),
      drag_origin_w(-1),
      drag_origin_h(-1),
      window_center_w(0),
      window_center_h(0),
      drag_window_center_start_w(-1),
      drag_window_center_start_h(-1),
      image_dl(0),
      image_width(0),
      image_height(0)
{
}

CPImageGL::~CPImageGL()
{
    if(image_dl)
      glDeleteLists(image_dl, 1);
}

void CPImageGL::setImage(const QImage &img)
{
    if(!isValid())
        return;

    if(img.width() != image_width || img.height() != image_height)
    {
        image_width = img.width();
        image_height = img.height();
        paintImage();
    }

    deleteTexture(tex_id);
    tex_id = bindTexture(img);

    update();
}

void CPImageGL::setZoom(const double zoom)
{
    zoom_level = zoom;
    updateZoom();
    update();
    emit zoomChanged(zoom_level);
}

void CPImageGL::setCenter(const double w, const double h)
{
    window_center_w = w;
    window_center_h = h;
    updateZoom();
    update();
    emit centerChanged(window_center_w, window_center_h);
}

void CPImageGL::initializeGL()
{
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    qglClearColor(QWidget::palette().color(QWidget::backgroundRole()));
    //glClearColor(0, 0, 0, 0);

    image_dl = glGenLists(1);
    paintImage();
}

void CPImageGL::resizeGL(int _w, int _h)
{
    w = _w;
    h = _h;
    updateZoom();
}

void CPImageGL::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glBindTexture(GL_TEXTURE_2D, tex_id);
    glCallList(image_dl);
}

void CPImageGL::paintImage()
{
    glNewList(image_dl, GL_COMPILE);
    glColor3f(1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_POLYGON);
    glTexCoord2f(0, 0);
    glVertex2f(-image_width, -image_height);
    glTexCoord2f(1, 0);
    glVertex2f(image_width, -image_height);
    glTexCoord2f(1, 1);
    glVertex2f(image_width, image_height);
    glTexCoord2f(0, 1);
    glVertex2f(-image_width, image_height);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glEndList();
}

void CPImageGL::mousePressEvent(QMouseEvent *event)
{
    if(event->button() & Qt::LeftButton)
    {
        drag_origin_w = event->x();
        drag_origin_h = event->y();
        drag_window_center_start_w = window_center_w;
        drag_window_center_start_h = window_center_h;
        event->accept();
    }
}

void CPImageGL::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button() & Qt::LeftButton)
    {
        drag_origin_w = drag_origin_h = -1;
        event->accept();
        emit centerChanged(window_center_w, window_center_h);
    }
}

void CPImageGL::mouseMoveEvent(QMouseEvent *event)
{
    if(drag_origin_w >= 0 && drag_origin_h >= 0)
    {
        window_center_w = drag_window_center_start_w + zoom_level * (drag_origin_w - event->x());
        window_center_h = drag_window_center_start_h - zoom_level * (drag_origin_h - event->y());
        updateZoom();
        update();
        event->accept();
    }
}

void CPImageGL::wheelEvent(QWheelEvent *event)
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

void CPImageGL::keyPressEvent(QKeyEvent* event)
{
    switch(event->key()) {
    case Qt::Key_Escape:
        close();
        break;
    default:
        event->ignore();
        break;
    }
}

void CPImageGL::updateZoom()
{
    const double w_2 = w / 2.0 * zoom_level;
    const double h_2 = h / 2.0 * zoom_level;
    makeCurrent();
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(window_center_w - w_2, window_center_w + w_2, window_center_h - h_2, window_center_h + h_2);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

float CPImageGL::getZoom()
{
    return zoom_level;
}

}

