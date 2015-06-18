#ifndef CPHEADING_H
#define CPHEADING_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>
#include <QtOpenGL/QGLWidget>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <GL/glu.h>

#include <boost/shared_ptr.hpp>

namespace Ui {
class CPHeadingPlugin;
}

namespace control_panel
{
class CPHeadingPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPHeadingPlugin();
    ~CPHeadingPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);
    void zoomChanged(double zoom);

signals:
    void changeYaw(double);
    void changeEnabled(bool);
    void setZoom(double zoom);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);  
 
private:
    void ImuCB(const sensor_msgs::Imu::ConstPtr &msg);

    Ui::CPHeadingPlugin *ui;
    QString topic;
    CPSSN *nodelet_priv;
};

class CPHeadingGL : public QGLWidget
{
    Q_OBJECT

public:
    CPHeadingGL(QWidget *_parent = NULL);
    ~CPHeadingGL();

signals:
    void zoomChanged(double zoom);

public slots:
    void setYaw(double);
    void setZoom(double zoom);
    void reloadTexture();

protected:
    // OpenGL Methods
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void paintCrosshair();
    void paintDial();
    void wheelEvent(QWheelEvent *event);
    void updateZoom();

    // Queries
    float getZoom();

private:
    // Widget Properties
    QWidget *parent;

    // Internal Properties
    double zoom_level;
    int w;
    int h;

    // Image
    int dial_dl;
    int crosshair_dl;
    int dial_tex;
    int crosshair_tex;
    int cover_tex;
    GLUquadricObj *stencil_quad;
    double yaw;
};
}

#endif // CPHEADING_H
