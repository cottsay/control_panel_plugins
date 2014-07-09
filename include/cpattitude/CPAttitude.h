#ifndef CPATTITUDE_H
#define CPATTITUDE_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>
#include <QtOpenGL/QGLWidget>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <GL/glu.h>

#include <boost/shared_ptr.hpp>

namespace Ui {
class CPAttitudePlugin;
}

namespace control_panel
{
class CPAttitudePlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPAttitudePlugin();
    ~CPAttitudePlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);
    void zoomChanged(double zoom);

signals:
    void changeRoll(double);
    void changePitch(double);
    void changeEnabled(bool);
    void setZoom(double zoom);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);  
 
private:
    void ImuCB(const sensor_msgs::Imu::ConstPtr &msg);

    Ui::CPAttitudePlugin *ui;
    QString topic;
    CPSSN *nodelet_priv;
};

class CPAttitudeGL : public QGLWidget
{
    Q_OBJECT

public:
    CPAttitudeGL(QWidget *_parent = NULL);
    ~CPAttitudeGL();

signals:
    void zoomChanged(double zoom);

public slots:
    void setRoll(double);
    void setPitch(double);
    void setZoom(double zoom);
    void reloadTexture();

protected:
    // OpenGL Methods
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void paintCrosshair();
    void paintHorizon();
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
    int horizon_dl;
    int crosshair_dl;
    int horizon_tex;
    int crosshair_tex;
    int cover_tex;
    GLUquadricObj *stencil_quad;
    double roll;
    double pitch;
};
}

#endif // CPATTITUDE_H
