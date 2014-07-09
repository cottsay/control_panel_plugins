#ifndef CPIMAGE_H
#define CPIMAGE_H

#include "cpssn/CPSSN.h"

#include <QtOpenGL/QGLWidget>
#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <boost/shared_ptr.hpp>

namespace Ui {
class CPImagePlugin;
}

namespace control_panel
{
class CPImagePlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPImagePlugin();
    ~CPImagePlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();
    static QImage rosImageToQt(const sensor_msgs::Image::ConstPtr &msg);

public slots:
    void configDialog();
    void setActive(bool active);
    void zoomChanged(double zoom);
    void centerChanged(double w, double h);

signals:
    void changeImage(const QImage &);
    void changeEnabled(bool);
    void setZoom(double zoom);
    void setCenter(double w, double h);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);  
 
private:
    void ImageCB(const sensor_msgs::Image::ConstPtr &msg);

    Ui::CPImagePlugin *ui;
    QString topic;
    CPSSN *nodelet_priv;
};

class CPImageGL : public QGLWidget
{
    Q_OBJECT

public:
    CPImageGL(QWidget *_parent = NULL);
    ~CPImageGL();

signals:
    void zoomChanged(double zoom);
    void centerChanged(double w, double h);

public slots:
    void setImage(const QImage &img);
    void setZoom(double zoom);
    void setCenter(double w, double h);
    void reloadTexture();

protected:
    // OpenGL Methods
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void paintImage();
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
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
    int drag_origin_w;
    int drag_origin_h;
    double window_center_w;
    double window_center_h;
    double drag_window_center_start_w;
    double drag_window_center_start_h;

    // Image
    int image_dl;
    int tex_id;
    int image_width;
    int image_height;
};
}

#endif // CPIMAGE_H
