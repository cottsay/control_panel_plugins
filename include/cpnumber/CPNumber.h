#ifndef CPNUMBER_H
#define CPNUMBER_H

#include "cpssn/CPSSN.h"

#include <QContextMenuEvent>

#include <control_panel/ControlPanelPlugin.h>

#include <ros/ros.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/shared_ptr.hpp>

#include <QMenu>
#include <QGridLayout>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>

namespace Ui {
class CPNumberPlugin;
}

namespace control_panel
{

enum MessageFlags
{
    MessageNone = 0,
    MessageSigned = (1 << 0),
    MessageTypeByte = (1 << 1),
    MessageTypeInt = (1 << 2),
    MessageTypeFloat = (1 << 3),
    MessageSize8 = (1 << 4),
    MessageSize16 = (1 << 5),
    MessageSize32 = (1 << 6),
    MessageSize64 = (1 << 7),
    MessageRadix8 = (1 << 8),
    MessageRadix10 = (1 << 9),
    MessageRadix16 = (1 << 10),
    MessageRadixChar = (1 << 11)
};

class numberConfigDialog : public QDialog
{
    Q_OBJECT
    
public slots:
    void typeChangeCB(int idx);

public:
    numberConfigDialog(QString topic, bool labelIsVisible, QString labelText, enum MessageFlags _messageFlags);

    QGridLayout *layout;
    QLineEdit *topicedit;
    QCheckBox *slabelcheck;
    QLineEdit *labeledit;
    QComboBox *typebox;
    QComboBox *sizebox;
    QCheckBox *signedcheck;
    QComboBox *radixbox;

private:
    enum MessageFlags messageFlags;
};

class CPNumberPlugin : public ControlPanelPlugin
{
    Q_OBJECT

public:
    explicit CPNumberPlugin();
    ~CPNumberPlugin();
    void start();
    void stop();
    void setup();
    boost::shared_ptr<nodelet::Nodelet> getNodelet();

public slots:
    void configDialog();
    void setActive(bool active);

protected:
    void contextMenuEvent(QContextMenuEvent *event);
    void activateNodelet(bool passive = false);

private:
    void intRadixDisplay(long msg, int bits);
    void floatRadixDisplay(double msg);

    void byteCB(const std_msgs::ByteConstPtr &msg);
    void int8CB(const std_msgs::Int8ConstPtr &msg);
    void int16CB(const std_msgs::Int16ConstPtr &msg);
    void int32CB(const std_msgs::Int32ConstPtr &msg);
    void int64CB(const std_msgs::Int64ConstPtr &msg);
    void uint8CB(const std_msgs::UInt8ConstPtr &msg);
    void uint16CB(const std_msgs::UInt16ConstPtr &msg);
    void uint32CB(const std_msgs::UInt32ConstPtr &msg);
    void uint64CB(const std_msgs::UInt64ConstPtr &msg);
    void float32CB(const std_msgs::Float32ConstPtr &msg);
    void float64CB(const std_msgs::Float64ConstPtr &msg);

    Ui::CPNumberPlugin *ui;
    QString topic;
    enum MessageFlags messageFlags;
    CPSSN *nodelet_priv;
};
}

#endif // CPNUMBER_H
