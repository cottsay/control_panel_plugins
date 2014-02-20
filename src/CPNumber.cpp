#include "cpnumber/CPNumber.h"
#include "ui_CPNumber.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(control_panel::CPNumberPlugin, control_panel::ControlPanelPlugin)

namespace control_panel
{

numberConfigDialog::numberConfigDialog(QString topic, bool labelIsVisible, QString labelText, enum MessageFlags _messageFlags) : 
    layout(new QGridLayout),
    topicedit(new QLineEdit(topic)),
    slabelcheck(new QCheckBox),
    labeledit(new QLineEdit(labelText)),
    typebox(new QComboBox),
    sizebox(new QComboBox),
    signedcheck(new QCheckBox),
    radixbox(new QComboBox)
{
    messageFlags = _messageFlags;

    QLabel *topictxt = new QLabel(tr("Topic Name:"));
    layout->addWidget(topictxt, 0, 0);
    layout->addWidget(topicedit, 0, 1);

    QLabel *slabeltxt = new QLabel(tr("Show Label:"));
    slabelcheck->setChecked(labelIsVisible);
    layout->addWidget(slabeltxt, 1, 0);
    layout->addWidget(slabelcheck, 1, 1);

    QLabel *labeltxt = new QLabel(tr("Label:"));
    labeledit->setEnabled(labelIsVisible);
    layout->addWidget(labeltxt, 2, 0);
    layout->addWidget(labeledit, 2, 1);

    QLabel *typetxt = new QLabel(tr("Type:"));
    typebox->addItem(tr("Byte"), (int)MessageTypeByte);
    if(messageFlags & MessageTypeByte)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("Int"), (int)MessageTypeInt);
    if(messageFlags & MessageTypeInt)
        typebox->setCurrentIndex(typebox->count() - 1);
    typebox->addItem(tr("Float"), (int)MessageTypeFloat);
    if(messageFlags & MessageTypeFloat)
        typebox->setCurrentIndex(typebox->count() - 1);
    layout->addWidget(typetxt, 3, 0);
    layout->addWidget(typebox, 3, 1);

    QLabel *sizetxt = new QLabel(tr("Size:"));
    sizebox->addItem(tr("8 bits"), (int)MessageSize8);
    if(messageFlags & MessageSize8)
    sizebox->setCurrentIndex(sizebox->count() - 1);
    sizebox->addItem(tr("16 bits"), (int)MessageSize16);
    if(messageFlags & MessageSize16)
        sizebox->setCurrentIndex(sizebox->count() - 1);
    sizebox->addItem(tr("32 bits"), (int)MessageSize32);
    if(messageFlags & MessageSize32)
        sizebox->setCurrentIndex(sizebox->count() - 1);
    sizebox->addItem(tr("64 bits"), (int)MessageSize64);
    if(messageFlags & MessageSize64)
        sizebox->setCurrentIndex(sizebox->count() - 1);
    layout->addWidget(sizetxt, 4, 0);
    layout->addWidget(sizebox, 4, 1);

    QLabel *signedtxt = new QLabel(tr("Signed:"));
    if(messageFlags & MessageSigned)
        signedcheck->setChecked(true);
    else
        signedcheck->setChecked(false);
    layout->addWidget(signedtxt, 5, 0);
    layout->addWidget(signedcheck, 5, 1);

    QLabel *radixtxt = new QLabel(tr("Display radix:"));
    radixbox->addItem(tr("Octal"), (int)MessageRadix8);
    if(messageFlags & MessageRadix8)
        radixbox->setCurrentIndex(radixbox->count() - 1);
    radixbox->addItem(tr("Decimal"), (int)MessageRadix10);
    if(messageFlags & MessageRadix10)
        radixbox->setCurrentIndex(radixbox->count() - 1);
    radixbox->addItem(tr("Hex"), (int)MessageRadix16);
    if(messageFlags & MessageRadix16)
        radixbox->setCurrentIndex(radixbox->count() - 1);
    layout->addWidget(radixtxt, 6, 0);
    layout->addWidget(radixbox, 6, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 7, 1);

    this->setLayout(layout);

    this->setWindowTitle("Plugin Configuration - Number");
    
    connect(slabelcheck, SIGNAL(toggled(bool)), labeledit, SLOT(setEnabled(bool)));
    connect(typebox, SIGNAL(currentIndexChanged(int)), this, SLOT(typeChangeCB(int)));
    connect(okbutton, SIGNAL(clicked()), this, SLOT(accept()));

    numberConfigDialog::typeChangeCB(typebox->currentIndex());
}

void numberConfigDialog::typeChangeCB(int idx)
{
    sizebox->clear();

    radixbox->clear();

    switch(idx)
    {
    case 0:
        signedcheck->setEnabled(false);

        sizebox->addItem(tr("8 bits"), (int)MessageSize8);

        radixbox->addItem(tr("Octal"), (int)MessageRadix8);
        if(messageFlags & MessageRadix8)
            radixbox->setCurrentIndex(radixbox->count() - 1);
        radixbox->addItem(tr("Decimal"), (int)MessageRadix10);
        if(messageFlags & MessageRadix10)
            radixbox->setCurrentIndex(radixbox->count() - 1);
        radixbox->addItem(tr("Hex"), (int)MessageRadix16);
        if(messageFlags & MessageRadix16)
            radixbox->setCurrentIndex(radixbox->count() - 1);
        radixbox->addItem(tr("Char"), (int)MessageRadixChar);
        if(messageFlags & MessageRadixChar)
            radixbox->setCurrentIndex(radixbox->count() - 1);
        break;
    case 1:
        signedcheck->setEnabled(true);

        sizebox->addItem(tr("8 bits"), (int)MessageSize8);
        if(messageFlags & MessageSize8)
            sizebox->setCurrentIndex(sizebox->count() - 1);
        sizebox->addItem(tr("16 bits"), (int)MessageSize16);
        if(messageFlags & MessageSize16)
            sizebox->setCurrentIndex(sizebox->count() - 1);
        sizebox->addItem(tr("32 bits"), (int)MessageSize32);
        if(messageFlags & MessageSize32)
            sizebox->setCurrentIndex(sizebox->count() - 1);
        sizebox->addItem(tr("64 bits"), (int)MessageSize64);
        if(messageFlags & MessageSize64)
            sizebox->setCurrentIndex(sizebox->count() - 1);

        radixbox->addItem(tr("Octal"), (int)MessageRadix8);
        if(messageFlags & MessageRadix8)
            radixbox->setCurrentIndex(radixbox->count() - 1);
        radixbox->addItem(tr("Decimal"), (int)MessageRadix10);
        if(messageFlags & MessageRadix10)
            radixbox->setCurrentIndex(radixbox->count() - 1);
        radixbox->addItem(tr("Hex"), (int)MessageRadix16);
        if(messageFlags & MessageRadix16)
            radixbox->setCurrentIndex(radixbox->count() - 1);
    break;
    case 2:
        signedcheck->setEnabled(false);

        sizebox->addItem(tr("32 bits"), (int)MessageSize32);
        if(messageFlags & MessageSize32)
            sizebox->setCurrentIndex(sizebox->count() - 1);
        sizebox->addItem(tr("64 bits"), (int)MessageSize64);
        if(messageFlags & MessageSize64)
            sizebox->setCurrentIndex(sizebox->count() - 1);

        radixbox->addItem(tr("Decimal"), (int)MessageRadix10);
        if(messageFlags & MessageRadix10)
            radixbox->setCurrentIndex(radixbox->count() - 1);
        break;
    }
}

CPNumberPlugin::CPNumberPlugin() :
    ui(new Ui::CPNumberPlugin),
    topic("cp_number"),
    messageFlags((enum MessageFlags)(MessageSize32 | MessageTypeFloat)),
    nodelet_priv(new CPSSN())
{
    ui->setupUi(this);
    ui->label->setEnabled(false);
    ui->value->setEnabled(false);
    ui->label->setText("CPNumber: ");
}

CPNumberPlugin::~CPNumberPlugin()
{
    delete ui;
}

void CPNumberPlugin::start()
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

void CPNumberPlugin::stop()
{
    settings->setValue(uuid.toString() + "/Active", false);
    ui->label->setEnabled(false);
    ui->value->setEnabled(false);
    nodelet_priv->deactivate();
}

void CPNumberPlugin::setup()
{
    ui->label->setText(settings->value(uuid.toString() + "/Label", ui->label->text()).toString());
    ui->label->setVisible(settings->value(uuid.toString() + "/ShowLabel", true).toBool());
    ui->value->setAlignment((settings->value(uuid.toString() + "/ShowLabel", true).toBool() ? Qt::AlignLeft : Qt::AlignHCenter) | Qt::AlignVCenter);
    topic = settings->value(uuid.toString() + "/Topic", topic).toString();
    messageFlags = (enum MessageFlags)settings->value(uuid.toString() + "/Type", (int)messageFlags).toInt();
    if(settings->value(uuid.toString() + "/Active", false).toBool())
        start();
}

boost::shared_ptr<nodelet::Nodelet> CPNumberPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>(nodelet_priv);
}

void CPNumberPlugin::intRadixDisplay(long msg, int bits)
{
    QString str;
    if(messageFlags & MessageRadix10)
        str.sprintf("%d", msg);
    else if(messageFlags & MessageRadix8)
        str.sprintf("0%.*o", (int)(bits/3.0+.8), msg);
    else if(messageFlags & MessageRadix16)
        str.sprintf("0x%.*X", (int)(bits/4.0+.8), msg);
    else if(messageFlags & MessageRadixChar)
        str.sprintf("%c", msg);
    ui->value->setText(str);
}

void CPNumberPlugin::floatRadixDisplay(double msg)
{
    if(messageFlags & MessageRadix10)
        ui->value->setText(QString::number(msg));
// un-comment when ubuntu is less bad, add int bits argument
//    else if(messageFlags & MessageRadix16)
//        ui->value->QString::sprintf("0x.*A", (int)(bits/4.0+.8), msg);
}

void CPNumberPlugin::byteCB(const std_msgs::ByteConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 8);
}

void CPNumberPlugin::int8CB(const std_msgs::Int8ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 8);
}

void CPNumberPlugin::int16CB(const std_msgs::Int16ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 16);
}

void CPNumberPlugin::int32CB(const std_msgs::Int32ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 32);
}

void CPNumberPlugin::int64CB(const std_msgs::Int64ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 64);
}

void CPNumberPlugin::uint8CB(const std_msgs::UInt8ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 8);
}

void CPNumberPlugin::uint16CB(const std_msgs::UInt16ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 16);
}

void CPNumberPlugin::uint32CB(const std_msgs::UInt32ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 32);
}

void CPNumberPlugin::uint64CB(const std_msgs::UInt64ConstPtr &msg)
{
    CPNumberPlugin::intRadixDisplay((long) msg->data, 64);
}

void CPNumberPlugin::float32CB(const std_msgs::Float32ConstPtr &msg)
{
    CPNumberPlugin::floatRadixDisplay((double) msg->data);
}

void CPNumberPlugin::float64CB(const std_msgs::Float64ConstPtr &msg)
{
    CPNumberPlugin::floatRadixDisplay((double) msg->data);
}

void CPNumberPlugin::setActive(bool active)
{
    if(active)
        start();
    else
        stop();
}

void CPNumberPlugin::contextMenuEvent(QContextMenuEvent *event)
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

void CPNumberPlugin::configDialog()
{
    bool reactivate = false;
    enum MessageFlags newMessageFlags;

    numberConfigDialog *dialog = new numberConfigDialog(topic, ui->label->isVisible(), ui->label->text(), messageFlags);


    if(!dialog->exec())
        return;

    if(topic != dialog->topicedit->text())
    {
        topic = dialog->topicedit->text();
        settings->setValue(uuid.toString() + "/Topic", topic);
        reactivate = true;
    }

    newMessageFlags = (enum MessageFlags)(dialog->sizebox->itemData(dialog->sizebox->currentIndex()).toInt() | dialog->typebox->itemData(dialog->typebox->currentIndex()).toInt() | (dialog->signedcheck->isChecked() ? MessageSigned : MessageNone ));

    if(newMessageFlags != messageFlags & ~(MessageRadix8 | MessageRadix10 | MessageRadix16 | MessageRadixChar))
        reactivate = true;

    messageFlags = (enum MessageFlags)(newMessageFlags | dialog->radixbox->itemData(dialog->radixbox->currentIndex()).toInt()); 
    settings->setValue(uuid.toString() + "/Type", (int)messageFlags);

    if (reactivate)
    {
        ui->value->setText("N/A");
        activateNodelet(true);
    }

    if(ui->label->isVisible() != dialog->slabelcheck->isChecked())
    {
        ui->label->setVisible(dialog->slabelcheck->isChecked());
        ui->value->setAlignment((dialog->slabelcheck->isChecked() ? Qt::AlignLeft : Qt::AlignHCenter) | Qt::AlignVCenter);
        settings->setValue(uuid.toString() + "/ShowLabel", dialog->slabelcheck->isChecked());
    }

    if(ui->label->text() != dialog->labeledit->text())
    {
        ui->label->setText(dialog->labeledit->text());
        settings->setValue(uuid.toString() + "/Label", dialog->labeledit->text());
    }
}

void CPNumberPlugin::activateNodelet(bool passive)
{
    if(messageFlags & MessageTypeByte)
            nodelet_priv->activate<const std_msgs::Byte_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::byteCB, this, _1), passive);
    else if(messageFlags & MessageTypeFloat)
    {
        if(messageFlags & MessageSize32)
            nodelet_priv->activate<const std_msgs::Float32_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::float32CB, this, _1), passive);
        else
            nodelet_priv->activate<const std_msgs::Float64_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::float64CB, this, _1), passive);
    }
    else if(messageFlags & MessageTypeInt)
    {
        if(messageFlags & MessageSigned)
        {
            if(messageFlags & MessageSize8)
                nodelet_priv->activate<const std_msgs::Int8_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::int8CB, this, _1), passive);
            else if(messageFlags & MessageSize16)
                nodelet_priv->activate<const std_msgs::Int16_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::int16CB, this, _1), passive);
            else if(messageFlags & MessageSize32)
                nodelet_priv->activate<const std_msgs::Int32_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::int32CB, this, _1), passive);
            else
                nodelet_priv->activate<const std_msgs::Int64_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::int64CB, this, _1), passive);
        }
        else
        {
            if(messageFlags & MessageSize8)
                nodelet_priv->activate<const std_msgs::UInt8_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::uint8CB, this, _1), passive);
            else if(messageFlags & MessageSize16)
                nodelet_priv->activate<const std_msgs::UInt16_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::uint16CB, this, _1), passive);
            else if(messageFlags & MessageSize32)
                nodelet_priv->activate<const std_msgs::UInt32_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::uint32CB, this, _1), passive);
            else
                nodelet_priv->activate<const std_msgs::UInt64_<std::allocator<void> > >(topic.toStdString(), boost::bind(&CPNumberPlugin::uint64CB, this, _1), passive);
        }
    }
}
}
