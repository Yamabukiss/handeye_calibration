#include "sensor.h"

Sensor::Sensor(QObject *parent)
    : QObject{parent}, fscale_(0), device_id_(0)
{
    call_one_times_ptr_ = new CallOneTimes();
    call_one_times_ptr_->device_id_ = device_id_;
}

void Sensor::getBatchNum(int max_batch)
{
    int batch_count = 0;
    while (batch_count < max_batch)
    {
        batch_count = SR7IF_ProfilePointCount(device_id_, NULL);
        emit postBatchNum(batch_count);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

int Sensor::getEncoderParameters()
{
    int type = 0x10;
    int target[4] = {0};
    int num_byte = 1;

    std::vector<unsigned char> vget_data;
    vget_data.resize(num_byte);

    int ret = SR7IF_GetSetting(device_id_, type, 0, 0x01, target,
                               &vget_data.at(0), num_byte);
    if(ret < 0)
        return -1;

    else
    {
        QString strInfo = QString("");
        for(int i = num_byte - 1; i >= 0; i --)
        {
            strInfo += QString("%1").arg(vget_data.at(i),
                                         2, 16, QLatin1Char('0'));
        }

        bool bOk = false;
        int mData = strInfo.toInt(&bOk, 16);

        return mData;
    }
}

bool Sensor::setEncoderParameters()
{
    int data = 0x02;
    int depth = 0x01;
    int type = 0x10;
    int target[4] = {0};
    int num_byte = 1;

    std::vector<unsigned char> vsend_data;
    unsigned char* byte_data = new unsigned char[4];

    for(int i = 0; i < 4; i++)
    {
        byte_data[i] = (unsigned char)( data >> (24 - i * 8));
    }

    for(int i = 0; i < num_byte; i ++)
    {
        vsend_data.push_back(byte_data[3 - i]);
    }


    int setting_ret = SR7IF_SetSetting(device_id_, depth, type, 0, 0x01, target,
                                       &vsend_data.at(0), num_byte);

    if (setting_ret < 0)
        return false;
    else
        return true;

}

void Sensor::getHeightUpperLower(double& _upper, double& _lower)
{
    const char* _version = SR7IF_GetModels(device_id_);   //型号获取
    QString str_Version;
    str_Version = QString(_version);

    double m_dHeightRange = 8.4;
    if(str_Version == tr("SR7050") || str_Version == tr("SR7060D"))
    {
        m_dHeightRange = 3;
    }
    else if(str_Version == tr("SR7080"))
    {
        m_dHeightRange = 9;
    }
    else if(str_Version == tr("SR7140"))
    {
        m_dHeightRange = 15;
    }
    else if(str_Version == tr("SR7240"))
    {
        m_dHeightRange = 24;
    }
    else if(str_Version == tr("SR7400"))
    {
        m_dHeightRange = 60;
    }
    else if(str_Version == tr("SR7300"))
    {
        m_dHeightRange = 150;
    }
    else if(str_Version == tr("SR6060"))
    {
        m_dHeightRange = 15;
    }
    else if(str_Version == tr("SR6030"))
    {
        m_dHeightRange = 8;
    }
    else if(str_Version == tr("SR6070"))
    {
        m_dHeightRange = 16;
    }
    else if(str_Version == tr("SR6071"))
    {
        m_dHeightRange = 40;
    }
    else if(str_Version == tr("SR6130"))
    {
        m_dHeightRange = 100;
    }
    else if(str_Version == tr("SR6260"))
    {
        m_dHeightRange = 240;
    }
    else if(str_Version == tr("SR8020"))
    {
        m_dHeightRange = 6;
    }
    else if(str_Version == tr("SR8060"))
    {
        m_dHeightRange = 20;
    }
    else if(str_Version == tr("SR7900"))
    {
        m_dHeightRange = 450;
    }
    else if(str_Version == tr("SR7060"))
    {
        m_dHeightRange = 6;
    }
    else if(str_Version == tr("SR71600"))
    {
        m_dHeightRange = 1500;
    }

    _upper = m_dHeightRange;
    _lower = -m_dHeightRange;
//    _upper = 0;
//    _lower = -200;
}

void Sensor::connectFunc()
{
    getHeightUpperLower(dheight_upper_, dheight_lower_); // determinant by the device
    int batch_width = SR7IF_ProfileDataWidth(device_id_, NULL);

    // get parameters from camera
    int ret = getEncoderParameters();

    if (ret != 0x02)
        setEncoderParameters();

    // initialize
    call_one_times_ptr_->VariableInit();
    call_one_times_ptr_->setDeviceId(device_id_);
    call_one_times_ptr_->DataMemoryInit(batch_width);
}

bool Sensor::ethenetConnect()
{
    SR7IF_ETHERNET_CONFIG sr_ethernet_config;

    // get ip address
    sr_ethernet_config.abyIpAddress[0] = 192;
    sr_ethernet_config.abyIpAddress[1] = 168;
    sr_ethernet_config.abyIpAddress[2] = 0;
    sr_ethernet_config.abyIpAddress[3] = 10;

    int open_ret = SR7IF_EthernetOpen(device_id_, &sr_ethernet_config);
    if (open_ret < 0)
    {
        int tmp_ret = SR7IF_EthernetOpen(device_id_, &sr_ethernet_config);

        //reconnect
        if (tmp_ret < 0)
           return false;

        else
        {
           connectFunc();
           return true;
        }
    }
    else
    {
        connectFunc();
        return true;
    }
}

bool Sensor::initBatch()
{
    int ret = call_one_times_ptr_->setBatchOneTimeDataHandler();
    if(ret < 0)
    {
        return false;
    }

    // start batch initialize
    call_one_times_ptr_->setBatchFinishFlagInit();

    //start batch
    ret = call_one_times_ptr_->startMeasureWithCallback();
    if(ret < 0)
    {
        return false;
    }

    // start display thread
    call_one_times_ptr_->startImageDisp();    

    return true;
}

bool Sensor::singleBatch()
{
    int ret = call_one_times_ptr_->SoftTrigger();
    if(ret < 0)
        return false;
    else
        return true;
}

QImage Sensor::BatchDataShow(int *_BatchData,
                                 double max_height,
                                 double min_height,
                                 int _ColorMax ,
                                 int img_w,
                                 int img_h,
                                 int _xscale,
                                 int _yscale,
                                 int _scaleW,
                                 int _scaleH )
{
    if (_BatchData == NULL || img_h == 0 || img_w == 0)
    {
        return QImage();
    }

    /* 数据转换 */
    double mSub = max_height - min_height;
    if(mSub <= 0.000001)
    {
        mSub = 1;
    }

    double fscale = double(_ColorMax) / mSub;   //颜色区间与高度区间比例
    if (fscale_ == 0)
        fscale_ = fscale;

    /* 抽帧抽点显示 */
    int imgW = _scaleW;
    int imgH = _scaleH;

    int TmpX = 0;
    int Tmppx = 0;

    if(img_h < imgH)
    {
        imgH = img_h;
    }

    if(img_w < imgW)
    {
        imgW = img_w;
    }

    if(_xscale <= 0)
    {
        _xscale = 1;
    }
    if(_yscale <= 0)
    {
        _yscale = 1;
    }

    int TT = (imgW * 8 + 31 ) / 32;   //图像宽度4字节对齐
    TT = TT * 4;

    int dwDataSize = TT * imgH;
    unsigned char* BatchImage = new unsigned char[dwDataSize];
    memset(BatchImage, 0, sizeof(BatchImage));

    std::shared_ptr<unsigned char> tmp_ptr(BatchImage);
    height_batch_ptr_ = std::move(tmp_ptr);

    for(int i = 0; i < imgH; i ++)
    {
        TmpX = i * _yscale * img_w;
        Tmppx = i * TT;
        for(int j = 0; j < imgW; j ++ )
        {
            double Tmp = double(_BatchData[TmpX + j * _xscale]) * 0.00001;
            if (Tmp < min_height)
                BatchImage[Tmppx + j] = 0;
            else if (Tmp > max_height)
                BatchImage[Tmppx + j] = 0xff;
            else
            {
                unsigned char tmpt = unsigned char((Tmp - min_height) * fscale);
                BatchImage[Tmppx + j] = tmpt;
            }
        }
    }

    //转成图像显示
    QImage heightImage = QImage(BatchImage, imgW, imgH, QImage::Format_Indexed8);


    QVector<QRgb> grayTable;

    for(int i = 0; i < 256; i++)
        grayTable.push_back(qRgb(i,i,i));

    heightImage.setColorTable(grayTable);

    return heightImage;
}

QImage Sensor::GrayDataShow(unsigned char* _grayData,
                          int img_w,
                          int img_h,
                          int _xscale,
                          int _yscale ,
                          int _scaleW,
                          int _scaleH)
{
    if (_grayData == NULL || img_h == 0 || img_w == 0)
    {
        return QImage();
    }

    /* 抽帧抽点显示 */
    int imgW = _scaleW;
    int imgH = _scaleH;

    int TmpX = 0;
    int Tmppx = 0;

    if(img_h < imgH)
    {
        imgH = img_h;
    }

    if(img_w < imgW)
    {
        imgW = img_w;
    }

    if(_xscale <= 0)
    {
        _xscale = 1;
    }
    if(_yscale <= 0)
    {
        _yscale = 1;
    }

    int TT = (imgW * 8 + 31 ) / 32;  //图像4字节对齐
    TT = TT * 4;

    unsigned char* BatchImage = new unsigned char[TT * imgH];
    memset(BatchImage, 0, sizeof(BatchImage));

    std::shared_ptr<unsigned char> tmp_ptr(BatchImage);
    gray_batch_ptr_ = std::move(tmp_ptr);

    for(int i = 0; i < imgH; i ++)
    {
        TmpX = i * _yscale * img_w;
        Tmppx = i * TT;
        for(int j = 0; j < imgW; j ++ )
        {
            BatchImage[Tmppx + j] = _grayData[TmpX + j * _xscale];
        }
    }

    //转成图像显示
    QImage grayImage = QImage(BatchImage, imgW, imgH, QImage::Format_Indexed8);

    QVector<QRgb> grayTable;

    for(int i = 0; i < 256; i++)
        grayTable.push_back(qRgb(i,i,i));

    grayImage.setColorTable(grayTable);

    return grayImage;
}

void Sensor::InitCallOneTimesBeforeDisConnect()
{
    if(call_one_times_ptr_)
    {
        call_one_times_ptr_->exitImageDiap();
        call_one_times_ptr_->deleteDataMemory();
    }
}

void Sensor::InitConfigBeforeDisConnect()
{
    SR7IF_StopMeasure(device_id_);
    InitCallOneTimesBeforeDisConnect();
}

void Sensor::ethenetDisconnect()
{
    InitConfigBeforeDisConnect();

    SR7IF_CommClose(device_id_);
}

Sensor::~Sensor()
{
//    ethenetDisconnect();
}


