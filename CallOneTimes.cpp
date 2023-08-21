#include "CallOneTimes.h"
#include <QMessageBox>
#include <QDebug>
#include <QIcon>
#include <exception>
#include <QFile>
#include <QApplication>
#include <fstream>
#include <QTextCodec>

using namespace std;
#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")
#endif

#pragma pack(4)
typedef struct {
    unsigned int version;
    int width;
    int height;
    double xInterval;
    double yInterval;
    char info[32];
    int reserve2[2545];
}BATCH_INFO;    //确保10240字节
#pragma pack()

enum {
    BATCH_SAVE_FILE_VERSION = /*0x00000101*/0x00000002,
};

CallOneTimes* CallOneTimes::DataCallModel = NULL;
CallOneTimes::CallOneTimes(QWidget *parent)
{
    DataCallModel = this;
    mImgDispT = new ImageDisplayThread();
    connect(mImgDispT, &ImageDisplayThread::SignalDataShow,
            this, &CallOneTimes::DataShowSignalFunc);
}

void CallOneTimes::VariableInit()
{
    mDeviceId = 0;
    m_BatchWait = 0;
    mBatchTimes = 0;
    mCurBatchPt = 0;
    mBatchPoint = 0;
    mBatchWidth = 0;
    mXinterVal = 0;
    mYinterVal = 0.175;
}

void CallOneTimes::DataMemoryInit(int mProW)
{
    //数据缓存申请
    int mDataC = MAXHEIGHT * mProW;
    try
    {
        mProfileData[0] = new int[mDataC];
    }
    catch (exception& e)
    {
        Q_UNUSED(e)
        QMessageBox bbox(QMessageBox::Warning, QString("CallOneTimes"),
                         QString("数据内存申请失败。"),
                         QMessageBox::NoButton);
        bbox.setIconPixmap(QPixmap(":/Image/warning.png"));
        bbox.setWindowIcon(QIcon(":/Image/warning.png"));
        bbox.setWindowFlags(Qt::Dialog|Qt::CustomizeWindowHint|Qt::WindowTitleHint);
        bbox.setButtonText(QMessageBox::Ok, QString(QString("确 定")));
        bbox.exec();
        exit(0);
    }

    try
    {
        mIntensityData[0] = new unsigned char[mDataC];
        mEncoderData[0] = new unsigned int[MAXHEIGHT];
    }
    catch (exception& e)
    {
        Q_UNUSED(e)
        QMessageBox bbox(QMessageBox::Warning, QString("CallOneTimes"),
                         QString("数据内存申请失败。"),
                         QMessageBox::NoButton);
        bbox.setIconPixmap(QPixmap(":/Image/warning.png"));
        bbox.setWindowIcon(QIcon(":/Image/warning.png"));
        bbox.setWindowFlags(Qt::Dialog|Qt::CustomizeWindowHint|Qt::WindowTitleHint);
        bbox.setButtonText(QMessageBox::Ok, QString(QString("确 定")));
        bbox.exec();
        exit(0);
    }

    try
    {
        mProfileData[1] = new int[mDataC];
    }
    catch (exception& e)
    {
        Q_UNUSED(e)
        QMessageBox bbox(QMessageBox::Warning, QString("CallOneTimes"),
                         QString("数据内存申请失败。"),
                         QMessageBox::NoButton);
        bbox.setIconPixmap(QPixmap(":/Image/warning.png"));
        bbox.setWindowIcon(QIcon(":/Image/warning.png"));
        bbox.setWindowFlags(Qt::Dialog|Qt::CustomizeWindowHint|Qt::WindowTitleHint);
        bbox.setButtonText(QMessageBox::Ok, QString(QString("确 定")));
        bbox.exec();
        exit(0);
    }

    try
    {
        mIntensityData[1] = new unsigned char[mDataC];
        mEncoderData[1] = new unsigned int[MAXHEIGHT];
    }
    catch (exception& e)
    {
        Q_UNUSED(e)
        QMessageBox bbox(QMessageBox::Warning, QString("CallOneTimes"),
                         QString("数据内存申请失败。"),
                         QMessageBox::NoButton);
        bbox.setIconPixmap(QPixmap(":/Image/warning.png"));
        bbox.setWindowIcon(QIcon(":/Image/warning.png"));
        bbox.setWindowFlags(Qt::Dialog|Qt::CustomizeWindowHint|Qt::WindowTitleHint);
        bbox.setButtonText(QMessageBox::Ok, QString(QString("确 定")));
        bbox.exec();
        exit(0);
    }
}

CallOneTimes::~CallOneTimes()
{
    deleteDataMemory();
}

void CallOneTimes::deleteDataMemory()
{
    if(mProfileData[0])
    {
        delete mProfileData[0];
        mProfileData[0] = NULL;
    }
    if(mIntensityData[0])
    {
        delete mIntensityData[0];
        mIntensityData[0] = NULL;
    }
    if(mEncoderData[0])
    {
        delete mEncoderData[0];
        mEncoderData[0] = NULL;
    }

    if(mProfileData[1])
    {
        delete mProfileData[1];
        mProfileData[1] = NULL;
    }
    if(mIntensityData[1])
    {
        delete mIntensityData[1];
        mIntensityData[1] = NULL;
    }
    if(mEncoderData[1])
    {
        delete mEncoderData[1];
        mEncoderData[1] = NULL;
    }
}

ImageDisplayThread::ImageDisplayThread()
{
    bImageDispShow = false;
    bBatchFinish = false;
}

ImageDisplayThread::~ImageDisplayThread()
{

}

void CallOneTimes::startImageDisp()
{
    if(mImgDispT)
    {
        mImgDispT->bImageDispShow = true;

        /* 启动线程 */
        mImgDispT->start();
    }
}

void CallOneTimes::exitImageDiap()
{
    if(mImgDispT)
    {
        mImgDispT->bImageDispShow = false;
        if(mImgDispT->isRunning())
        {
            mImgDispT->quit();
            while(mImgDispT->isRunning())  //等待线程退出完成
            {
                qApp->processEvents();
            }
        }
    }
}

void CallOneTimes::setBatchFinishFlagInit()
{
    mImgDispT->bBatchFinish = false;
}

void ImageDisplayThread::run()
{
    while(bImageDispShow)
    {
        if(bBatchFinish)   //批处理结束后显示
        {
            bBatchFinish = false;
            SignalDataShow();
        }
    }
}

void CallOneTimes::DataShowSignalFunc()
{
    SignalDataShow(mBatchWidth, mBatchPoint);
}

void CallOneTimes::ImageUpdate()
{
    SignalDataShow(mBatchWidth, mBatchPoint);
}

int* CallOneTimes::getBatchData(int _camId)
{
    if(_camId < 0 || _camId > CAMNUM - 1)
        return NULL;
    return mProfileData[_camId];
}

unsigned char* CallOneTimes::getIntensityData(int _camId)
{
    if(_camId < 0 || _camId > CAMNUM - 1)
        return NULL;
    return mIntensityData[_camId];
}

unsigned int* CallOneTimes::getEncoderData(int _camId)
{
    if(_camId < 0 || _camId > CAMNUM - 1)
        return NULL;
    return mEncoderData[_camId];
}

int CallOneTimes::saveHeightData(QString _path, int _camId)
{
    if(mProfileData[_camId] == NULL || mBatchPoint == 0 || mBatchWidth == 0
            || _camId < 0 || _camId > CAMNUM - 1)
    {
        return -1;
    }

    //获取后缀
    QString strA = _path.mid(_path.length() - 4);
    if(strA == ".bin")
    {
        QFile file(_path);
        if(file.open(QIODevice::WriteOnly))
        {
            file.write((char *)mProfileData[_camId],
                       sizeof(int) * mBatchPoint * mBatchWidth);
            file.close();
        }
        else
        {
            return -1;
        }
    }
    else if(strA == ".ecd")
    {
        BATCH_INFO mBatchInfo;
        mBatchInfo.version = BATCH_SAVE_FILE_VERSION;
        mBatchInfo.width = mBatchWidth;
        mBatchInfo.height = mBatchPoint;
        mBatchInfo.xInterval = mXinterVal;
        mBatchInfo.yInterval = mYinterVal;
        strcpy(mBatchInfo.info, "SSZN2021 V00000002");
        QFile file(_path);
        if(file.open(QIODevice::WriteOnly))
        {
            file.write((char *)&mBatchInfo, sizeof(BATCH_INFO));
            file.write((char *)mProfileData[_camId],
                                   mBatchWidth * mBatchPoint * sizeof(int));

            //写入编码器信息
            if(mEncoderData[_camId])
            {
                file.write((char *)mEncoderData[_camId],
                                       mBatchPoint * sizeof(int));
            }

            file.close();
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }

    return 0;
}

int CallOneTimes::saveIntensityData(QString _path, int _camId)
{
    if(mIntensityData[_camId] == NULL || mBatchPoint == 0 || mBatchWidth == 0
            || _camId < 0 || _camId > CAMNUM - 1)
    {
        return -1;
    }

    //获取后缀
    QString strA = _path.mid(_path.length() - 4);
    if(strA == ".bin")
    {
        QFile file(_path);
        if(file.open(QIODevice::WriteOnly))
        {
            file.write((char *)mIntensityData[_camId],
                       sizeof(unsigned char) * mBatchPoint * mBatchWidth);
            file.close();
        }
        else
        {
            return -1;
        }
    }
    else if(strA == ".bmp")
    {
        QImage mImage =  QImage(mBatchWidth,
                                mBatchPoint,
                                QImage::Format_Indexed8);
        QVector<QRgb> grayTable;
        for(int i = 0; i < 256; i++)
        {
            grayTable.push_back(qRgb(i,i,i));
        }
        mImage.setColorTable(grayTable);

        uchar *pGray = mImage.bits();
        if((mBatchWidth % 4) != 0)
        {
            int xPoints = (mBatchWidth + 4) / 4;
            xPoints = xPoints * 4;
            for(int i = 0; i < mBatchPoint; i ++)
            {
                memcpy(pGray + xPoints * i,
                       mIntensityData[_camId] + mBatchWidth * i, mBatchWidth);
            }
        }
        else
        {
            memcpy(pGray, mIntensityData[_camId], mBatchWidth * mBatchPoint);
        }

        if(!mImage.save(_path))
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }

    return 0;
}

int CallOneTimes::saveEncoderData(QString _path, int _camId)
{
    if(mEncoderData[_camId] == NULL || mBatchPoint == 0
            || _camId < 0 || _camId > CAMNUM - 1)
    {
        return -1;
    }

    //获取后缀
    QString strA = _path.mid(_path.length() - 4);
    if(strA == ".txt")
    {
        try
        {
            QTextCodec *code = QTextCodec::codecForName("gb18030");
            //此时的name所指向的文件即可直接被程序内部读取，包括中文路径
            std::string name = code->fromUnicode(_path).data();
            char* strFilePath = (char*)name.c_str();
            std::fstream file(strFilePath, std::ios::out);

            /* 以逗号隔开写入 */
            char aa[32];
            memset(aa, 0, sizeof(aa));
            for (unsigned int i = 0; i < mBatchPoint; i++)
            {
                int tt = i * 1;
                for (int j = 0; j < 1; j++)
                {
                    int len = sprintf(aa, "%u ", mEncoderData[_camId][tt + j]);
                    file.write(aa, len);
                }
                file << "\n";
            }
            file.close();
        }
        catch (exception& e)
        {
            return -1;
        }
    }
    else if(strA == ".bin")
    {
        QFile file(_path);
        if(file.open(QIODevice::WriteOnly))
        {
            file.write((char *)mEncoderData[_camId],
                       sizeof(unsigned) * mBatchPoint);
            file.close();
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }

    return 0;
}

void CallOneTimes::BatchOneTimeCallBack(const void *info, const SR7IF_Data *data)
{

    const SR7IF_STR_CALLBACK_INFO * conInfo = (const SR7IF_STR_CALLBACK_INFO*) info;
    if(conInfo->returnStatus != SR7IF_OK)
    {
//        QString strErrorInfo = tr("回调停止，返回值：%1").arg(conInfo->returnStatus);
        QString strErrorInfo = tr("批处理已经关闭，其回调返回值为:%1").arg(conInfo->returnStatus);
        DataCallModel->setTextInfo(strErrorInfo);    //界面显示信息
        return;
    }

    //信息---info
    QString strInfo = QString("");
    QString stmp = QString("传感头数量: %1").arg(conInfo->HeadNumber);
    strInfo += stmp;
    strInfo += QString("\r\n");
    stmp = QString("当前获取批处理行数: %1").arg(conInfo->BatchPoints);
    strInfo += stmp;
    strInfo += QString("\r\n");
    stmp = QString("X方向数据数量: %1").arg(conInfo->xPoints);
    strInfo += stmp;
    strInfo += QString("\r\n");
    stmp = QString("X方向间距: ") + QString::number(conInfo->xPixth, 'f', 3);
    strInfo += stmp;
    strInfo += QString("\r\n");

    if(conInfo->BatchPoints != 0)
    {
        DataCallModel->mBatchTimes ++;
    }
    stmp = QString("批处理次数: %1").arg(DataCallModel->mBatchTimes);
    strInfo += stmp;
    strInfo += QString("\r\n");

    DataCallModel->setTextInfo(strInfo);   //界面显示信息

    //高度数据获取--相机A
    const int* mTmpData = SR7IF_GetBatchProfilePoint(data, 0);
    int mNumP = conInfo->BatchPoints * conInfo->xPoints;
    if(mTmpData !=  NULL)
    {
        for(int i = 0; i < mNumP; i ++)
        {
            DataCallModel->mProfileData[0][i] = -1000000000;
        }
        memcpy(&DataCallModel->mProfileData[0][0], &mTmpData[0], mNumP * sizeof(int));
    }

    //灰度数据获取--相机A
    int mNumG = sizeof(unsigned char) * conInfo->BatchPoints * conInfo->xPoints;
    const unsigned char* mTmpGraydata = SR7IF_GetBatchIntensityPoint(data, 0);
    if(mTmpGraydata != NULL)
    {
        memset(&DataCallModel->mIntensityData[0][0], 0, mNumG);
        memcpy(&DataCallModel->mIntensityData[0][0], &mTmpGraydata[0], mNumG);
    }

    //编码器数据获取--相机A
    int mNumE = sizeof(unsigned int) * conInfo->BatchPoints;
    const unsigned int* mTmpEncoderdata  = SR7IF_GetBatchEncoderPoint(data, 0);
    if(mTmpEncoderdata != NULL)
    {
        memset(&DataCallModel->mEncoderData[0][0], 0, mNumE);
        memcpy(&DataCallModel->mEncoderData[0][0], &mTmpEncoderdata[0], mNumE);
    }

    //双相机
    if(conInfo->HeadNumber == 2)
    {
        //相机B数据---高度数据
        const int* mTmpDataB = SR7IF_GetBatchProfilePoint(data, 1);
        if(mTmpData !=  NULL)
        {
            for(int i = 0; i < mNumP; i ++)
            {
                DataCallModel->mProfileData[1][i] = -1000000000;
            }
            memcpy(&DataCallModel->mProfileData[1][0], &mTmpDataB[0], mNumP * sizeof(int));
        }
        //相机B数据---灰度数据
        const unsigned char* mTmpGraydataB = SR7IF_GetBatchIntensityPoint(data, 1);
        if(mTmpGraydataB != NULL)
        {
            memset(&DataCallModel->mIntensityData[1][0], 0, mNumG);
            memcpy(&DataCallModel->mIntensityData[1][0], &mTmpGraydataB[0], mNumG);
        }
        //相机B数据---编码器数据
        const unsigned int* mTmpEncoderdataB  = SR7IF_GetBatchEncoderPoint(data, 1);
        if(mTmpEncoderdata != NULL)
        {
            memset(&DataCallModel->mEncoderData[1][0], 0, mNumE);
            memcpy(&DataCallModel->mEncoderData[1][0], &mTmpEncoderdataB[0], mNumE);
        }
    }


    //行数 宽度赋值
    DataCallModel->mBatchPoint = conInfo->BatchPoints;    //当前批处理行数
    DataCallModel->mBatchWidth = conInfo->xPoints;        //批处理宽度
    DataCallModel->mXinterVal = conInfo->xPixth;          //X间距

    DataCallModel->mImgDispT->bBatchFinish = true;        //批处理结束
}

void CallOneTimes::setTextInfo(QString _text)
{
    SignalTextInfo(_text);   //信号--传递给主窗口界面显示
}

int CallOneTimes::setBatchOneTimeDataHandler()
{
    int reT = SR7IF_SetBatchOneTimeDataHandler(mDeviceId,
                                               BatchOneTimeCallBack);
    return reT;
}

int CallOneTimes::startMeasureWithCallback()
{
    int reT = SR7IF_StartMeasureWithCallback(mDeviceId, m_BatchWait);
    return reT;
}

int CallOneTimes::SoftTrigger()
{
    int reT = SR7IF_TriggerOneBatch(mDeviceId);
    return reT;
}
