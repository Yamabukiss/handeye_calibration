#ifndef CALLONETIMES_H
#define CALLONETIMES_H

/*****一次性回调模式A B 相机数据单独获取***
******数据缓存A B相机分开****************/

#include "SR7Link.h"
#include <QString>
#include <QThread>
#include <QMainWindow>
#include <QTime>
#define CAMNUM 2
#define MAXHEIGHT 15000

class ImageDisplayThread : public QThread
{
    Q_OBJECT
public:
    ImageDisplayThread();
    ~ImageDisplayThread();

public:
    bool bImageDispShow;            //图像显示标志
    bool bBatchFinish;              //批处理完成标志

protected:
    void run();

signals:
    void SignalDataShow();
};

class CallOneTimes : public QWidget
{
    Q_OBJECT

public:
    CallOneTimes(QWidget *parent = 0);
    ~CallOneTimes();
    int device_id_;
    double mXinterVal;                      //X间距
    double mYinterVal;                      //Y间距



public:
    ///
    /// \brief VariableInit             变量初始化
    ///
    void VariableInit();
    ///
    /// \brief DataMemoryInit           数据内存初始化
    /// \param mProW                   轮廓宽度
    ///
    void DataMemoryInit(int mProW = 3200);
    ///
    /// \brief deleteDataMemory         数据内存释放
    ///
    void deleteDataMemory();
    ///
    /// \brief setDeviceId    设置设备ID号
    /// \param _id
    ///
    void setDeviceId(int _id) { mDeviceId = _id; }
    ///
    /// \brief setBatchWait  设置立即或等待开始批处理变量
    /// \param _val
    ///
    void setBatchWait(int _val) { m_BatchWait = _val; }
    ///
    /// \brief setBatchOneTimeDataHandler  设置回调函数
    /// \return                            0:成功  <0:失败
    ///
    int setBatchOneTimeDataHandler();
    ///
    /// \brief startMeasureWithCallback  开始批处理
    /// \return                          0:成功  <0:失败
    ///
    int startMeasureWithCallback();
    ///
    /// \brief SoftTrigger         软触发
    /// \return                    0:成功  <0:失败
    ///
    int SoftTrigger();
    ///
    /// \brief startImageDisp      图像显示开始
    ///
    void startImageDisp();
    ///
    /// \brief exitImageDiap       退出图像显示
    ///
    void exitImageDiap();
    ///
    /// \brief getBatchData        获取批处理数据
    /// \param _camId              相机ID
    /// \return                    数据地址
    ///
    int* getBatchData(int _camId);
    ///
    /// \brief getIntensityData    获取亮度(灰度)数据
    /// \param _camId              相机ID
    /// \return                    亮度(灰度)数据地址
    ///
    unsigned char* getIntensityData(int _camId);
    ///
    /// \brief getEncoderData      获取编码器数据
    /// \param _camId              相机ID
    /// \return                    编码器数据地址
    ///
    unsigned int* getEncoderData(int _camId);
    ///
    /// \brief getBatchHeight      获取批处理行数
    /// \return
    ///
    int getBatchHeight() { return mBatchPoint; }
    ///
    /// \brief getProfileWidth     获取轮廓宽度
    /// \return
    ///
    int getProfileWidth() { return mBatchWidth; }
    ///
    /// \brief getXpitch           获取X间距
    /// \return
    ///
    double getXpitch() { return mXinterVal; }
    ///
    /// \brief ImageUpdate         更新图像
    ///
    void ImageUpdate();
    ///
    /// \brief saveHeightData      高度数据保存
    /// \param _path               路径
    /// \param _camId              相机ID
    /// \return                    0:成功 <0:失败
    ///
    int saveHeightData(QString _path, int _camId);
    ///
    /// \brief saveIntensityData   灰度数据保存
    /// \param _path               路径
    /// \param _camId              相机ID
    /// \return                    0:成功 <0:失败
    ///
    int saveIntensityData(QString _path, int _camId);
    ///
    /// \brief saveEncoderData     编码器数据保存
    /// \param _path               路径
    /// \param _camId              相机ID
    /// \return                    0:成功 <0:失败
    ///
    int saveEncoderData(QString _path, int _camId);
    ///
    /// \brief set3DYinterval      设置3D显示Y方向间距
    /// \param _val
    ///
    void set3DYinterval(double _val) { mYinterVal = _val; }
    ///
    /// \brief setBatchFinishFlagInit  批处理结束标志初始化
    ///
    void setBatchFinishFlagInit();

    ImageDisplayThread* mImgDispT;          //图像显示线程

private:
    ///
    /// \brief BatchOneTimeCallBack     回调函数
    /// \param info
    /// \param data
    ///
    static void BatchOneTimeCallBack(const void *info,
                                     const SR7IF_Data *data);
    ///
    /// \brief setTextInfo              信息显示
    /// \param _text
    ///
    void setTextInfo(QString _text);
    ///
    /// \brief DataShowSignalFunc      数据显示线程信号响应
    ///
    void DataShowSignalFunc();

private:
    static CallOneTimes* DataCallModel;     //静态变量--用于回调函数里调用非静态变量
    int mDeviceId;                          //设备ID号
    int m_BatchWait;                        //立即或等待开始批处理变量
    int mBatchPoint;                        //批处理行数设置
    int mCurBatchPt;                        //回调一次获取批处理行数
    int mBatchTimes;                        //批处理次数
    int mBatchWidth;                        //轮廓宽度

    int* mProfileData[CAMNUM];              //批处理数据缓存 下标 0：A相机  1：B相机
    unsigned char* mIntensityData[CAMNUM];  //灰度数据缓存
    unsigned int* mEncoderData[CAMNUM];     //编码器数据缓存



signals:
    void SignalTextInfo(QString _text);     //信息显示信号
    void SignalDataShow(int _width,
                        int _height);       //数据显示信号

    void save3dSignal(bool signal);

};

#endif // CALLONETIMES_H
