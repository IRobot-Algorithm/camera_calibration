
/*-------------------------------------------------------------
brief  :  Mercure Camera driver 
author :  XDU Robomaster, skystarry
date   :  2019.05.29
---------------------------------------------------------------*/
#include "mercure_driver.h"
#include <sstream>

namespace camera{


MercureDriver::MercureDriver():
    status_(GX_STATUS_SUCCESS),
    device_(nullptr),
    pFrameBuffer_(nullptr)
{
  GetSetParam();
  init_sdk();
  LoadParam();
  rgbImagebuf_ = new uint8_t[param_.height_ * param_.width_ * 3];

  status_ = GXStreamOn(device_);

    if(status_ != GX_STATUS_SUCCESS)
    {
      if (rgbImagebuf_ != nullptr)
      {
        delete[] rgbImagebuf_;
        rgbImagebuf_ = nullptr;
      }
      GXCloseDevice(device_);
      device_ = nullptr;
      GXCloseLib();
      //ROS_ERROR("Exit!");  
    }
}

void MercureDriver::GetSetParam()
{
    if (!ros::param::get("camera_id", camera_id_))
    {
        std::cout << "Can not get the value of camera_id" << std::endl;
        exit(1);
    }   
    if (!ros::param::get("image_width", param_.width_))
    {
        std::cout << "Can not get the value of image_width" << std::endl;
        exit(1);
    }
    if (!ros::param::get("image_height", param_.height_))
    {
        std::cout << "Can not get the value of image_height" << std::endl;
        exit(1);
    }
}


GX_STATUS MercureDriver::init_sdk()
{
    //ROS_WARN("Mercure Camera Initializing......\n");
    uint32_t ui32DeviceNum = 0;
    
    status_ = GXInitLib(); 
    if(status_ != GX_STATUS_SUCCESS)
    {
      //ROS_ERROR("Initialize Mercure Camera Driver libary failed. Error Code: %d", status_);
      return status_;
    }

    status_ = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if(status_ != GX_STATUS_SUCCESS)
    { 
        //ROS_ERROR("Get Mercure Camera device enumerated number failed. Error Code: %d", status_);
        GXCloseLib();
        return status_;
    }

    if(ui32DeviceNum <= 0)
    {
        //ROS_WARN("No Mercure Camera device found.");
        GXCloseLib();
        return status_;
    }

    status_ = GXOpenDeviceByIndex(camera_id_, &device_);

    if(status_ != GX_STATUS_SUCCESS)
    {
        //ROS_ERROR("Open Mercure Camera failed. Error Code: %d", status_);
        GXCloseLib();
        return status_;           
    }
    GetVision();
    
    //status_ = GXGetInt(device_, GX_INT_PAYLOAD_SIZE, &payloadsize_);
    status_ = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    status_ = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    
    status_ = GXSetAcqusitionBufferNumber(device_, ACQ_BUFFER_NUM);
    
    //bool bStreamTransferSize = false;
    //status_ = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    //if(bStreamTransferSize) {
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
    //}

    //bool bStreamTransferNumberUrb = false;
    //status_ = GXIsImplemented(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    //if(bStreamTransferNumberUrb) {
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
    //}

    // support
    status_ = GXSetInt(device_, GX_INT_WIDTH, param_.width_);
    status_ = GXSetInt(device_, GX_INT_HEIGHT, param_.height_);
    status_ = GXSetInt(device_, GX_INT_OFFSET_X, (1920 - param_.width_) / 2);
    status_ = GXSetInt(device_, GX_INT_OFFSET_Y, (1200 - param_.height_) / 2);

    return status_;
}

void MercureDriver::LoadParam()
{
    if (!ros::param::get("exposure_auto", param_.exp_auto_))
    {
        std::cout << "Can not get the value of exposure_auto" << std::endl;
        exit(1);
    }
    if (!ros::param::get("exposure_time", param_.exp_time_))
    {
        std::cout << "Can not get the value of exposure_time" << std::endl;
        exit(1);
    }


    if(param_.exp_auto_){
        status_ = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);

        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, 5000);
        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, 20000);
    }
    else{
        status_ = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);

        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, param_.exp_time_ - 200);
        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, param_.exp_time_ + 200);
        status_ = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, param_.exp_time_);
    }


    if (!ros::param::get("w_auto", param_.w_auto_))
    {
        std::cout << "Can not get the value of w_auto" << std::endl;
        exit(1);
    }
    if (!ros::param::get("w_red", param_.w_red_))
    {
        std::cout << "Can not get the value of w_red" << std::endl;
        exit(1);
    }
    if (!ros::param::get("w_green", param_.w_green_))
    {
        std::cout << "Can not get the value of w_green" << std::endl;
        exit(1);
    }
    if (!ros::param::get("w_blue", param_.w_blue_))
    {
        std::cout << "Can not get the value of w_blue" << std::endl;
        exit(1);
    }

    /* Sense ROI
    status_ = GXSetInt(device_, GX_INT_AWBROI_WIDTH,   1920);
    status_ = GXSetInt(device_, GX_INT_AWBROI_HEIGHT,  1200);
    status_ = GXSetInt(device_, GX_INT_AWBROI_OFFSETX, 0);
    status_ = GXSetInt(device_, GX_INT_AWBROI_OFFSETY, 0);
    */

    if(param_.w_auto_){
        
        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        
        /*
        status_ = GXSetEnum(device_, GX_ENUM_AWB_LAMP_HOUSE, GX_AWB_LAMP_HOUSE_FLUORESCENCE);

        GX_AWB_LAMP_HOUSE_ADAPTIVE            = 0,           ///< Adaptive light source
        GX_AWB_LAMP_HOUSE_D65                 = 1,           ///< Color temperature 6500k
        GX_AWB_LAMP_HOUSE_FLUORESCENCE        = 2,           ///< Fluorescent
        GX_AWB_LAMP_HOUSE_INCANDESCENT        = 3,           ///< Incandescent
        GX_AWB_LAMP_HOUSE_D75                 = 4,           ///< Color temperature 7500k
        GX_AWB_LAMP_HOUSE_D50                 = 5,           ///< Color temperature 5000k
        GX_AWB_LAMP_HOUSE_U30                 = 6,           ///< Color temperature 3000k
        */
    }
    else{
        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);

        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
        status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, param_.w_red_);

        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
        status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, param_.w_green_);

        status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
        status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, param_.w_blue_);
    }


    if (!ros::param::get("gain_auto", param_.gain_auto_))
    {
        std::cout << "Can not get the value of gain_auto" << std::endl;
        exit(1);
    }
    if (!ros::param::get("gain", param_.gain_))
    {
        std::cout << "Can not get the value of gain" << std::endl;
        exit(1);
    }


    if(param_.gain_auto_)
    {
        status_ = GXSetEnum(device_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, 0);
        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, 10);
    }
    else{
        status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        //status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_RED);
        //status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_GREEN);
        //status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_BLUE);

        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, 0);
        status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, 10);
        status_ = GXSetFloat(device_, GX_FLOAT_GAIN, param_.gain_);
    }
    /*
    if(black_auto_){
        status_ = GXSetEnum(device_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);
    }else{
        status_ = GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_ALL);
        //status_=GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_RED);
        //status_=GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_GREEN);
        //status_=GXSetEnum(device_, GX_ENUM_BLACKLEVEL_SELECTOR, GX_BLACKLEVEL_SELECTOR_BLUE);
        //GX_FLOAT_RANGE blackLevelRange;
        //blackLevelRange.dMin
        //blackLevelRange.dMax
        //status_ = GXGetFloatRange(device_, GX_FLOAT_BLACKLEVEL, &blackLevelRange);
        //status_ = GXSetFloat(device_, GX_Float_BLACKLEVEL, blackLevelRange.dMin);
        //status_ = GXSetFloat(device_, GX_Float_BLACKLEVEL, blackLevelRange.dMax);
    }*/
    
}

void MercureDriver::GetVision()
{
   // ROS_WARN("<Libary Version : %s>", GXGetLibVersion());
    size_t nSize = 0;
    GXGetStringLength(device_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    char *pszVendorName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    //ROS_WARN("<Vendor Name : %s>", pszVendorName);
    delete[] pszVendorName;
    pszVendorName = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    char *pszModelName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
   // ROS_WARN("<Model Name : %s>", pszModelName);
    delete[] pszModelName;
    pszModelName = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    char *pszSerialNumber = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    //ROS_WARN("<Serial Number : %s>", pszSerialNumber);
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    GXGetStringLength(device_, GX_STRING_DEVICE_VERSION, &nSize);
    char *pszDeviceVersion = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    //ROS_WARN("<Device Version : %s>", pszDeviceVersion);
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
}

// 可以记录开始时间以及结束时间
void MercureDriver::operator >> (cv::Mat& Image)
{       
        // printf("1");
            // auto speed_test_start_begin_time = std::chrono::steady_clock::now();
        // printf("2");
            GXDQBuf(device_, &pFrameBuffer_, 1000);
        // printf("3");
       DxRaw8toRGB24(pFrameBuffer_->pImgBuf, 
                     rgbImagebuf_, 
                     pFrameBuffer_->nWidth,
                     pFrameBuffer_->nHeight,
                     RAW2RGB_NEIGHBOUR, 
                     DX_PIXEL_COLOR_FILTER(4),//DX_PIXEL_COLOR_FILTER(colorfilter_), 
                     false);

       memcpy(Image.data, rgbImagebuf_, pFrameBuffer_->nHeight * pFrameBuffer_->nWidth * 3);
      
      GXQBuf(device_, pFrameBuffer_);

    //   auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0;
      //ROS_INFO("mercure acq time cost = %.2f ms", cost);	
}

MercureDriver::~MercureDriver()
{
    status_ = GXStreamOff(device_);
    if(status_ != GX_STATUS_SUCCESS)
    {
      delete[] rgbImagebuf_;
      rgbImagebuf_ = nullptr;
      GXCloseDevice(device_);
      device_ = nullptr;
      GXCloseLib();
     // ROS_ERROR("Exit!");  
    }

    delete[] rgbImagebuf_;
    rgbImagebuf_ = nullptr;

    status_ = GXCloseDevice(device_);
    if(status_ != GX_STATUS_SUCCESS)
    {
       // ROS_ERROR("GXCloseDevice failed. ");
        device_ = nullptr;
        GXCloseLib();
    }
    status_ = GXCloseLib();
    if(status_ != GX_STATUS_SUCCESS)
    {
      //  ROS_ERROR("GXCloseLib failed. ");
    }
}

} // namespace camera

