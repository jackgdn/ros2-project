#include <iostream>
#include <opencv2/opencv.hpp>
#include <zbar.h>
//二维码识别基类
class QRcodeScanner
{
    public:
        virtual std::string getQRresult(cv::Mat& raw_image)=0;
};
//使用ZbarC++方法进行识别    70-80ms左右
class ZbarQRcodeScanner: public QRcodeScanner
{
    private:
        zbar::ImageScanner* image_scanner; 
        cv::Mat gray_image;
    public:
        ZbarQRcodeScanner():image_scanner(new zbar::ImageScanner)
        {
            image_scanner->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        }
         ~ZbarQRcodeScanner()
        {
            delete image_scanner;
        }
        std::string getQRresult(cv::Mat& raw_image)
        {
           
            if(raw_image.empty())
            {
            std::cout<<"EMPTY IMAGE~~"<<std::endl;
            return "";
            }
           
            cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);
            
            zbar::Image zbar_image(gray_image.cols, gray_image.rows, "Y800", gray_image.data, gray_image.cols * gray_image.rows);

            image_scanner->scan(zbar_image);//最耗时操作，70ms
            std::string result;
            if (zbar_image.symbol_begin() == zbar_image.symbol_end())
	        {
		        //std::cout << "identify fail" << std::endl;
	        }
            for (zbar::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
            {
                result = symbol->get_data();
                if (result!="")
                {
                    break;
                }
                
            }
            //std::cout<<"result:"<<result<<std::endl;
            // 清理 ZBar 图像
            zbar_image.set_data(NULL, 0);
            return result;
        }
};