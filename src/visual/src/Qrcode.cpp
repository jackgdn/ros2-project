#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/wechat_qrcode.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
class QrcodeTest: public rclcpp::Node
{
    public:
        QrcodeTest():
            Node("path_msg_noder")
        {  
            auto qos_profile_1 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            auto qos_profile_2 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
            image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("/image",1,std::bind(&QrcodeTest::imageCB, this, std::placeholders::_1));
            qrcodePub = this->create_publisher<std_msgs::msg::String>("/qrcode",qos_profile_1);
            signPub = this->create_publisher<std_msgs::msg::Int32>("/sign",qos_profile_2);
        }
        const std::string modelDir = "/root/dev_ws/src/visual/model/";
        cv::wechat_qrcode::WeChatQRCode detector{
        modelDir + "detect.prototxt", 
        modelDir + "detect.caffemodel",
        modelDir + "sr.prototxt",     
        modelDir + "sr.caffemodel"
        };
        cv::Mat raw_image;
        cv::Mat detect_img;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qrcodePub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr signPub;
        void get(){
            auto start = std::chrono::high_resolution_clock::now();
            std::vector<cv::Mat> points;
            auto res = detector.detectAndDecode(raw_image, points);

            if(res.size() !=0 ){
                std_msgs::msg::String result;
                std_msgs::msg::Int32 sign;
                result.data = res.front();
                std::cout<<res.front()<<std::endl;
                if((res.front().back() - '0') % 2 == 0){
                    sign.data = 4;
                }
                else   
                    sign.data = 3;
                qrcodePub->publish(result);
                signPub->publish(sign);
            }
            else{
                std_msgs::msg::String result;
                qrcodePub->publish(result);
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            //std::cout << "二维码耗时: " << duration.count() << " 毫秒" << std::endl;
        }
        void imageCB(const sensor_msgs::msg::CompressedImage::SharedPtr raw_image_data)
        {
            if (raw_image_data->data.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Received empty image data.");
                return;
            }
            raw_image = cv::imdecode(cv::Mat(raw_image_data->data), cv::IMREAD_UNCHANGED);
            get();
        }
};
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QrcodeTest>());
    rclcpp::shutdown();
    return 0;
}
