#ifndef MSG_CENTER
#define MSG_CENTER
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "ai_msgs/msg/target.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
class MsgCenter : public rclcpp::Node
{
    public:
        MsgCenter(): Node("msg_center"),
        car_state(1),
        global_path(new std::vector<ai_msgs::msg::Target>()),
        obstacles(new std::vector<ai_msgs::msg::Target>())
        {
            std::cout<<"msgcenter init"<<std::endl;
            yoloResultSub = this->create_subscription<ai_msgs::msg::PerceptionTargets>("/hobot_dnn_detection",1,std::bind(&MsgCenter::yoloResultCB, this, std::placeholders::_1));
            velPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
            signSub = this->create_subscription<std_msgs::msg::Int32>("/sign",1,std::bind(&MsgCenter::signCB, this, std::placeholders::_1));
        }
        int car_state;
        std::vector<ai_msgs::msg::Target>* global_path;
        std::vector<ai_msgs::msg::Target>* obstacles;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub;
    private:
        void signCB(const std_msgs::msg::Int32::SharedPtr msg){
            if(msg->data == 3 ||msg->data == 4)
                car_state = 2;
            if(msg->data == 6)
                car_state = 3;
        }
        void yoloResultCB(const ai_msgs::msg::PerceptionTargets::SharedPtr yolo_result){
            global_path->clear();
            obstacles->clear();
            for (auto target :yolo_result->targets){
                if(target.type == "lines\r")
                    global_path->push_back(target);
                //这里补逻辑 根据障碍物的高度来判断障碍物的距离 来决定是不是需要避障
                if(target.type == "obstacles\r" ){ 
                    obstacles->push_back(target);
                    if (target.rois[0].rect.height > 300){ //[TODO] 改阈值
                    car_state = 4;
                    }
                }
                
            }

            std::sort(global_path->begin(), global_path->end(),
              [](const auto& a, const auto& b) {
                  return a.rois[0].rect.y_offset > b.rois[0].rect.y_offset;
              });
            std::sort(obstacles->begin(), obstacles->end(),
              [](const auto& a, const auto& b) {

                  return a.rois[0].rect.height * a.rois[0].rect.width > b.rois[0].rect.height * b.rois[0].rect.width;
              });
        }
        rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr yoloResultSub;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr signSub;

};

#endif