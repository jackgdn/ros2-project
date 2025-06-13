#ifndef STATE
#define STATE
#include "rclcpp/rclcpp.hpp"
#include "/root/dev_ws/src/racecar/include/racecar/Racecar_context.hpp"
#include "/root/dev_ws/src/racecar/include/racecar/MsgCenter.hpp"
class State
{
public:
    std::shared_ptr<MsgCenter> msg_center;
    State(std::shared_ptr<MsgCenter> msg_center_) : msg_center(msg_center_)
    {
    }
    geometry_msgs::msg::Twist cmd_vel;
    double lwf_x, lwf_y;
    virtual void run(int rate = 30) = 0;
};
class FirstState : public State, rclcpp::Node
{
public:
    FirstState(std::shared_ptr<MsgCenter> msg_center_) : State(msg_center_), Node("first_state")
    {
        std::cout << "first_state" << std::endl;
    }
    void run(int rate = 30) override
    {
        rclcpp::WallRate loop_rate(rate);
        double p = 0.3;
        while (rclcpp::ok())
        {
            if (msg_center->car_state == 2)
                break;
            std::cout << "global path size: " << msg_center->global_path->size() << std::endl;
            std::cout << "obstacles size: " << msg_center->obstacles->size() << std::endl;
            if (!msg_center->global_path->empty())
            {
                lwf_x = msg_center->global_path->back().rois[0].rect.x_offset;
                lwf_y = msg_center->global_path->back().rois[0].rect.y_offset;
                double angle = -1 * atan2(lwf_x - 1920 / 2, 1080 - lwf_y);
                cmd_vel.linear.x = 0.3;
                cmd_vel.angular.z = angle * p;
            }
            msg_center->velPub->publish(cmd_vel);
            rclcpp::spin_some(msg_center);
            loop_rate.sleep();
        }
        return;
    }
};
class SecondState : public State, rclcpp::Node
{
public:
    SecondState(std::shared_ptr<MsgCenter> msg_center_) : State(msg_center_), Node("second_state")
    {
        std::cout << "second_state" << std::endl;
    }
    void run(int rate = 30) override
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        msg_center->velPub->publish(cmd_vel);
        rclcpp::WallRate loop_rate(rate);
        while (rclcpp::ok())
        {
            if (msg_center->car_state == 3)
                break;
            rclcpp::spin_some(msg_center);
            loop_rate.sleep();
        }
        return;
    }
};

class ThirdState : public State, rclcpp::Node
{
public:
    ThirdState(std::shared_ptr<MsgCenter> msg_center_) : State(msg_center_), Node("third_state")
    {
        std::cout << "third_state" << std::endl;
    }
    void run(int rate = 30) override
    {
        rclcpp::WallRate loop_rate(rate);
        double p = 0.3;
        while (rclcpp::ok())
        {
            std::cout << msg_center->global_path->size() << std::endl;
            if (!msg_center->global_path->empty())
            {
                lwf_x = msg_center->global_path->back().rois[0].rect.x_offset;
                lwf_y = msg_center->global_path->back().rois[0].rect.y_offset;
                double angle = -1 * atan2(lwf_x - 1920 / 2, 1080 - lwf_y);
                cmd_vel.linear.x = 0.5;
                cmd_vel.angular.z = angle * p;
            }
            msg_center->velPub->publish(cmd_vel);
            rclcpp::spin_some(msg_center);
            loop_rate.sleep();
        }
        return;
    }
};
#endif