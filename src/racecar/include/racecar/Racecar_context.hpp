#ifndef RACECAR
#define RACECAR
#include "rclcpp/rclcpp.hpp"
#include "/root/dev_ws/src/racecar/include/racecar/State.hpp"
#include "/root/dev_ws/src/racecar/include/racecar/MsgCenter.hpp"
class Racecar : public rclcpp::Node{
    public:
        Racecar() : Node("racecar_context"),msg_center(std::make_shared<MsgCenter>())
        {
            std::cout<<"racecar_context"<<std::endl;
        }
        
        std::shared_ptr<MsgCenter> msg_center;
        std::shared_ptr<State> current_state;
        void run(){
            this->current_state = std::make_shared<FirstState>(msg_center); 
            current_state->run(30);
            this->current_state = std::make_shared<SecondState>(msg_center); 
            current_state->run(30);
            this->current_state = std::make_shared<ThirdState>(msg_center); 
            current_state->run(30);
        }
};

#endif