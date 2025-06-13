#ifndef RACECAR
#define RACECAR
#include "rclcpp/rclcpp.hpp"
#include "/root/dev_ws/src/racecar/include/racecar/State.hpp"
#include "/root/dev_ws/src/racecar/include/racecar/MsgCenter.hpp"
class Racecar : public rclcpp::Node
{
public:
    Racecar() : Node("racecar_context"), msg_center(std::make_shared<MsgCenter>())
    {
        std::cout << "racecar_context" << std::endl;
    }

    std::shared_ptr<MsgCenter> msg_center;
    std::shared_ptr<State> current_state;
    void run()
    {
        while (true)
        {
            switch (msg_center->car_state)
            {
            case 1:
                this->current_state = std::make_shared<FirstState>(msg_center);
                break;
            case 2:
                this->current_state = std::make_shared<SecondState>(msg_center);
                break;
            case 3:
                this->current_state = std::make_shared<ThirdState>(msg_center);
                break;
            case 4:
                this->current_state = std::make_shared<FourthState>(msg_center);
                break;

            default:
                break;
            }
            current_state->run(30);
        }
    }
};

#endif