
#include "usv/pid_algorithm/differential_drive.hpp"

DifferentialDrive::DifferentialDrive()
: Node("differential_drive")
{   
    auto qos = rclcpp::QoS(1).best_effort();

    _change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "differential_guidance/change_state");
    
    _get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(
        "differential_guidance/get_state");

    _vehicle_status_sub =
        this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status",
            qos,
            std::bind(
                &DifferentialDrive::vehicleStatusCallback, this, std::placeholders::_1
            )
        );

}


void DifferentialDrive::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus &msg)
{

    bool armed = (msg.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
     
    if(armed) {
        _change_arm_mode = true;
        switch (msg.nav_state) {
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_FREE5: // For physical PX4
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: // For QGC simulation
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1: // Activate Slalom mode
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL3: // Activate Docking mode
                if (_change_nav_mode) {

                    sendGetStateRequest();

                    this->_delay_timer = 
                        this->create_wall_timer(
                            std::chrono::milliseconds(1000),
                            [this]() {
                                if (_current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                                    RCLCPP_INFO(this->get_logger(), "Unconfigured state. Changing to inactive.");
                                    sendChangeStateRequest(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                                    _change_nav_mode = true; //Switch from inactive to active state (run only after unconfigured state)
                                }
                                else if (_current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                                    RCLCPP_INFO(this->get_logger(), "Inactive state. Changing to active.");
                                    sendChangeStateRequest(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                                }
                                this->_delay_timer->cancel();
                            }
                        );

                    _change_nav_mode = false;
                }

                break;
            case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                if (!_change_nav_mode) {

                    sendGetStateRequest();

                    this->_delay_timer = 
                        this->create_wall_timer(
                            std::chrono::milliseconds(1000),
                            [this]() {
                                if (_current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                                    RCLCPP_INFO(this->get_logger(), "Active state. Changing to inactive.");
                                    sendChangeStateRequest(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                                }

                                this->_delay_timer->cancel();
                            }
                        );

                    _change_nav_mode = true;
                }
                
                break;
        }
    }
    else if (!armed && _change_arm_mode){
        sendGetStateRequest();

        this->_delay_timer = 
            this->create_wall_timer(
                std::chrono::milliseconds(1000),
                [this]() {
                    if (_current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                        RCLCPP_INFO(this->get_logger(), "Inactive state. Changing to unconfigured.");
                        sendChangeStateRequest(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
                    }
                    else if (_current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                        RCLCPP_INFO(this->get_logger(), "Active state. Changing to inactive.");
                        sendChangeStateRequest(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                        _change_nav_mode = false;
                    }
                    this->_delay_timer->cancel();
                }
            );

        _change_arm_mode = false;
    }
}

/************** Lifecycle requests *****************/

void DifferentialDrive::sendChangeStateRequest(const uint8_t transition_id)
{
    auto change_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_request->transition.id = transition_id;

    auto change_state_future = _change_state_client->async_send_request(change_request,
        std::bind(&DifferentialDrive::stateChangeCallback, this, std::placeholders::_1));
}

void DifferentialDrive::sendGetStateRequest()
{
    auto get_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto get_state_future = _get_state_client->async_send_request(get_request,
        std::bind(&DifferentialDrive::getStateCallback, this, std::placeholders::_1));
}



/************** Lifecycle request callbacks *****************/
void DifferentialDrive::stateChangeCallback(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
{
    auto result = future.get();
    if (result->success) {
        RCLCPP_INFO(this->get_logger(), "State transition complete.");
        _req_transition_success = true;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "State transition failed.");
    }
}

void DifferentialDrive::getStateCallback(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
{
    auto result = future.get();
    if (!result) {
        RCLCPP_INFO(this->get_logger(), "Failed to fetch current state.");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Successfully fetched current state.");
        _current_state = result->current_state.id;
        _req_state_success = true;
    }

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDrive>());
    rclcpp::shutdown();
    return 0;
}
