//
// Simple example to demonstrate how to use the MAVSDK.
//
// Author: Julian Oes <julian@oes.ch>

#include <chrono>
#include <cstdint>
#include <cmath>
#include <future>
#include <string>

#include <iostream>	//
#include <fstream>	//


#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>



#include <iostream>
#include <thread>


using namespace mavsdk;
using namespace std::this_thread;
using namespace std::chrono;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour
//**********************************************************************************
    int forward = 3;
    int reverse = 4;
    int left = 5;
    int right = 6;
    int climb = 7;
    int decline = 8;
    int hawk_mode = 9;
    int nothing = 0;

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string& message)
{
    if (result != Action::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string& message)
{
    if (result != Offboard::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string& message)
{
    if (result != ConnectionResult::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string& offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}
//**********************************************************************************

/**
 * Does Offboard control using body co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log
 * otherwise.
 */
bool offb_ctrl_body(mavsdk::Offboard& offboard, int direction)
{
    const std::string offb_mode = "BODY";

    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityBodyYawspeed stay{};
    offboard.set_velocity_body(stay);

    Offboard::Result offboard_result = offboard.start();
    offboard_error_exit(offboard_result, "Offboard start failed: ");
    offboard_log(offb_mode, "Offboard started");

    if(direction == climb){
        offboard_log(offb_mode, "Climb");
        Offboard::VelocityBodyYawspeed climb{};
        climb.down_m_s = -1.0f;
        offboard.set_velocity_body(climb);
        sleep_for(seconds(1));
        climb.down_m_s = 0.0f;
        offboard.set_velocity_body(climb);
    }
    else if(direction == forward)
    {
        offboard_log(offb_mode, "Forward");
        Offboard::VelocityBodyYawspeed forward{};
        forward.forward_m_s = 1.0f;
        offboard.set_velocity_body(forward);
        sleep_for(seconds(1));
        forward.forward_m_s = 0.0f;
        offboard.set_velocity_body(forward);
    }
        else if(direction == reverse)
    {
        offboard_log(offb_mode, "Reverse");
        Offboard::VelocityBodyYawspeed reverse{};
        reverse.forward_m_s = -1.0f;
        offboard.set_velocity_body(reverse);
        sleep_for(seconds(1));
        reverse.forward_m_s = 0.0f;
        offboard.set_velocity_body(reverse);
    }
        else if(direction == left)
    {
        offboard_log(offb_mode, "Left");
        Offboard::VelocityBodyYawspeed left{};
        left.right_m_s = -1.0f;
        offboard.set_velocity_body(left);
        sleep_for(seconds(1));
        left.right_m_s = 0.0f;
        offboard.set_velocity_body(left);
    }
        else if(direction == right)
    {
        offboard_log(offb_mode, "Right");
        Offboard::VelocityBodyYawspeed right{};
        right.right_m_s = 1.0f;
        offboard.set_velocity_body(right);
        sleep_for(seconds(1));
        right.right_m_s = 0.0f;
        offboard.set_velocity_body(right);
    }
    else if(direction == decline)
    {
        offboard_log(offb_mode, "Decline");
        Offboard::VelocityBodyYawspeed decline{};
        decline.down_m_s = 1.0f;
        offboard.set_velocity_body(decline);
        sleep_for(seconds(1));
        decline.down_m_s = 0.0f;
        offboard.set_velocity_body(decline);

    }
    else if(direction == hawk_mode)
    {
        offboard_log(offb_mode, "Hawk mode iniciated in: ");
        sleep_for(seconds(1));
        offboard_log(offb_mode, "3");
        sleep_for(seconds(1));
        offboard_log(offb_mode, "2");
        sleep_for(seconds(1));
        offboard_log(offb_mode, "1");
        sleep_for(seconds(1));


        offboard_log(offb_mode, "Climb");
        Offboard::VelocityBodyYawspeed climb{};
        climb.down_m_s = -5.0f;
        offboard.set_velocity_body(climb);
        sleep_for(seconds(3));
        climb.down_m_s = 0.0f;
        offboard.set_velocity_body(climb);

   	offboard_log(offb_mode, "Rotate left(anti-clockwise for 1 second)");
    	Offboard::VelocityBodyYawspeed ccw{};
    	ccw.yawspeed_deg_s = -30.0f; // 30 degree a second
   	offboard.set_velocity_body(ccw);
   	sleep_for(seconds(3));//second
   	ccw.yawspeed_deg_s = 0.0f; // stop turning
   	offboard.set_velocity_body(ccw);

        offboard_log(offb_mode, "Forward");
        Offboard::VelocityBodyYawspeed forward{};
        forward.forward_m_s = 1.0f;
        offboard.set_velocity_body(forward);
        sleep_for(seconds(2));
        forward.forward_m_s = 0.0f;
        offboard.set_velocity_body(forward);
	
	offboard_log(offb_mode, "Left");
        Offboard::VelocityBodyYawspeed left{};
        left.right_m_s = -1.0f;
        offboard.set_velocity_body(left);
        sleep_for(seconds(2));
        left.right_m_s = 0.0f;
        offboard.set_velocity_body(left);

        offboard_log(offb_mode, "Forward");
        //Offboard::VelocityBodyYawspeed forward{};
        forward.forward_m_s = 1.0f;
        offboard.set_velocity_body(forward);
        sleep_for(seconds(2));
        forward.forward_m_s = 0.0f;
        offboard.set_velocity_body(forward);
	
        offboard_log(offb_mode, "Right");
        Offboard::VelocityBodyYawspeed right{};
        right.right_m_s = 1.0f;
        offboard.set_velocity_body(right);
        sleep_for(seconds(4));
        right.right_m_s = 0.0f;
        offboard.set_velocity_body(right);

        offboard_log(offb_mode, "Reverse");
        Offboard::VelocityBodyYawspeed reverse{};
        reverse.forward_m_s = -1.0f;
        offboard.set_velocity_body(reverse);
        sleep_for(seconds(2));
        reverse.forward_m_s = 0.0f;
        offboard.set_velocity_body(reverse);
	
	offboard_log(offb_mode, "Left");
        //Offboard::VelocityBodyYawspeed left{};
        left.right_m_s = -1.0f;
        offboard.set_velocity_body(left);
        sleep_for(seconds(2));
        left.right_m_s = 0.0f;
        offboard.set_velocity_body(left);

        offboard_log(offb_mode, "Reverse");
        //Offboard::VelocityBodyYawspeed reverse{};
        reverse.forward_m_s = -1.0f;
        offboard.set_velocity_body(reverse);
        sleep_for(seconds(2));
        reverse.forward_m_s = 0.0f;
        offboard.set_velocity_body(reverse);
	
   	offboard_log(offb_mode, "Rotate right(clockwise for 1 second)");
    //	Offboard::VelocityBodyYawspeed ccw{};
    	ccw.yawspeed_deg_s = 30.0f; // 30 degree a second
   	offboard.set_velocity_body(ccw);
   	sleep_for(seconds(3));//second
   	ccw.yawspeed_deg_s = 0.0f; // stop turning
   	offboard.set_velocity_body(ccw);

        offboard_log(offb_mode, "Decline");
       // Offboard::VelocityBodyYawspeed climb{};
        climb.down_m_s = 5.0f;
        offboard.set_velocity_body(climb);
        sleep_for(seconds(3));
        climb.down_m_s = 0.0f;
        offboard.set_velocity_body(climb);

    }
    else if(direction == nothing){

    }


    return true;
}
//***************************

void wait_until_discover(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system..." << std::endl;
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &discover_promise]() {
        const auto system = mavsdk.systems().at(0);

        if (system->is_connected()) {
            std::cout << "Discovered system" << std::endl;
            discover_promise.set_value();
        }
    });

    discover_future.wait();
}

//**************
void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}
//***************************

Telemetry::LandedStateCallback
landed_state_callback(Telemetry& telemetry, std::promise<void>& landed_promise)
{
    return [&landed_promise, &telemetry](Telemetry::LandedState landed) {
        switch (landed) {
            case Telemetry::LandedState::OnGround:
                std::cout << "On ground" << std::endl;
                break;
            case Telemetry::LandedState::TakingOff:
                std::cout << "Taking off..." << std::endl;
                break;
            case Telemetry::LandedState::Landing:
                std::cout << "Landing..." << std::endl;
                break;
            case Telemetry::LandedState::InAir:
                std::cout << "Taking off has finished." << std::endl;
                telemetry.subscribe_landed_state(nullptr);
                landed_promise.set_value();
                break;
            case Telemetry::LandedState::Unknown:
                std::cout << "Unknown landed state." << std::endl;
                break;
        }
    };
}
//***************************

void component_discovered(ComponentType component_type)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Discovered a component with type "
              << unsigned(component_type) << std::endl;
}

int main(int argc, char** argv)
{
    Mavsdk mavsdk;
    std::string connection_url;
    ConnectionResult connection_result;
    bool discovered_system = false;


    //forward, reverse, left, right - depending on body
    int argc1 = 1;// takeoff
    int argc2 = 2;// land
    int argc3 = 3;// forward
    int argc4 = 4;// reverse
    int argc5 = 5;// left
    int argc6 = 6;// right
    int argc7 = 7;// climb
    int argc8 = 8;// decline
    int argc9 = 9;// HAWK_MODE!!!

    int count = 30;
    int i = 0;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = mavsdk.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

        // Wait for the system to connect via heartbeat
    wait_until_discover(mavsdk);

    // System got discovered.
    auto system = mavsdk.systems().at(0);
    auto action = Action{system};
    auto offboard = Offboard{system};
    auto telemetry = Telemetry{system};

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(seconds(2));
/*
    if (!discovered_system) {
        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }
*/

    std::promise<void> in_air_promise;
    auto in_air_future = in_air_promise.get_future();


    //const auto system = mavsdk.systems().at(0);

    // Register a callback so we get told when components (camera, gimbal) etc
    // are found.
    system->register_component_discovered_callback(component_discovered);



    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Setting rate failed:" << set_rate_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                  << "Altitude: " << position.relative_altitude_m << " m"
                  << NORMAL_CONSOLE_TEXT // set to default color again
                  << std::endl;
    });

    // Check if vehicle is ready to arm
    while (telemetry.health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << arm_result << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }
        //  using body co-ordinates
    bool ret = offb_ctrl_body(offboard,nothing);
   std::string command;
   std::string holdCommand = "HOLD";
	bool flag_takeoff = false;
 	std::string line_;
    while(1){
	command.replace(command.begin(),command.end(),holdCommand);       
	sleep_for(seconds(1));
 	std::ifstream file_("pythonToCppCommand.txt");//this line opens the text file	

        //std::cin >> command;    
	// if(file_.is_open()) // check if it really is open
	 {
	     while(getline(file_,line_))
	     {
            	//std::cout << line_ << std::endl;
	  	command.replace(command.begin(),command.end(),line_);       
	     }
	     file_.close(); // close file
	 }

        std::cout << command << std::endl;


        if(command ==  "TAKEOFF" && !flag_takeoff){// Take off   
		flag_takeoff = true;
            std::cout << "Taking off and hover and flag_takeoff = true" << std::endl;
            const Action::Result takeoff_result = action.takeoff();
            if (takeoff_result != Action::Result::Success) {
                std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << takeoff_result
                        << NORMAL_CONSOLE_TEXT << std::endl;
                return 1;
            }
            telemetry.subscribe_landed_state(landed_state_callback(telemetry, in_air_promise));
            in_air_future.wait();
        } else if(command == "LAND"){// Landing
                        // Let it hover for a bit before landing again.
            sleep_for(seconds(5));
            std::cout << "Landing..." << std::endl;
            const Action::Result land_result = action.land();
            if (land_result != Action::Result::Success) {
                std::cout << ERROR_CONSOLE_TEXT << "Land failed:" << land_result << 		  			NORMAL_CONSOLE_TEXT
                        << std::endl;
                return 1;
            }

        } else if(command == "FORWARD"){// Forward
            //  using body co-ordinates
            ret = offb_ctrl_body(offboard, forward);
            if (ret == false) {
                return EXIT_FAILURE;
            }  
        } else if(command == "REVERSE"){// Reverse
                    //  using body co-ordinates
            ret = offb_ctrl_body(offboard, reverse);
            if (ret == false) {
                return EXIT_FAILURE;
            }  
            
        } else if(command == "LEFT"){// Left
                    //  using body co-ordinates
            ret = offb_ctrl_body(offboard, left);
            if (ret == false) {
                return EXIT_FAILURE;
            }  
            
        } else if(command ==  "RIGHT"){// Right
                    //  using body co-ordinates
            ret = offb_ctrl_body(offboard, right);
            if (ret == false) {
                return EXIT_FAILURE;
            }
        } else if(command== "CLIMB"){// Climb
                //  using body co-ordinates
        ret = offb_ctrl_body(offboard, climb);
        if (ret == false) {
            return EXIT_FAILURE;
            } 
        } else if(command == "DECLINE"){// Decline
                //  using body co-ordinates
        ret = offb_ctrl_body(offboard, decline);
        if (ret == false) {
            return EXIT_FAILURE;
            }  
        } else if(command == "HAWK_MODE"){// HAWK_MODE!!!
                //  using body co-ordinates
        ret = offb_ctrl_body(offboard, hawk_mode);

	//After HawkMode return to launch
	const Action::Result rtl_result = action.return_to_launch();
	if (rtl_result != Action::Result::Success) {
	    //RTL failed, so exit (in reality might send kill command.)
	    return 1;
	}
	if (ret == false) {
	    return EXIT_FAILURE;
	    }  
		break;

          }  
        
    }


    const Action::Result land_result = action.land();
    action_error_exit(land_result, "Landing failed");

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return 0;
}
