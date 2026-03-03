#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <fstream>
#include <cmath>
#include <limits>
#include <iostream>

// =============================================================================
// CONTEST CONFIGURATION PARAMETERS
// Adjust these values during physical testing on the real robot.
// =============================================================================
namespace Config {
    // --- Arm Poses (x, y, z, roll, pitch, yaw) ---
    // Pose to look at the object on the top plate before picking it up
    const float ARM_LOOK_X = 0.15;
    const float ARM_LOOK_Y = 0.0;
    const float ARM_LOOK_Z = 0.25;
    const float ARM_LOOK_ROLL = 0.0;
    const float ARM_LOOK_PITCH = 1.0; // Angled down slightly to view the plate
    const float ARM_LOOK_YAW = 0.0;

    // Pose to pick up the manipulable object from the top plate
    const float ARM_PICKUP_X = 0.2;
    const float ARM_PICKUP_Y = 0.0;
    const float ARM_PICKUP_Z = 0.2;
    const float ARM_PICKUP_ROLL = 0.0;
    const float ARM_PICKUP_PITCH = 1.57;
    const float ARM_PICKUP_YAW = 0.0;
    
    // Pose to carry the object while navigating
    const float ARM_CARRY_X = 0.1;
    const float ARM_CARRY_Y = 0.0;
    const float ARM_CARRY_Z = 0.3;
    const float ARM_CARRY_ROLL = 0.0;
    const float ARM_CARRY_PITCH = 0.0;
    const float ARM_CARRY_YAW = 0.0;
    
    // Pose to drop the object into the bin
    const float ARM_DROP_X = 0.3;
    const float ARM_DROP_Y = 0.0;
    const float ARM_DROP_Z = 0.1;
    const float ARM_DROP_ROLL = 0.0;
    const float ARM_DROP_PITCH = 1.57;
    const float ARM_DROP_YAW = 0.0;

    // --- Alignment Parameters ---
    // Desired distance from the AprilTag to safely drop the object (in meters)
    const float DESIRED_DROP_DISTANCE = 0.4;
    
    // Tolerance for distance alignment (in meters)
    const float DISTANCE_TOLERANCE = 0.05;
    
    // Tolerance for angle alignment (in radians)
    const float ANGLE_TOLERANCE = 0.1;

    // --- Search Spin Parameters ---
    // Number of steps for a full 360-degree spin
    const int SPIN_STEPS = 8;
    
    // Angle increment per spin step (in radians)
    const float SPIN_INCREMENT = M_PI / 4.0;
}
// =============================================================================

void runDebugMode(std::shared_ptr<rclcpp::Node> node) {
    RCLCPP_INFO(node->get_logger(), "=== ENTERING DEBUG MODE ===");
    
    // Initialize controllers
    ArmController armController(node);
    YoloInterface yolo(node);
    AprilTagDetector tag_detector(node);

    while (rclcpp::ok()) {
        std::cout << "\n========================================\n";
        std::cout << "Select a hardware component to test:\n";
        std::cout << "1. Test Arm Control\n";
        std::cout << "2. Test OAK-D Camera (YOLO)\n";
        std::cout << "3. Test Wrist Camera (YOLO)\n";
        std::cout << "4. Test AprilTag Detection\n";
        std::cout << "5. Test Everything in One Shot\n";
        std::cout << "0. Exit Debug Mode\n";
        std::cout << "Enter your choice (0-5): ";
        
        int choice;
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (choice == 0) {
            RCLCPP_INFO(node->get_logger(), "Exiting Debug Mode.");
            break;
        }

        bool test_arm = (choice == 1 || choice == 5);
        bool test_oakd = (choice == 2 || choice == 5);
        bool test_wrist = (choice == 3 || choice == 5);
        bool test_apriltag = (choice == 4 || choice == 5);

        if (test_arm) {
            RCLCPP_INFO(node->get_logger(), "=== TESTING ARM CONTROL ===");
            RCLCPP_INFO(node->get_logger(), "Moving arm to a reachable pose...");
            
            // Test arm movement with pose 1
            bool success = armController.moveToCartesianPose(0.043, 0.199, 0.313,
                                                             -0.471, -0.557, 0.564, -0.387);

            if(success) {
                RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
                // Test gripper
                RCLCPP_INFO(node->get_logger(), "Testing gripper...");
                armController.openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                armController.closeGripper();
                std::this_thread::sleep_for(std::chrono::seconds(2));
            } else {
                RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose may be unreachable");
            }

            // Test arm movement with pose 2
            success = armController.moveToCartesianPose(0.142, -0.064, 0.400,
                                                        -0.418, 0.844, 0.238, -0.237);

            if(success) {
                RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose may be unreachable");
            }
        }

        if (test_oakd) {
            RCLCPP_INFO(node->get_logger(), "=== TESTING OAK-D CAMERA (YOLO) ===");
            // Changed to true to save the annotated image to disk
            std::string oakd_obj = yolo.getObjectName(CameraSource::OAKD, true);
            if (!oakd_obj.empty()) {
                RCLCPP_INFO(node->get_logger(), "OAK-D detected: %s (Image saved as detected_object.jpg)", oakd_obj.c_str());
            } else {
                RCLCPP_WARN(node->get_logger(), "OAK-D detected nothing.");
            }
        }

        if (test_wrist) {
            RCLCPP_INFO(node->get_logger(), "=== TESTING WRIST CAMERA (YOLO) ===");
            // Changed to true to save the annotated image to disk
            std::string wrist_obj = yolo.getObjectName(CameraSource::WRIST, true);
            if (!wrist_obj.empty()) {
                RCLCPP_INFO(node->get_logger(), "Wrist camera detected: %s (Image saved as detected_object.jpg)", wrist_obj.c_str());
            } else {
                RCLCPP_WARN(node->get_logger(), "Wrist camera detected nothing.");
            }
        }

        if (test_apriltag) {
            RCLCPP_INFO(node->get_logger(), "=== TESTING APRILTAG DETECTION ===");
            std::vector<int> candidate_tags = {0, 1, 2, 3, 4, 5};
            auto visible_tags = tag_detector.getVisibleTags(candidate_tags);
            if (!visible_tags.empty()) {
                RCLCPP_INFO(node->get_logger(), "Detected AprilTags: ");
                for (int tag : visible_tags) {
                    RCLCPP_INFO(node->get_logger(), "Tag ID: %d", tag);
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "No AprilTags detected.");
            }
        }

        if (choice < 0 || choice > 5) {
            std::cout << "Invalid choice. Please try again.\n";
        }
    }

    RCLCPP_INFO(node->get_logger(), "=== DEBUG MODE COMPLETE ===");
}

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters so that
    // MoveGroupInterface builds the SO-ARM101 model
    {
        std::string desc_dir = ament_index_cpp::get_package_share_directory("lerobot_description");
        std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
        if (urdf_file.is_open()) {
            std::stringstream ss;
            ss << urdf_file.rdbuf();
            node->declare_parameter("robot_description", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
        }

        std::string moveit_dir = ament_index_cpp::get_package_share_directory("lerobot_moveit");
        std::ifstream srdf_file(moveit_dir + "/config/so101.srdf");
        if (srdf_file.is_open()) {
            std::stringstream ss;
            ss << srdf_file.rdbuf();
            node->declare_parameter("robot_description_semantic", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm SRDF file");
        }
    }

    // Check for debug flag
    bool debug_mode = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--debug") {
            debug_mode = true;
            break;
        }
    }

    if (debug_mode) {
        runDebugMode(node);
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // Initialize box coordinates
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }
    

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/
        static bool initialized = false;
        static std::string manipulable_object_name = "";
        static std::vector<std::pair<std::string, std::vector<float>>> detected_scene_objects;
        static size_t boxes_visited = 0;
        static std::vector<bool> visited_boxes(boxes.coords.size(), false);
        static bool manipulable_picked = false;
        static bool returning_home = false;
        static bool finished = false;

        if (finished) {
            break;
        }

        if (!initialized) {
            // Initialize components
            ArmController arm(node);
            YoloInterface yolo(node);
            
            // 0. Move arm to a position where the wrist camera can see the top plate
            RCLCPP_INFO(node->get_logger(), "Moving arm to look at the manipulable object...");
            arm.moveToCartesianPose(Config::ARM_LOOK_X, Config::ARM_LOOK_Y, Config::ARM_LOOK_Z, 
                                    Config::ARM_LOOK_ROLL, Config::ARM_LOOK_PITCH, Config::ARM_LOOK_YAW);
            
            // Give the camera a moment to stabilize and capture a clear frame
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // 1. Detect and pick up manipulable object
            RCLCPP_INFO(node->get_logger(), "Detecting manipulable object...");
            manipulable_object_name = yolo.getObjectName(CameraSource::WRIST, true);
            
            if (!manipulable_object_name.empty()) {
                RCLCPP_INFO(node->get_logger(), "Detected manipulable object: %s. Proceeding to pick it up.", manipulable_object_name.c_str());
                
                // Pick up object (assuming coordinates are provided or fixed for top plate)
                RCLCPP_INFO(node->get_logger(), "Opening gripper...");
                arm.openGripper();
                
                RCLCPP_INFO(node->get_logger(), "Moving arm to top plate...");
                arm.moveToCartesianPose(Config::ARM_PICKUP_X, Config::ARM_PICKUP_Y, Config::ARM_PICKUP_Z, 
                                        Config::ARM_PICKUP_ROLL, Config::ARM_PICKUP_PITCH, Config::ARM_PICKUP_YAW); 
                
                RCLCPP_INFO(node->get_logger(), "Closing gripper to grasp object...");
                arm.closeGripper();
                
                RCLCPP_INFO(node->get_logger(), "Moving arm to carry position...");
                arm.moveToCartesianPose(Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z, 
                                        Config::ARM_CARRY_ROLL, Config::ARM_CARRY_PITCH, Config::ARM_CARRY_YAW);
                
                RCLCPP_INFO(node->get_logger(), "Successfully picked up manipulable object.");
                manipulable_picked = true;
            } else {
                RCLCPP_WARN(node->get_logger(), "Failed to detect manipulable object!");
                // Even if failed, we should proceed to explore the environment
                RCLCPP_INFO(node->get_logger(), "Proceeding to explore the environment without the object.");
                manipulable_picked = true; 
            }
            
            initialized = true;
            continue;
        }

        if (manipulable_picked && boxes_visited < boxes.coords.size()) {
            Navigation nav(node);
            YoloInterface yolo(node);
            AprilTagDetector tag_detector(node);

            // OPTIMIZATION: Find the nearest unvisited box (Greedy Nearest Neighbor)
            int current_box_index = -1;
            float min_dist = std::numeric_limits<float>::max();
            for (size_t i = 0; i < boxes.coords.size(); ++i) {
                if (!visited_boxes[i]) {
                    float dx = boxes.coords[i][0] - robotPose.x;
                    float dy = boxes.coords[i][1] - robotPose.y;
                    float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < min_dist) {
                        min_dist = dist;
                        current_box_index = i;
                    }
                }
            }

            if (current_box_index != -1) {
                visited_boxes[current_box_index] = true; // Mark as visited

                float x = boxes.coords[current_box_index][0];
                float y = boxes.coords[current_box_index][1];
                float phi = boxes.coords[current_box_index][2];

                RCLCPP_INFO(node->get_logger(), "--------------------------------------------------");
                RCLCPP_INFO(node->get_logger(), "Navigating to nearest box %d: x=%.2f, y=%.2f, phi=%.2f (Distance: %.2fm)", current_box_index, x, y, phi, min_dist);
                
                if (nav.moveToGoal(x, y, phi)) {
                    RCLCPP_INFO(node->get_logger(), "Successfully reached box %d. Initiating scene object detection...", current_box_index);
                    
                    // Detect scene object
                    std::string scene_object_name = yolo.getObjectName(CameraSource::OAKD, false);
                    if (!scene_object_name.empty()) {
                        RCLCPP_INFO(node->get_logger(), "Detected scene object: %s at box %d", scene_object_name.c_str(), current_box_index);
                        detected_scene_objects.push_back({scene_object_name, {x, y, phi}});
                        
                        // Check if it matches manipulable object
                        if (!manipulable_object_name.empty() && scene_object_name == manipulable_object_name) {
                            RCLCPP_INFO(node->get_logger(), "Match found! Scene object '%s' matches manipulable object. Looking for AprilTag...", scene_object_name.c_str());
                            
                            // Look for AprilTag (assuming IDs 0-5 for bins)
                            std::vector<int> candidate_tags = {0, 1, 2, 3, 4, 5};
                            auto visible_tags = tag_detector.getVisibleTags(candidate_tags);
                            
                            // OPTIMIZATION: If AprilTag is not immediately visible, spin to find it
                            if (visible_tags.empty()) {
                                RCLCPP_WARN(node->get_logger(), "AprilTag not immediately visible. Initiating search spin...");
                                
                                // Simple spin behavior: rotate in place by sending small navigation goals
                                // or by publishing to cmd_vel directly. Since we are using Nav2, we can 
                                // send a new goal with the same x, y but a different phi (yaw).
                                
                                for (int spin_step = 1; spin_step <= Config::SPIN_STEPS; ++spin_step) {
                                    float new_phi = phi + (spin_step * Config::SPIN_INCREMENT); // Rotate
                                    // Normalize phi to [-PI, PI]
                                    while (new_phi > M_PI) new_phi -= 2.0 * M_PI;
                                    while (new_phi < -M_PI) new_phi += 2.0 * M_PI;
                                    
                                    RCLCPP_INFO(node->get_logger(), "Spinning to phi=%.2f to search for AprilTag...", new_phi);
                                    nav.moveToGoal(x, y, new_phi);
                                    
                                    // Check again
                                    visible_tags = tag_detector.getVisibleTags(candidate_tags);
                                    if (!visible_tags.empty()) {
                                        RCLCPP_INFO(node->get_logger(), "AprilTag found after spinning!");
                                        break;
                                    }
                                }
                            }

                            if (!visible_tags.empty()) {
                                int tag_id = visible_tags[0];
                                RCLCPP_INFO(node->get_logger(), "Found AprilTag %d on the bin. Initiating placement sequence...", tag_id);
                                
                                // OPTIMIZATION: Use AprilTag pose to adjust robot position before dropping
                                auto tag_pose_opt = tag_detector.getTagPose(tag_id);
                                if (tag_pose_opt.has_value()) {
                                    geometry_msgs::msg::Pose tag_pose = tag_pose_opt.value();
                                    
                                    // tag_pose is relative to the robot's base_link.
                                    // We want to move the robot so it is exactly a certain distance (e.g., 0.4m) 
                                    // directly in front of the tag.
                                    
                                    // Calculate the distance and angle to the tag
                                    float dist_to_tag = std::sqrt(tag_pose.position.x * tag_pose.position.x + tag_pose.position.y * tag_pose.position.y);
                                    float angle_to_tag = std::atan2(tag_pose.position.y, tag_pose.position.x);
                                    
                                    RCLCPP_INFO(node->get_logger(), "AprilTag is %.2fm away at angle %.2f rad.", dist_to_tag, angle_to_tag);
                                    
                                    // Desired distance from the bin to safely drop the object
                                    float desired_distance = Config::DESIRED_DROP_DISTANCE; 
                                    
                                    // If we are too far or not facing it directly, adjust position
                                    if (std::abs(dist_to_tag - desired_distance) > Config::DISTANCE_TOLERANCE || std::abs(angle_to_tag) > Config::ANGLE_TOLERANCE) {
                                        RCLCPP_INFO(node->get_logger(), "Adjusting robot position to align with the bin...");
                                        
                                        // Calculate new goal in the map frame
                                        // We need to move forward/backward by (dist_to_tag - desired_distance)
                                        // and rotate by angle_to_tag
                                        
                                        float move_dist = dist_to_tag - desired_distance;
                                        
                                        // Calculate new map coordinates based on current robot pose
                                        float new_x = robotPose.x + move_dist * std::cos(robotPose.phi + angle_to_tag);
                                        float new_y = robotPose.y + move_dist * std::sin(robotPose.phi + angle_to_tag);
                                        float new_phi = robotPose.phi + angle_to_tag;
                                        
                                        // Normalize phi
                                        while (new_phi > M_PI) new_phi -= 2.0 * M_PI;
                                        while (new_phi < -M_PI) new_phi += 2.0 * M_PI;
                                        
                                        RCLCPP_INFO(node->get_logger(), "Moving to aligned drop position: x=%.2f, y=%.2f, phi=%.2f", new_x, new_y, new_phi);
                                        nav.moveToGoal(new_x, new_y, new_phi);
                                    } else {
                                        RCLCPP_INFO(node->get_logger(), "Robot is already perfectly aligned with the bin.");
                                    }
                                } else {
                                    RCLCPP_WARN(node->get_logger(), "Could not get precise pose of AprilTag. Proceeding with current position.");
                                }

                                // Place object in bin
                                // Note: The arm coordinates below are placeholders. 
                                // You must calibrate these values on the real robot so the arm reaches 
                                // forward enough to drop the object into the bin without hitting it.
                                ArmController arm(node);
                                RCLCPP_INFO(node->get_logger(), "Moving arm to bin position...");
                                arm.moveToCartesianPose(Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z, 
                                                        Config::ARM_DROP_ROLL, Config::ARM_DROP_PITCH, Config::ARM_DROP_YAW);
                                
                                RCLCPP_INFO(node->get_logger(), "Opening gripper to release object...");
                                arm.openGripper();
                                
                                RCLCPP_INFO(node->get_logger(), "Moving arm back to carry position...");
                                arm.moveToCartesianPose(Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z, 
                                                        Config::ARM_CARRY_ROLL, Config::ARM_CARRY_PITCH, Config::ARM_CARRY_YAW);
                                
                                RCLCPP_INFO(node->get_logger(), "Object successfully placed in the bin.");
                                manipulable_picked = false; // Object placed
                            } else {
                                RCLCPP_WARN(node->get_logger(), "No AprilTag found at matching object location! Cannot place object.");
                            }
                        } else {
                            RCLCPP_INFO(node->get_logger(), "Scene object '%s' does not match manipulable object '%s'. Moving on.", scene_object_name.c_str(), manipulable_object_name.c_str());
                        }
                    } else {
                        RCLCPP_WARN(node->get_logger(), "Failed to detect scene object at box %d", current_box_index);
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(), "Failed to reach box %d. Navigation aborted or blocked.", current_box_index);
                }
                
                boxes_visited++;
            }
        } else if (!returning_home && boxes_visited >= boxes.coords.size()) {
            // Return to start (0, 0, 0)
            RCLCPP_INFO(node->get_logger(), "--------------------------------------------------");
            RCLCPP_INFO(node->get_logger(), "All boxes visited. Returning to start (0, 0, 0)...");
            Navigation nav(node);
            if (nav.moveToGoal(0.0, 0.0, 0.0)) {
                RCLCPP_INFO(node->get_logger(), "Successfully returned to start.");
            } else {
                RCLCPP_WARN(node->get_logger(), "Failed to return to start.");
            }
            returning_home = true;
            
            // Output results to file
            std::ofstream outfile("contest2_results.txt");
            if (outfile.is_open()) {
                outfile << "Manipulable Object: " << manipulable_object_name << "\n";
                outfile << "Detected Scene Objects:\n";
                for (size_t i = 0; i < detected_scene_objects.size(); ++i) {
                    outfile << "Object: " << detected_scene_objects[i].first 
                            << " at Location: x=" << detected_scene_objects[i].second[0] 
                            << ", y=" << detected_scene_objects[i].second[1] 
                            << ", phi=" << detected_scene_objects[i].second[2] << "\n";
                }
                outfile.close();
                RCLCPP_INFO(node->get_logger(), "Results saved to contest2_results.txt");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to open results file for writing.");
            }
            
            finished = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
