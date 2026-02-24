#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <fstream>
#include <cmath>
#include <limits>

void runDebugMode(std::shared_ptr<rclcpp::Node> node) {
    RCLCPP_INFO(node->get_logger(), "=== ENTERING DEBUG MODE ===");
    
    // Initialize arm controller
    ArmController armController(node);

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

    // Additional Debug Actions
    RCLCPP_INFO(node->get_logger(), "=== TESTING YOLO DETECTION ===");
    YoloInterface yolo(node);
    RCLCPP_INFO(node->get_logger(), "Testing OAK-D Camera...");
    std::string oakd_obj = yolo.getObjectName(CameraSource::OAKD, false);
    if (!oakd_obj.empty()) {
        RCLCPP_INFO(node->get_logger(), "OAK-D detected: %s", oakd_obj.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "OAK-D detected nothing.");
    }

    RCLCPP_INFO(node->get_logger(), "Testing Wrist Camera...");
    std::string wrist_obj = yolo.getObjectName(CameraSource::WRIST, false);
    if (!wrist_obj.empty()) {
        RCLCPP_INFO(node->get_logger(), "Wrist camera detected: %s", wrist_obj.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "Wrist camera detected nothing.");
    }

    RCLCPP_INFO(node->get_logger(), "=== TESTING APRILTAG DETECTION ===");
    AprilTagDetector tag_detector(node);
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

    RCLCPP_INFO(node->get_logger(), "=== DEBUG MODE COMPLETE ===");
}

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

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
        static int boxes_visited = 0;
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
            
            // 1. Detect and pick up manipulable object
            RCLCPP_INFO(node->get_logger(), "Detecting manipulable object...");
            manipulable_object_name = yolo.getObjectName(CameraSource::WRIST, true);
            
            if (!manipulable_object_name.empty()) {
                RCLCPP_INFO(node->get_logger(), "Detected manipulable object: %s. Proceeding to pick it up.", manipulable_object_name.c_str());
                
                // Pick up object (assuming coordinates are provided or fixed for top plate)
                RCLCPP_INFO(node->get_logger(), "Opening gripper...");
                arm.openGripper();
                
                RCLCPP_INFO(node->get_logger(), "Moving arm to top plate...");
                arm.moveToCartesianPose(0.2, 0.0, 0.2, 0.0, 1.57, 0.0); 
                
                RCLCPP_INFO(node->get_logger(), "Closing gripper to grasp object...");
                arm.closeGripper();
                
                RCLCPP_INFO(node->get_logger(), "Moving arm to carry position...");
                arm.moveToCartesianPose(0.1, 0.0, 0.3, 0.0, 0.0, 0.0);
                
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
                            
                            if (!visible_tags.empty()) {
                                int tag_id = visible_tags[0];
                                RCLCPP_INFO(node->get_logger(), "Found AprilTag %d on the bin. Initiating placement sequence...", tag_id);
                                
                                // Place object in bin
                                ArmController arm(node);
                                RCLCPP_INFO(node->get_logger(), "Moving arm to bin position...");
                                arm.moveToCartesianPose(0.3, 0.0, 0.1, 0.0, 1.57, 0.0);
                                
                                RCLCPP_INFO(node->get_logger(), "Opening gripper to release object...");
                                arm.openGripper();
                                
                                RCLCPP_INFO(node->get_logger(), "Moving arm back to carry position...");
                                arm.moveToCartesianPose(0.1, 0.0, 0.3, 0.0, 0.0, 0.0);
                                
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
