#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
    // --- Arm Poses (x, y, z, qx, qy, qz, qw) --- QUATERNION format
    // To get these values: move the arm in RViz/MoveIt, then run:
    //   ros2 topic echo /arm_moveit/display_planned_path --once
    // or read the end-effector pose from MoveIt's planning scene.

    // Pose to look at the object on the top plate before picking it up
    const float ARM_LOOK_X = 0.15;
    const float ARM_LOOK_Y = 0.0;
    const float ARM_LOOK_Z = 0.25;
    const float ARM_LOOK_QX = 0.0;
    const float ARM_LOOK_QY = 0.4794;   // ~1.0 rad pitch downward
    const float ARM_LOOK_QZ = 0.0;
    const float ARM_LOOK_QW = 0.8776;

    // Pose to pick up the manipulable object from the top plate
    const float ARM_PICKUP_X = 0.2;
    const float ARM_PICKUP_Y = 0.0;
    const float ARM_PICKUP_Z = 0.2;
    const float ARM_PICKUP_QX = 0.0;
    const float ARM_PICKUP_QY = 0.7071;  // ~90° pitch (pointing down)
    const float ARM_PICKUP_QZ = 0.0;
    const float ARM_PICKUP_QW = 0.7071;
    
    // Pose to carry the object while navigating
    const float ARM_CARRY_X = 0.1;
    const float ARM_CARRY_Y = 0.0;
    const float ARM_CARRY_Z = 0.3;
    const float ARM_CARRY_QX = 0.0;
    const float ARM_CARRY_QY = 0.0;      // Upright / no rotation
    const float ARM_CARRY_QZ = 0.0;
    const float ARM_CARRY_QW = 1.0;
    
    // Pose to drop the object into the bin
    const float ARM_DROP_X = 0.3;
    const float ARM_DROP_Y = 0.0;
    const float ARM_DROP_Z = 0.1;
    const float ARM_DROP_QX = 0.0;
    const float ARM_DROP_QY = 0.7071;    // ~90° pitch (pointing down)
    const float ARM_DROP_QZ = 0.0;
    const float ARM_DROP_QW = 0.7071;

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

    // --- Spin velocity for in-place rotation (rad/s) ---
    const double SPIN_ANGULAR_VEL = 0.5;
    
    // Duration per spin step (seconds): SPIN_INCREMENT / SPIN_ANGULAR_VEL
    const double SPIN_STEP_DURATION = SPIN_INCREMENT / SPIN_ANGULAR_VEL;

    // --- AprilTag Candidate IDs ---
    const std::vector<int> CANDIDATE_TAG_IDS = {0, 1, 2, 3, 4, 5};

    // --- YOLO Detection Parameters ---
    // Number of retry attempts for YOLO detection (reduces false negatives)
    const int YOLO_MAX_RETRIES = 3;
    // Delay between retries (seconds)
    const double YOLO_RETRY_DELAY = 1.0;

    // --- Scene Object Search Spin Parameters ---
    // Small spin steps to search for the scene object if initial detection fails
    const int SCENE_SPIN_STEPS = 4;
    // ~30 degrees per step
    const float SCENE_SPIN_INCREMENT = M_PI / 6.0;
    const double SCENE_SPIN_DURATION = SCENE_SPIN_INCREMENT / SPIN_ANGULAR_VEL;
}
// =============================================================================

void runDebugMode(std::shared_ptr<rclcpp::Node> node) {
    RCLCPP_INFO(node->get_logger(), "=== ENTERING DEBUG MODE ===");
    
    // Initialize controllers
    ArmController armController(node);
    RCLCPP_INFO(node->get_logger(), "ArmController initialized successfully.");
    YoloInterface yolo(node);
    RCLCPP_INFO(node->get_logger(), "yolo initialized successfully.");
    AprilTagDetector tag_detector(node);
    RCLCPP_INFO(node->get_logger(), "tag_detector initialized successfully.");

    while (rclcpp::ok()) {
        std::cout << "\n========================================\n";
        std::cout << "Select a hardware component to test:\n";
        std::cout << "1. Test Arm Control\n";
        std::cout << "2. Test OAK-D Camera (YOLO)\n";
        std::cout << "3. Test Wrist Camera (YOLO)\n";
        std::cout << "4. Test AprilTag Detection\n";
        std::cout << "5. Test Everything in One Shot\n";
        std::cout << "6. Test Config Arm Poses (LOOK/PICKUP/CARRY/DROP)\n";
        std::cout << "0. Exit Debug Mode\n";
        std::cout << "Enter your choice (0-6): ";
        
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
        bool test_config_poses = (choice == 6);

        if (test_arm) {
            RCLCPP_INFO(node->get_logger(), "=== TESTING ARM CONTROL ===");
            RCLCPP_INFO(node->get_logger(), "Moving arm to a reachable pose...");
            armController.openGripper();
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

        if (test_config_poses) {
            RCLCPP_INFO(node->get_logger(), "=== TESTING CONFIG ARM POSES (LOOK/PICKUP/CARRY/DROP) ===");

            RCLCPP_INFO(node->get_logger(), "--- Testing LOOK pose ---");
            bool s = armController.moveToCartesianPose(Config::ARM_LOOK_X, Config::ARM_LOOK_Y, Config::ARM_LOOK_Z,
                                                       Config::ARM_LOOK_QX, Config::ARM_LOOK_QY, Config::ARM_LOOK_QZ, Config::ARM_LOOK_QW);
            RCLCPP_INFO(node->get_logger(), "LOOK pose: %s", s ? "SUCCESS" : "FAILED");
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Testing PICKUP pose ---");
            armController.openGripper();
            s = armController.moveToCartesianPose(Config::ARM_PICKUP_X, Config::ARM_PICKUP_Y, Config::ARM_PICKUP_Z,
                                                   Config::ARM_PICKUP_QX, Config::ARM_PICKUP_QY, Config::ARM_PICKUP_QZ, Config::ARM_PICKUP_QW);
            RCLCPP_INFO(node->get_logger(), "PICKUP pose: %s", s ? "SUCCESS" : "FAILED");
            armController.closeGripper();
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Testing CARRY pose ---");
            s = armController.moveToCartesianPose(Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                                                   Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
            RCLCPP_INFO(node->get_logger(), "CARRY pose: %s", s ? "SUCCESS" : "FAILED");
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Testing DROP pose ---");
            s = armController.moveToCartesianPose(Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z,
                                                   Config::ARM_DROP_QX, Config::ARM_DROP_QY, Config::ARM_DROP_QZ, Config::ARM_DROP_QW);
            RCLCPP_INFO(node->get_logger(), "DROP pose: %s", s ? "SUCCESS" : "FAILED");
            armController.openGripper();
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Returning to CARRY pose ---");
            armController.moveToCartesianPose(Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                                              Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
            RCLCPP_INFO(node->get_logger(), "=== CONFIG POSE TEST COMPLETE ===");
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
            auto visible_tags = tag_detector.getVisibleTags(Config::CANDIDATE_TAG_IDS);
            if (!visible_tags.empty()) {
                RCLCPP_INFO(node->get_logger(), "Detected %zu AprilTag(s):", visible_tags.size());
                for (int tag : visible_tags) {
                    auto pose_opt = tag_detector.getTagPose(tag);
                    if (pose_opt.has_value()) {
                        auto& p = pose_opt.value();
                        float dist = std::sqrt(p.position.x * p.position.x +
                                               p.position.y * p.position.y +
                                               p.position.z * p.position.z);
                        float angle = std::atan2(p.position.y, p.position.x) * 180.0 / M_PI;
                        RCLCPP_INFO(node->get_logger(),
                            "  Tag ID: %d | Distance: %.3fm | Angle: %.1f° | "
                            "Position: (x=%.3f, y=%.3f, z=%.3f)",
                            tag, dist, angle,
                            p.position.x, p.position.y, p.position.z);
                    } else {
                        RCLCPP_INFO(node->get_logger(), "  Tag ID: %d (visible but pose expired)", tag);
                    }
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "No AprilTags detected.");
                RCLCPP_WARN(node->get_logger(), "  Checklist:");
                RCLCPP_WARN(node->get_logger(), "  - Is apriltag_ros running? (ros2 launch mie443_contest2 apriltag_oakd.launch.py)");
                RCLCPP_WARN(node->get_logger(), "  - Is the OAK-D camera streaming? (ros2 topic hz /oakd/rgb/preview/image_raw)");
                RCLCPP_WARN(node->get_logger(), "  - Is a 36h11 AprilTag (ID 0-5) in view of the OAK-D camera?");
            }
        }

        if (choice < 0 || choice > 6) {
            std::cout << "Invalid choice. Please try again.\n";
        }
    }

    RCLCPP_INFO(node->get_logger(), "=== DEBUG MODE COMPLETE ===");
}

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Spin the node in a background thread. This is absolutely required for MoveIt!
    // Without this, the MoveGroupInterface constructor will deadlock waiting for the action server.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() {
        executor.spin();
    });
    spinner_thread.detach();

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

        // Declare the kinematics parameters for the 'arm' planning group
        // so that MoveIt knows which Inverse Kinematics (IK) solver to use.
        node->declare_parameter("robot_description_kinematics.arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.005);
        node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_timeout", 0.005);
        node->declare_parameter("robot_description_kinematics.arm.position_only_ik", false);
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

    RCLCPP_INFO(node->get_logger(), "============================================");
    RCLCPP_INFO(node->get_logger(), "         CONTEST 2 - MAIN MODE");
    RCLCPP_INFO(node->get_logger(), "============================================");

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // FIX: cmd_vel publisher for in-place spinning (much faster than Nav2 goals)
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Initialize box coordinates
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Loaded %zu box coordinates:", boxes.coords.size());
    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "  Box %zu: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // Wait for AMCL to publish a valid initial pose
    RCLCPP_INFO(node->get_logger(), "Waiting for AMCL pose...");
    {
        auto wait_start = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - wait_start).count();
            if (elapsed > 2) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // FIX: Save starting position so we can return here (not hardcoded 0,0,0)
    float start_x = robotPose.x;
    float start_y = robotPose.y;
    float start_phi = robotPose.phi;
    RCLCPP_INFO(node->get_logger(), "Starting position saved: x=%.2f, y=%.2f, phi=%.2f",
                start_x, start_y, start_phi);

    // FIX: Create all persistent objects ONCE (not per-loop iteration)
    ArmController arm(node);
    RCLCPP_INFO(node->get_logger(), "ArmController initialized.");
    YoloInterface yolo(node);
    RCLCPP_INFO(node->get_logger(), "YoloInterface initialized.");
    Navigation nav(node);
    RCLCPP_INFO(node->get_logger(), "Navigation initialized.");
    AprilTagDetector tag_detector(node);
    RCLCPP_INFO(node->get_logger(), "AprilTagDetector initialized.");

    // State tracking (regular variables, not static)
    bool initialized = false;
    std::string manipulable_object_name = "";
    std::vector<std::pair<std::string, std::vector<float>>> detected_scene_objects;
    size_t boxes_visited = 0;
    std::vector<bool> visited_boxes(boxes.coords.size(), false);
    bool manipulable_picked = false;
    bool object_placed = false;
    bool returning_home = false;
    bool finished = false;

    // Contest countdown timer
    auto start_time = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "============================================");
    RCLCPP_INFO(node->get_logger(), "  Starting contest - 300 seconds begins NOW");
    RCLCPP_INFO(node->get_logger(), "============================================");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        if (finished) {
            break;
        }

        // =====================================================================
        // PHASE 1: Detect and pick up manipulable object from top plate
        // =====================================================================
        if (!initialized) {
            RCLCPP_INFO(node->get_logger(), "=== PHASE 1: Detect & Pick Up Manipulable Object ===");
            
            // Move arm to LOOK pose so wrist camera can see the top plate
            RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to LOOK pose...", secondsElapsed);
            arm.moveToCartesianPose(Config::ARM_LOOK_X, Config::ARM_LOOK_Y, Config::ARM_LOOK_Z,
                                    Config::ARM_LOOK_QX, Config::ARM_LOOK_QY, Config::ARM_LOOK_QZ, Config::ARM_LOOK_QW);
            
            // Give the camera a moment to stabilize
            RCLCPP_INFO(node->get_logger(), "[%lus] Waiting for wrist camera to stabilize...", secondsElapsed);
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Detect manipulable object using wrist camera (with retries)
            RCLCPP_INFO(node->get_logger(), "[%lus] Running YOLO detection on wrist camera...", secondsElapsed);
            for (int attempt = 1; attempt <= Config::YOLO_MAX_RETRIES; ++attempt) {
                manipulable_object_name = yolo.getObjectName(CameraSource::WRIST, true);
                if (!manipulable_object_name.empty()) {
                    RCLCPP_INFO(node->get_logger(), "[%lus] Detected on attempt %d/%d",
                                secondsElapsed, attempt, Config::YOLO_MAX_RETRIES);
                    break;
                }
                if (attempt < Config::YOLO_MAX_RETRIES) {
                    RCLCPP_WARN(node->get_logger(), "[%lus] YOLO attempt %d/%d returned empty. Retrying...",
                                secondsElapsed, attempt, Config::YOLO_MAX_RETRIES);
                    std::this_thread::sleep_for(std::chrono::duration<double>(Config::YOLO_RETRY_DELAY));
                }
            }
            
            if (!manipulable_object_name.empty()) {
                RCLCPP_INFO(node->get_logger(), "[%lus] *** Detected manipulable object: '%s' ***", secondsElapsed, manipulable_object_name.c_str());
                
                // Pick up sequence
                RCLCPP_INFO(node->get_logger(), "[%lus] Opening gripper...", secondsElapsed);
                arm.openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to PICKUP pose...", secondsElapsed);
                arm.moveToCartesianPose(Config::ARM_PICKUP_X, Config::ARM_PICKUP_Y, Config::ARM_PICKUP_Z,
                                        Config::ARM_PICKUP_QX, Config::ARM_PICKUP_QY, Config::ARM_PICKUP_QZ, Config::ARM_PICKUP_QW);
                
                RCLCPP_INFO(node->get_logger(), "[%lus] Closing gripper to grasp object...", secondsElapsed);
                arm.closeGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to CARRY pose...", secondsElapsed);
                arm.moveToCartesianPose(Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                                        Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
                
                RCLCPP_INFO(node->get_logger(), "[%lus] *** Object picked up successfully! ***", secondsElapsed);
                manipulable_picked = true;
            } else {
                RCLCPP_WARN(node->get_logger(), "[%lus] WARNING: Failed to detect manipulable object!", secondsElapsed);
                RCLCPP_WARN(node->get_logger(), "[%lus] Proceeding to explore without object.", secondsElapsed);
                manipulable_picked = false;
            }
            
            initialized = true;
            continue;
        }

        // =====================================================================
        // PHASE 2: Navigate to each box, detect scene objects, match & drop
        // =====================================================================
        if (boxes_visited < boxes.coords.size()) {
            // Greedy Nearest Neighbor: find closest unvisited box
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

            if (current_box_index == -1) {
                RCLCPP_ERROR(node->get_logger(), "No unvisited box found! Logic error.");
                boxes_visited = boxes.coords.size();
                continue;
            }

            // FIX: Mark visited and increment BEFORE any continue statements
            visited_boxes[current_box_index] = true;
            boxes_visited++;

            float x = boxes.coords[current_box_index][0];
            float y = boxes.coords[current_box_index][1];
            float phi = boxes.coords[current_box_index][2];

            RCLCPP_INFO(node->get_logger(), "==================================================");
            RCLCPP_INFO(node->get_logger(), "=== PHASE 2: Box %d (%zu/%zu) ===",
                        current_box_index, boxes_visited, boxes.coords.size());
            RCLCPP_INFO(node->get_logger(), "[%lus] Target: x=%.2f, y=%.2f, phi=%.2f (Dist: %.2fm)",
                        secondsElapsed, x, y, phi, min_dist);
            
            if (!nav.moveToGoal(x, y, phi)) {
                RCLCPP_WARN(node->get_logger(), "[%lus] FAILED to reach box %d. Skipping.", secondsElapsed, current_box_index);
                continue;
            }
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Arrived at box %d. Detecting scene object with OAK-D...", secondsElapsed, current_box_index);
            
            // Detect scene object using the front-facing OAK-D camera (with retries)
            std::string scene_object_name;
            for (int attempt = 1; attempt <= Config::YOLO_MAX_RETRIES; ++attempt) {
                scene_object_name = yolo.getObjectName(CameraSource::OAKD, false);
                if (!scene_object_name.empty()) break;
                if (attempt < Config::YOLO_MAX_RETRIES) {
                    RCLCPP_WARN(node->get_logger(), "[%lus] OAK-D attempt %d/%d empty. Retrying...",
                                secondsElapsed, attempt, Config::YOLO_MAX_RETRIES);
                    std::this_thread::sleep_for(std::chrono::duration<double>(Config::YOLO_RETRY_DELAY));
                }
            }
            
            // If still nothing, spin in small steps and try again
            if (scene_object_name.empty()) {
                RCLCPP_WARN(node->get_logger(), "[%lus] No scene object after %d retries. Spinning to search...",
                            secondsElapsed, Config::YOLO_MAX_RETRIES);
                
                for (int spin_step = 1; spin_step <= Config::SCENE_SPIN_STEPS; ++spin_step) {
                    // Rotate ~30 degrees
                    geometry_msgs::msg::Twist twist;
                    twist.angular.z = Config::SPIN_ANGULAR_VEL;
                    
                    auto spin_start = std::chrono::steady_clock::now();
                    while (std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - spin_start).count()
                           < Config::SCENE_SPIN_DURATION) {
                        cmd_vel_pub->publish(twist);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    
                    // Stop and settle
                    geometry_msgs::msg::Twist stop;
                    cmd_vel_pub->publish(stop);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    
                    // Try YOLO again
                    scene_object_name = yolo.getObjectName(CameraSource::OAKD, false);
                    if (!scene_object_name.empty()) {
                        RCLCPP_INFO(node->get_logger(), "[%lus] Scene object found after spin step %d!",
                                    secondsElapsed, spin_step);
                        break;
                    }
                }
            }
            
            if (scene_object_name.empty()) {
                RCLCPP_WARN(node->get_logger(), "[%lus] No scene object detected at box %d.", secondsElapsed, current_box_index);
                continue;
            }
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Detected scene object: '%s' at box %d",
                        secondsElapsed, scene_object_name.c_str(), current_box_index);
            detected_scene_objects.push_back({scene_object_name, {x, y, phi}});
            
            // Check if we should attempt to place the object
            if (!manipulable_picked || object_placed || manipulable_object_name.empty()) {
                RCLCPP_INFO(node->get_logger(), "[%lus] Not carrying object or already placed. Moving on.", secondsElapsed);
                continue;
            }
            
            if (scene_object_name != manipulable_object_name) {
                RCLCPP_INFO(node->get_logger(), "[%lus] '%s' != '%s'. No match. Moving on.",
                            secondsElapsed, scene_object_name.c_str(), manipulable_object_name.c_str());
                continue;
            }
            
            // *** MATCH FOUND ***
            RCLCPP_INFO(node->get_logger(), "==================================================");
            RCLCPP_INFO(node->get_logger(), "*** MATCH FOUND! '%s' == '%s' ***",
                        scene_object_name.c_str(), manipulable_object_name.c_str());
            RCLCPP_INFO(node->get_logger(), "[%lus] Searching for AprilTag on the bin...", secondsElapsed);
            
            auto visible_tags = tag_detector.getVisibleTags(Config::CANDIDATE_TAG_IDS);
            
            // FIX: If not visible, spin using cmd_vel (much faster than Nav2 goals)
            if (visible_tags.empty()) {
                RCLCPP_WARN(node->get_logger(), "[%lus] AprilTag not visible. Spinning to search...", secondsElapsed);
                
                for (int spin_step = 1; spin_step <= Config::SPIN_STEPS; ++spin_step) {
                    RCLCPP_INFO(node->get_logger(), "[%lus] Spin step %d/%d (%.0f deg total)...",
                                secondsElapsed, spin_step, Config::SPIN_STEPS,
                                spin_step * Config::SPIN_INCREMENT * 180.0 / M_PI);
                    
                    // Publish angular velocity to rotate in place
                    geometry_msgs::msg::Twist twist;
                    twist.angular.z = Config::SPIN_ANGULAR_VEL;
                    
                    auto spin_start = std::chrono::steady_clock::now();
                    while (std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - spin_start).count()
                           < Config::SPIN_STEP_DURATION) {
                        cmd_vel_pub->publish(twist);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    
                    // Stop the robot
                    geometry_msgs::msg::Twist stop;
                    cmd_vel_pub->publish(stop);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // settle
                    
                    // Check for AprilTag
                    visible_tags = tag_detector.getVisibleTags(Config::CANDIDATE_TAG_IDS);
                    if (!visible_tags.empty()) {
                        RCLCPP_INFO(node->get_logger(), "[%lus] AprilTag found after spin step %d!", secondsElapsed, spin_step);
                        break;
                    }
                }
            }
            
            if (visible_tags.empty()) {
                RCLCPP_WARN(node->get_logger(), "[%lus] No AprilTag found after full 360 spin! Cannot place object.", secondsElapsed);
                continue;
            }
            
            int tag_id = visible_tags[0];
            RCLCPP_INFO(node->get_logger(), "[%lus] Found AprilTag ID=%d. Aligning with bin...", secondsElapsed, tag_id);
            
            // Use AprilTag pose to align the robot precisely
            auto tag_pose_opt = tag_detector.getTagPose(tag_id);
            if (tag_pose_opt.has_value()) {
                geometry_msgs::msg::Pose tag_pose = tag_pose_opt.value();
                
                float dist_to_tag = std::sqrt(tag_pose.position.x * tag_pose.position.x +
                                               tag_pose.position.y * tag_pose.position.y);
                float angle_to_tag = std::atan2(tag_pose.position.y, tag_pose.position.x);
                
                RCLCPP_INFO(node->get_logger(), "[%lus] Tag distance: %.3fm, angle: %.1f deg",
                            secondsElapsed, dist_to_tag, angle_to_tag * 180.0 / M_PI);
                
                // Adjust position if needed
                if (std::abs(dist_to_tag - Config::DESIRED_DROP_DISTANCE) > Config::DISTANCE_TOLERANCE ||
                    std::abs(angle_to_tag) > Config::ANGLE_TOLERANCE) {
                    
                    float move_dist = dist_to_tag - Config::DESIRED_DROP_DISTANCE;
                    float new_x = robotPose.x + move_dist * std::cos(robotPose.phi + angle_to_tag);
                    float new_y = robotPose.y + move_dist * std::sin(robotPose.phi + angle_to_tag);
                    float new_phi = robotPose.phi + angle_to_tag;
                    while (new_phi > M_PI) new_phi -= 2.0 * M_PI;
                    while (new_phi < -M_PI) new_phi += 2.0 * M_PI;
                    
                    RCLCPP_INFO(node->get_logger(), "[%lus] Adjusting to: x=%.2f, y=%.2f, phi=%.2f",
                                secondsElapsed, new_x, new_y, new_phi);
                    nav.moveToGoal(new_x, new_y, new_phi);
                } else {
                    RCLCPP_INFO(node->get_logger(), "[%lus] Already aligned. No adjustment needed.", secondsElapsed);
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "[%lus] Could not get tag pose. Dropping from current position.", secondsElapsed);
            }
            
            // === DROP SEQUENCE ===
            RCLCPP_INFO(node->get_logger(), "[%lus] === DROPPING OBJECT INTO BIN ===", secondsElapsed);
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to DROP pose...", secondsElapsed);
            arm.moveToCartesianPose(Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z,
                                    Config::ARM_DROP_QX, Config::ARM_DROP_QY, Config::ARM_DROP_QZ, Config::ARM_DROP_QW);
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Opening gripper to release object...", secondsElapsed);
            arm.openGripper();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Returning arm to CARRY pose...", secondsElapsed);
            arm.moveToCartesianPose(Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                                    Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
            
            RCLCPP_INFO(node->get_logger(), "[%lus] *** Object placed in bin successfully! ***", secondsElapsed);
            object_placed = true;
            
            continue;
        }

        // =====================================================================
        // PHASE 3: Return home and save results
        // =====================================================================
        if (!returning_home) {
            RCLCPP_INFO(node->get_logger(), "==================================================");
            RCLCPP_INFO(node->get_logger(), "=== PHASE 3: Returning Home ===");
            // FIX: Return to saved starting position, not (0,0,0)
            RCLCPP_INFO(node->get_logger(), "[%lus] All %zu boxes visited. Returning to start (%.2f, %.2f, %.2f)...",
                        secondsElapsed, boxes.coords.size(), start_x, start_y, start_phi);
            
            if (nav.moveToGoal(start_x, start_y, start_phi)) {
                RCLCPP_INFO(node->get_logger(), "[%lus] Successfully returned to start.", secondsElapsed);
            } else {
                RCLCPP_WARN(node->get_logger(), "[%lus] Failed to return to start.", secondsElapsed);
            }
            returning_home = true;
            
            // Output results to file
            RCLCPP_INFO(node->get_logger(), "[%lus] Writing results to contest2_results.txt...", secondsElapsed);
            std::ofstream outfile("contest2_results.txt");
            if (outfile.is_open()) {
                outfile << "=== CONTEST 2 RESULTS ===\n";
                outfile << "Manipulable Object: " << (manipulable_object_name.empty() ? "NOT DETECTED" : manipulable_object_name) << "\n";
                outfile << "Object Placed: " << (object_placed ? "YES" : "NO") << "\n";
                outfile << "Boxes Visited: " << boxes_visited << "/" << boxes.coords.size() << "\n";
                outfile << "\nDetected Scene Objects:\n";
                for (size_t i = 0; i < detected_scene_objects.size(); ++i) {
                    outfile << "  " << (i + 1) << ". Object: " << detected_scene_objects[i].first
                            << " at (x=" << detected_scene_objects[i].second[0]
                            << ", y=" << detected_scene_objects[i].second[1]
                            << ", phi=" << detected_scene_objects[i].second[2] << ")\n";
                }
                outfile << "\nTime Elapsed: " << secondsElapsed << " seconds\n";
                outfile.close();
                RCLCPP_INFO(node->get_logger(), "[%lus] Results saved successfully.", secondsElapsed);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to open results file for writing.");
            }
            
            finished = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "============================================");
        RCLCPP_WARN(node->get_logger(), "  CONTEST TIME LIMIT REACHED (300s)!");
        RCLCPP_WARN(node->get_logger(), "============================================");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down. Total time: %lu seconds.", secondsElapsed);
    rclcpp::shutdown();
    return 0;
}
