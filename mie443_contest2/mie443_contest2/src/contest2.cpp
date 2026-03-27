#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <fstream>
#include <cmath>
#include <limits>
#include <iostream>
#include <memory>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <sys/select.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

// =============================================================================
// CONTEST CONFIGURATION PARAMETERS
// Loaded from arm_poses.yaml at runtime — edit the YAML, no rebuild needed!
// Search order: ./arm_poses.yaml → installed share path → hardcoded defaults.
// =============================================================================
namespace Config {
    // --- Arm Poses as JOINT VALUES (radians) ---
    std::vector<double> ARM_LOOK_JOINTS   = {-1.70983, -0.730833172, 0.719931, 1.6410469858, 1.46864237};
    std::vector<double> ARM_PICKUP_JOINTS = {-1.794406, -1.0182992, 1.1159992, 1.384434, 1.086024};
    std::vector<double> ARM_CARRY_JOINTS  = {-1.800044, -1.4320071, 0.929259, 1.6438824, 1.0511085};
    std::vector<double> ARM_DROP_JOINTS   = {-0.1395492, 0.8129663, -1.20920725, 1.20863287, 1.080205};

    // --- Arm Poses (x, y, z, qx, qy, qz, qw) --- QUATERNION format (fallback)
    float ARM_LOOK_X = 0.115049f;   float ARM_LOOK_Y = 0.00866053f;  float ARM_LOOK_Z = 0.182472f;
    float ARM_LOOK_QX = 0.0771996f; float ARM_LOOK_QY = 0.0283588f;  float ARM_LOOK_QZ = 0.996288f;  float ARM_LOOK_QW = 0.025429f;

    float ARM_PICKUP_X = 0.103496f;    float ARM_PICKUP_Y = 0.00683864f;  float ARM_PICKUP_Z = 0.154764f;
    float ARM_PICKUP_QX = 0.0100663f;  float ARM_PICKUP_QY = -0.0163231f; float ARM_PICKUP_QZ = -0.201975f; float ARM_PICKUP_QW = 0.979203f;

    float ARM_CARRY_X = 0.0636712f;  float ARM_CARRY_Y = -0.0981863f;  float ARM_CARRY_Z = 0.238941f;
    float ARM_CARRY_QX = 0.444479f;  float ARM_CARRY_QY = 0.864106f;   float ARM_CARRY_QZ = 0.128399f;  float ARM_CARRY_QW = -0.198173f;

    float ARM_DROP_X = 0.0243134f;   float ARM_DROP_Y = -0.242021f;    float ARM_DROP_Z = 0.259773f;
    float ARM_DROP_QX = -0.153172f;  float ARM_DROP_QY = -0.174677f;   float ARM_DROP_QZ = -0.720359f;  float ARM_DROP_QW = 0.653536f;

    // --- Alignment Parameters ---
    float DESIRED_DROP_DISTANCE = 0.1f;
    float DISTANCE_TOLERANCE = 0.05f;
    float ANGLE_TOLERANCE = 0.1f;

    // --- Search Spin Parameters ---
    const int SPIN_STEPS = 12;
    const float SPIN_INCREMENT = M_PI / 6.0;
    const double SPIN_ANGULAR_VEL = 0.5;
    const double SPIN_STEP_DURATION = SPIN_INCREMENT / SPIN_ANGULAR_VEL;

    // --- AprilTag Candidate IDs ---
    const std::vector<int> CANDIDATE_TAG_IDS = {0, 1, 2, 3, 4, 5};

    // --- YOLO Detection Parameters ---
    int YOLO_MAX_RETRIES = 4;
    double YOLO_RETRY_DELAY = 0.8;

    // --- Scene Object Search Spin Parameters ---
    // Sweep ±90° from arrival heading: 9 steps × 20° = 180° total
    int SCENE_SPIN_STEPS = 9;
    // 20 degrees per step
    float SCENE_SPIN_INCREMENT = M_PI / 9.0;
    double SCENE_SPIN_DURATION = SCENE_SPIN_INCREMENT / SPIN_ANGULAR_VEL;


    // --- Time Budget ---
    int TIME_RESERVE_SECONDS = 45;

    // --- Navigation Tuning (per-goal) ---
    double NAV_TIMEOUT_MAX = 45.0;          // seconds — max time per moveToGoal
    double ALIGNMENT_XY_TOLERANCE = 0.05;   // meters — tight tolerance for alignment moves
    double FORWARD_DRIVE_BEFORE_DROP = 0.20; // meters — fixed forward drive after facing bin, before dropping

    // Path to the YAML file that was actually loaded (empty = defaults)
    std::string loaded_yaml_path = "";
}

// Load arm poses from YAML file. Returns the path loaded, or "" if defaults used.
static std::string loadConfigFromYaml(rclcpp::Logger logger) {
    // Search order: CWD → installed share path
    std::vector<std::string> search_paths;
    search_paths.push_back("./arm_poses.yaml");
    try {
        std::string share = ament_index_cpp::get_package_share_directory("mie443_contest2");
        search_paths.push_back(share + "/config/arm_poses.yaml");
    } catch (...) {}

    std::string yaml_path;
    for (const auto& p : search_paths) {
        std::ifstream test(p);
        if (test.good()) {
            yaml_path = p;
            break;
        }
    }

    if (yaml_path.empty()) {
        RCLCPP_WARN(logger, "[Config] arm_poses.yaml not found. Using hardcoded defaults.");
        RCLCPP_WARN(logger, "[Config] Searched: %s", search_paths.front().c_str());
        return "";
    }

    try {
        YAML::Node cfg = YAML::LoadFile(yaml_path);
        RCLCPP_INFO(logger, "[Config] Loading arm poses from: %s", yaml_path.c_str());

        // Helper lambdas
        auto readVec = [&](const std::string& key, std::vector<double>& out) {
            if (cfg[key] && cfg[key].IsSequence()) {
                out.clear();
                for (const auto& v : cfg[key]) out.push_back(v.as<double>());
            }
        };
        auto readCartesian = [&](const std::string& key,
                                  float& x, float& y, float& z,
                                  float& qx, float& qy, float& qz, float& qw) {
            if (cfg[key] && cfg[key].IsSequence() && cfg[key].size() == 7) {
                x  = cfg[key][0].as<float>();
                y  = cfg[key][1].as<float>();
                z  = cfg[key][2].as<float>();
                qx = cfg[key][3].as<float>();
                qy = cfg[key][4].as<float>();
                qz = cfg[key][5].as<float>();
                qw = cfg[key][6].as<float>();
            }
        };
        auto readFloat = [&](const std::string& key, float& out) {
            if (cfg[key]) out = cfg[key].as<float>();
        };
        auto readInt = [&](const std::string& key, int& out) {
            if (cfg[key]) out = cfg[key].as<int>();
        };
        auto readDouble = [&](const std::string& key, double& out) {
            if (cfg[key]) out = cfg[key].as<double>();
        };

        // Joint targets
        readVec("arm_look_joints",   Config::ARM_LOOK_JOINTS);
        readVec("arm_pickup_joints", Config::ARM_PICKUP_JOINTS);
        readVec("arm_carry_joints",  Config::ARM_CARRY_JOINTS);
        readVec("arm_drop_joints",   Config::ARM_DROP_JOINTS);

        // Cartesian fallbacks
        readCartesian("arm_look_cartesian",
            Config::ARM_LOOK_X, Config::ARM_LOOK_Y, Config::ARM_LOOK_Z,
            Config::ARM_LOOK_QX, Config::ARM_LOOK_QY, Config::ARM_LOOK_QZ, Config::ARM_LOOK_QW);
        readCartesian("arm_pickup_cartesian",
            Config::ARM_PICKUP_X, Config::ARM_PICKUP_Y, Config::ARM_PICKUP_Z,
            Config::ARM_PICKUP_QX, Config::ARM_PICKUP_QY, Config::ARM_PICKUP_QZ, Config::ARM_PICKUP_QW);
        readCartesian("arm_carry_cartesian",
            Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
            Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
        readCartesian("arm_drop_cartesian",
            Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z,
            Config::ARM_DROP_QX, Config::ARM_DROP_QY, Config::ARM_DROP_QZ, Config::ARM_DROP_QW);

        // Scalar params
        readFloat("desired_drop_distance", Config::DESIRED_DROP_DISTANCE);
        readFloat("distance_tolerance",    Config::DISTANCE_TOLERANCE);
        readFloat("angle_tolerance",       Config::ANGLE_TOLERANCE);
        readInt("yolo_max_retries",        Config::YOLO_MAX_RETRIES);
        readDouble("yolo_retry_delay",     Config::YOLO_RETRY_DELAY);
        readInt("time_reserve_seconds",    Config::TIME_RESERVE_SECONDS);

        // Scene spin search parameters
        readInt("scene_spin_steps",        Config::SCENE_SPIN_STEPS);
        if (cfg["scene_spin_step_deg"]) {
            float deg = cfg["scene_spin_step_deg"].as<float>();
            Config::SCENE_SPIN_INCREMENT = deg * M_PI / 180.0f;
            Config::SCENE_SPIN_DURATION  = Config::SCENE_SPIN_INCREMENT / Config::SPIN_ANGULAR_VEL;
            RCLCPP_INFO(logger, "  SCENE_SPIN: %d steps × %.0f° = %.0f° sweep",
                        Config::SCENE_SPIN_STEPS, deg, Config::SCENE_SPIN_STEPS * deg);
        }

        // Log what was loaded
        auto logJoints = [&](const std::string& name, const std::vector<double>& j) {
            std::ostringstream ss;
            ss << "  " << name << ": [";
            for (size_t i = 0; i < j.size(); ++i) {
                if (i > 0) ss << ", ";
                ss << std::fixed << std::setprecision(6) << j[i];
            }
            ss << "]";
            RCLCPP_INFO(logger, "%s", ss.str().c_str());
        };
        logJoints("LOOK",   Config::ARM_LOOK_JOINTS);
        logJoints("PICKUP", Config::ARM_PICKUP_JOINTS);
        logJoints("CARRY",  Config::ARM_CARRY_JOINTS);
        logJoints("DROP",   Config::ARM_DROP_JOINTS);

        Config::loaded_yaml_path = yaml_path;
        return yaml_path;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "[Config] Failed to parse %s: %s", yaml_path.c_str(), e.what());
        RCLCPP_WARN(logger, "[Config] Using hardcoded defaults.");
        return "";
    }
}

// Save current joint values to the YAML file for a given pose name
static bool saveJointsToYaml(const std::string& pose_name,
                              const std::vector<double>& joints,
                              rclcpp::Logger logger) {
    // Determine which YAML file to write to
    std::string yaml_path = Config::loaded_yaml_path;
    if (yaml_path.empty()) {
        // Default to CWD
        yaml_path = "./arm_poses.yaml";
    }

    // Map pose name to YAML key
    std::string key;
    if (pose_name == "LOOK")        key = "arm_look_joints";
    else if (pose_name == "PICKUP") key = "arm_pickup_joints";
    else if (pose_name == "CARRY")  key = "arm_carry_joints";
    else if (pose_name == "DROP")   key = "arm_drop_joints";
    else {
        RCLCPP_ERROR(logger, "Unknown pose name: %s", pose_name.c_str());
        return false;
    }

    try {
        // Load existing YAML (or create new)
        YAML::Node cfg;
        std::ifstream test(yaml_path);
        if (test.good()) {
            test.close();
            cfg = YAML::LoadFile(yaml_path);
        }

        // Update the specific key
        cfg[key] = joints;

        // Write back
        std::ofstream out(yaml_path);
        if (!out.is_open()) {
            RCLCPP_ERROR(logger, "Cannot write to %s", yaml_path.c_str());
            return false;
        }
        out << "# MIE443 Contest 2 — Arm Pose Configuration\n";
        out << "# Auto-updated by debug mode. Edit freely — no rebuild needed!\n\n";
        out << cfg;
        out.close();

        RCLCPP_INFO(logger, "Saved %s joints to %s (key: %s)", pose_name.c_str(), yaml_path.c_str(), key.c_str());

        // Also update in-memory Config
        if (pose_name == "LOOK")        Config::ARM_LOOK_JOINTS = joints;
        else if (pose_name == "PICKUP") Config::ARM_PICKUP_JOINTS = joints;
        else if (pose_name == "CARRY")  Config::ARM_CARRY_JOINTS = joints;
        else if (pose_name == "DROP")   Config::ARM_DROP_JOINTS = joints;

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Failed to save YAML: %s", e.what());
        return false;
    }
}
// =============================================================================

// Helper: move arm to a known pose. Tries joint target first (fastest, bypasses
// IK), falls back to Cartesian target if joints are empty or fail.
static bool moveArmPose(ArmController& arm, rclcpp::Logger logger,
                        const std::vector<double>& joints,
                        double x, double y, double z,
                        double qx, double qy, double qz, double qw) {
    if (!joints.empty()) {
        RCLCPP_INFO(logger, "  Using joint-space target (%zu joints)...", joints.size());
        if (arm.moveToJointTarget(joints)) return true;
        RCLCPP_WARN(logger, "  Joint target failed, falling back to Cartesian...");
    }
    return arm.moveToCartesianPose(x, y, z, qx, qy, qz, qw);
}
// Overload that takes ArmController pointer (for main loop where arm is unique_ptr)
static bool moveArmPose(ArmController* arm, rclcpp::Logger logger,
                        const std::vector<double>& joints,
                        double x, double y, double z,
                        double qx, double qy, double qz, double qw) {
    return moveArmPose(*arm, logger, joints, x, y, z, qx, qy, qz, qw);
}

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
        std::cout << "7. Read Current Arm Pose (for calibrating Config values)\n";
        std::cout << "8. Interactive Pose Tuning (move arm, read, repeat)\n";
        std::cout << "9. Step-by-Step Pick & Place (LOOK→detect→PICKUP→CARRY→DROP)\n";
        std::cout << "10. Save Current Joints to arm_poses.yaml (LOOK/PICKUP/CARRY/DROP)\n";
        std::cout << "0. Exit Debug Mode\n";
        std::cout << "Enter your choice (0-10): ";
        
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
        bool read_current_pose = (choice == 7);
        bool interactive_tuning = (choice == 8);
        bool step_by_step = (choice == 9);

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
            bool s = moveArmPose(armController, node->get_logger(),
                                 Config::ARM_LOOK_JOINTS,
                                 Config::ARM_LOOK_X, Config::ARM_LOOK_Y, Config::ARM_LOOK_Z,
                                 Config::ARM_LOOK_QX, Config::ARM_LOOK_QY, Config::ARM_LOOK_QZ, Config::ARM_LOOK_QW);
            RCLCPP_INFO(node->get_logger(), "LOOK pose: %s", s ? "SUCCESS" : "FAILED");
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Testing PICKUP pose ---");
            armController.openGripper();
            s = moveArmPose(armController, node->get_logger(),
                            Config::ARM_PICKUP_JOINTS,
                            Config::ARM_PICKUP_X, Config::ARM_PICKUP_Y, Config::ARM_PICKUP_Z,
                            Config::ARM_PICKUP_QX, Config::ARM_PICKUP_QY, Config::ARM_PICKUP_QZ, Config::ARM_PICKUP_QW);
            RCLCPP_INFO(node->get_logger(), "PICKUP pose: %s", s ? "SUCCESS" : "FAILED");
            armController.closeGripper();
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Testing CARRY pose ---");
            s = moveArmPose(armController, node->get_logger(),
                            Config::ARM_CARRY_JOINTS,
                            Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                            Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
            RCLCPP_INFO(node->get_logger(), "CARRY pose: %s", s ? "SUCCESS" : "FAILED");
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Testing DROP pose ---");
            s = moveArmPose(armController, node->get_logger(),
                            Config::ARM_DROP_JOINTS,
                            Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z,
                            Config::ARM_DROP_QX, Config::ARM_DROP_QY, Config::ARM_DROP_QZ, Config::ARM_DROP_QW);
            RCLCPP_INFO(node->get_logger(), "DROP pose: %s", s ? "SUCCESS" : "FAILED");
            armController.openGripper();
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "--- Returning to CARRY pose ---");
            moveArmPose(armController, node->get_logger(),
                        Config::ARM_CARRY_JOINTS,
                        Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
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

        if (read_current_pose) {
            RCLCPP_INFO(node->get_logger(), "=== LIVE ARM POSE MONITOR ===");
            RCLCPP_INFO(node->get_logger(), "Planning frame: %s", armController.getPlanningFrame().c_str());
            RCLCPP_INFO(node->get_logger(), "End-effector link: %s", armController.getEndEffectorLink().c_str());
            RCLCPP_INFO(node->get_logger(), "Move the arm in RViz — values refresh every 0.5s.");
            RCLCPP_INFO(node->get_logger(), "Press ENTER to stop monitoring and print copy-paste values.\n");

            // ── Live monitoring loop ──
            // Refreshes every 0.5s until user presses Enter.
            // Use non-blocking stdin check so we keep updating.
            bool monitoring = true;
            while (monitoring && rclcpp::ok()) {
                auto pose_stamped = armController.getCurrentPose();
                auto& p = pose_stamped.pose;
                auto joints = armController.getCurrentJointValues();

                tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // Clear line and print compact summary (overwrites previous)
                std::cout << "\r\033[K"  // clear line
                          << "pos=(" << std::fixed << std::setprecision(4)
                          << p.position.x << ", " << p.position.y << ", " << p.position.z << ")  "
                          << "joints=[";
                for (size_t i = 0; i < joints.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << std::fixed << std::setprecision(2) << (joints[i] * 180.0 / M_PI) << "°";
                }
                std::cout << "]" << std::flush;

                // Non-blocking check for Enter key
                struct timeval tv = {0, 0};
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(STDIN_FILENO, &fds);
                if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                    // Consume the newline
                    int ch = getchar();
                    (void)ch;
                    monitoring = false;
                }

                if (monitoring) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
            std::cout << "\n";

            // ── Final snapshot: full details + copy-paste block ──
            auto pose_stamped = armController.getCurrentPose();
            auto& p = pose_stamped.pose;
            auto joints = armController.getCurrentJointValues();

            tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            std::cout << "\n============ FINAL ARM POSE SNAPSHOT ============\n";
            std::cout << "Frame: " << pose_stamped.header.frame_id << "\n";
            std::cout << "Position:\n";
            std::cout << "  x = " << p.position.x << "\n";
            std::cout << "  y = " << p.position.y << "\n";
            std::cout << "  z = " << p.position.z << "\n";
            std::cout << "Orientation (Quaternion):\n";
            std::cout << "  qx = " << p.orientation.x << "\n";
            std::cout << "  qy = " << p.orientation.y << "\n";
            std::cout << "  qz = " << p.orientation.z << "\n";
            std::cout << "  qw = " << p.orientation.w << "\n";
            std::cout << "Orientation (RPY):\n";
            std::cout << "  roll  = " << roll << " (" << roll * 180.0 / M_PI << " deg)\n";
            std::cout << "  pitch = " << pitch << " (" << pitch * 180.0 / M_PI << " deg)\n";
            std::cout << "  yaw   = " << yaw << " (" << yaw * 180.0 / M_PI << " deg)\n";
            std::cout << "Joint Values:";
            for (size_t i = 0; i < joints.size(); ++i) {
                std::cout << "  j" << (i+1) << "=" << joints[i]
                          << " (" << joints[i] * 180.0 / M_PI << " deg)";
            }
            std::cout << "\n";
            
            std::cout << "\n--- Copy-paste for Config namespace ---\n";
            std::cout << "// Cartesian (fallback):\n";
            std::cout << "const float ARM_XXXX_X = " << p.position.x << ";\n";
            std::cout << "const float ARM_XXXX_Y = " << p.position.y << ";\n";
            std::cout << "const float ARM_XXXX_Z = " << p.position.z << ";\n";
            std::cout << "const float ARM_XXXX_QX = " << p.orientation.x << ";\n";
            std::cout << "const float ARM_XXXX_QY = " << p.orientation.y << ";\n";
            std::cout << "const float ARM_XXXX_QZ = " << p.orientation.z << ";\n";
            std::cout << "const float ARM_XXXX_QW = " << p.orientation.w << ";\n";
            std::cout << "\n// Joint values (PREFERRED — paste into ARM_XXXX_JOINTS):\n";
            std::cout << "const std::vector<double> ARM_XXXX_JOINTS = {";
            for (size_t i = 0; i < joints.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << joints[i];
            }
            std::cout << "};\n";
            std::cout << "=================================================\n\n";
        }

        if (interactive_tuning) {
            RCLCPP_INFO(node->get_logger(), "=== INTERACTIVE POSE TUNING ===");
            RCLCPP_INFO(node->get_logger(), "Instructions:");
            RCLCPP_INFO(node->get_logger(), "  1. Use RViz MoveIt plugin to drag the arm to desired position");
            RCLCPP_INFO(node->get_logger(), "  2. OR enter joint values manually below");
            RCLCPP_INFO(node->get_logger(), "  3. Read back the end-effector Cartesian pose");
            RCLCPP_INFO(node->get_logger(), "  4. Copy values into Config namespace\n");
            
            std::cout << "Choose method:\n";
            std::cout << "  a) Read current pose (move arm physically/via RViz first)\n";
            std::cout << "  b) Send arm to a Cartesian pose (x y z qx qy qz qw)\n";
            std::cout << "  c) Send arm to a Cartesian pose (x y z roll pitch yaw)\n";
            std::cout << "Choice (a/b/c): ";
            
            char method;
            std::cin >> method;
            
            if (method == 'a') {
                // Just read and display
                auto pose_stamped = armController.getCurrentPose();
                auto& p = pose_stamped.pose;
                std::cout << "\nCurrent pose: x=" << p.position.x 
                          << " y=" << p.position.y 
                          << " z=" << p.position.z
                          << " qx=" << p.orientation.x 
                          << " qy=" << p.orientation.y
                          << " qz=" << p.orientation.z 
                          << " qw=" << p.orientation.w << "\n";
            } else if (method == 'b') {
                double x, y, z, qx, qy, qz, qw;
                std::cout << "Enter: x y z qx qy qz qw\n> ";
                std::cin >> x >> y >> z >> qx >> qy >> qz >> qw;
                bool ok = armController.moveToCartesianPose(x, y, z, qx, qy, qz, qw);
                if (ok) {
                    auto pose_stamped = armController.getCurrentPose();
                    auto& p = pose_stamped.pose;
                    std::cout << "Actual achieved pose: x=" << p.position.x 
                              << " y=" << p.position.y 
                              << " z=" << p.position.z
                              << " qx=" << p.orientation.x 
                              << " qy=" << p.orientation.y
                              << " qz=" << p.orientation.z 
                              << " qw=" << p.orientation.w << "\n";
                } else {
                    std::cout << "Pose unreachable!\n";
                }
            } else if (method == 'c') {
                double x, y, z, r, p_val, yaw_val;
                std::cout << "Enter: x y z roll pitch yaw (radians)\n> ";
                std::cin >> x >> y >> z >> r >> p_val >> yaw_val;
                bool ok = armController.moveToCartesianPose(x, y, z, r, p_val, yaw_val);
                if (ok) {
                    auto pose_stamped = armController.getCurrentPose();
                    auto& p = pose_stamped.pose;
                    std::cout << "Actual achieved pose: x=" << p.position.x 
                              << " y=" << p.position.y 
                              << " z=" << p.position.z
                              << " qx=" << p.orientation.x 
                              << " qy=" << p.orientation.y
                              << " qz=" << p.orientation.z 
                              << " qw=" << p.orientation.w << "\n";
                } else {
                    std::cout << "Pose unreachable!\n";
                }
            }
        }

        if (step_by_step) {
            RCLCPP_INFO(node->get_logger(), "=== STEP-BY-STEP PICK & PLACE ===");
            std::string input;
            bool aborted = false;

            // Helper lambda for user confirmation
            auto waitForUser = [&](const std::string& next_step) -> bool {
                std::cout << "\n>>> Next: " << next_step << "\n";
                std::cout << "    Proceed? (y/n): ";
                std::cin >> input;
                if (input != "y" && input != "Y") {
                    std::cout << "    Skipping remaining steps.\n";
                    return false;
                }
                return true;
            };

            // Pre-flight: verify arm connectivity with a simple "home" move
            std::cout << "\n=== PRE-FLIGHT CHECK ===\n";
            std::cout << "Testing arm connectivity by moving to 'home' (all joints=0)...\n";
            std::cout << "If this fails, run on Pi:  ros2 action list | grep follow_joint_trajectory\n";
            std::cout << "And kill zombies:          killall -9 move_group\n\n";
            
            {
                auto joints = armController.getCurrentJointValues();
                if (joints.empty()) {
                    RCLCPP_ERROR(node->get_logger(), "CANNOT READ JOINT STATES! Arm controller is disconnected.");
                    RCLCPP_ERROR(node->get_logger(), "Fix: Ensure so101_turtlebot.launch.py is running on Pi.");
                    continue;
                }
                std::cout << "Current joints: ";
                for (size_t i = 0; i < joints.size(); ++i) {
                    std::cout << "j" << (i+1) << "=" << (joints[i] * 180.0 / M_PI) << "° ";
                }
                std::cout << "\n";
            }

            if (!waitForUser("Move to HOME position (pre-flight test)")) { continue; }
            {
                bool home_ok = armController.moveToNamedTarget("home");
                if (!home_ok) {
                    RCLCPP_ERROR(node->get_logger(), 
                        "HOME move failed! Arm execution is broken.\n"
                        "  1. Kill zombies:  killall -9 move_group\n"
                        "  2. On Pi, restart: ros2 launch lerobot_moveit so101_turtlebot.launch.py\n"
                        "  3. On laptop, restart: ros2 launch lerobot_moveit so101_laptop.launch.py\n"
                        "  4. Try debug option 9 again.");
                    std::cout << "Continue anyway? (y/n): ";
                    std::cin >> input;
                    if (input != "y" && input != "Y") continue;
                } else {
                    RCLCPP_INFO(node->get_logger(), "Pre-flight PASSED — arm can plan and execute.");
                }
            }

            // Step 1: LOOK pose
            if (!aborted && waitForUser("Move arm to LOOK pose")) {
                bool s = moveArmPose(armController, node->get_logger(),
                    Config::ARM_LOOK_JOINTS,
                    Config::ARM_LOOK_X, Config::ARM_LOOK_Y, Config::ARM_LOOK_Z,
                    Config::ARM_LOOK_QX, Config::ARM_LOOK_QY, Config::ARM_LOOK_QZ, Config::ARM_LOOK_QW);
                RCLCPP_INFO(node->get_logger(), "LOOK pose: %s", s ? "SUCCESS" : "FAILED");
                if (s) {
                    // Read back actual achieved pose for debugging
                    auto actual = armController.getCurrentPose();
                    auto& p = actual.pose;
                    RCLCPP_INFO(node->get_logger(), "  Actual: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
                        p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));
            } else { aborted = true; }

            // Step 2: Wrist camera YOLO detection (3 attempts)
            std::string detected_name;
            if (!aborted && waitForUser("Detect object with wrist camera (3 attempts)")) {
                for (int attempt = 1; attempt <= 3; ++attempt) {
                    std::string obj = yolo.getObjectName(CameraSource::WRIST, true);
                    if (!obj.empty()) {
                        detected_name = obj;
                        RCLCPP_INFO(node->get_logger(), "Attempt %d/3: Detected '%s'", attempt, obj.c_str());
                        break;
                    } else {
                        RCLCPP_WARN(node->get_logger(), "Attempt %d/3: No detection.", attempt);
                    }
                    if (attempt < 3) std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                if (detected_name.empty()) {
                    RCLCPP_WARN(node->get_logger(), "All 3 attempts failed. Continuing anyway.");
                } else {
                    RCLCPP_INFO(node->get_logger(), "Final detection: '%s'", detected_name.c_str());
                }
            } else { aborted = true; }

            // Step 3: Open gripper
            if (!aborted && waitForUser("Open gripper")) {
                bool g = armController.openGripper();
                RCLCPP_INFO(node->get_logger(), "Open gripper: %s", g ? "SUCCESS" : "FAILED (execution error)");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else { aborted = true; }

            // Step 4: PICKUP pose
            if (!aborted && waitForUser("Move arm to PICKUP pose")) {
                bool s = moveArmPose(armController, node->get_logger(),
                    Config::ARM_PICKUP_JOINTS,
                    Config::ARM_PICKUP_X, Config::ARM_PICKUP_Y, Config::ARM_PICKUP_Z,
                    Config::ARM_PICKUP_QX, Config::ARM_PICKUP_QY, Config::ARM_PICKUP_QZ, Config::ARM_PICKUP_QW);
                RCLCPP_INFO(node->get_logger(), "PICKUP pose: %s", s ? "SUCCESS" : "FAILED");
            } else { aborted = true; }

            // Step 5: Close gripper (grasp)
            if (!aborted && waitForUser("Close gripper to grasp object")) {
                bool g = armController.closeGripper();
                RCLCPP_INFO(node->get_logger(), "Close gripper: %s", g ? "SUCCESS" : "FAILED (execution error)");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else { aborted = true; }

            // Step 6: CARRY pose
            if (!aborted && waitForUser("Move arm to CARRY pose")) {
                bool s = moveArmPose(armController, node->get_logger(),
                    Config::ARM_CARRY_JOINTS,
                    Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                    Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
                RCLCPP_INFO(node->get_logger(), "CARRY pose: %s", s ? "SUCCESS" : "FAILED");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else { aborted = true; }

            // Step 7: DROP pose
            if (!aborted && waitForUser("Move arm to DROP pose")) {
                bool s = moveArmPose(armController, node->get_logger(),
                    Config::ARM_DROP_JOINTS,
                    Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z,
                    Config::ARM_DROP_QX, Config::ARM_DROP_QY, Config::ARM_DROP_QZ, Config::ARM_DROP_QW);
                RCLCPP_INFO(node->get_logger(), "DROP pose: %s", s ? "SUCCESS" : "FAILED");
            } else { aborted = true; }

            // Step 8: Open gripper (release)
            if (!aborted && waitForUser("Open gripper to release object")) {
                bool g = armController.openGripper();
                RCLCPP_INFO(node->get_logger(), "Open gripper: %s", g ? "SUCCESS" : "FAILED (execution error)");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } else { aborted = true; }

            // Step 9: Return to home (safest resting pose)
            if (!aborted && waitForUser("Return arm to HOME pose")) {
                armController.moveToNamedTarget("home");
                RCLCPP_INFO(node->get_logger(), "Back to HOME pose.");
            }

            RCLCPP_INFO(node->get_logger(), "=== STEP-BY-STEP COMPLETE ===");
        }

        if (choice == 10) {
            RCLCPP_INFO(node->get_logger(), "=== SAVE CURRENT JOINTS TO YAML ===");
            std::cout << "Which pose to save? (LOOK / PICKUP / CARRY / DROP): ";
            std::string pose_name;
            std::cin >> pose_name;
            // Uppercase it
            for (auto& c : pose_name) c = std::toupper(c);

            if (pose_name != "LOOK" && pose_name != "PICKUP" &&
                pose_name != "CARRY" && pose_name != "DROP") {
                std::cout << "Invalid pose name. Must be LOOK, PICKUP, CARRY, or DROP.\n";
            } else {
                auto joints = armController.getCurrentJointValues();
                if (joints.empty()) {
                    RCLCPP_ERROR(node->get_logger(), "Cannot read joint values!");
                } else {
                    std::cout << "Current joints: [";
                    for (size_t i = 0; i < joints.size(); ++i) {
                        if (i > 0) std::cout << ", ";
                        std::cout << joints[i];
                    }
                    std::cout << "]\n";
                    std::cout << "Save these as " << pose_name << " joints? (y/n): ";
                    std::string confirm;
                    std::cin >> confirm;
                    if (confirm == "y" || confirm == "Y") {
                        saveJointsToYaml(pose_name, joints, node->get_logger());
                    } else {
                        std::cout << "Cancelled.\n";
                    }
                }
            }
        }

        if (choice < 0 || choice > 10) {
            std::cout << "Invalid choice. Please try again.\n";
        }
    }

    RCLCPP_INFO(node->get_logger(), "=== DEBUG MODE COMPLETE ===");
}

// Signal handler to log crash info before dying
void crash_signal_handler(int sig) {
    const char* sig_name = "UNKNOWN";
    switch (sig) {
        case SIGSEGV: sig_name = "SIGSEGV (Segmentation Fault)"; break;
        case SIGABRT: sig_name = "SIGABRT (Abort)"; break;
        case SIGFPE:  sig_name = "SIGFPE (Floating Point Exception)"; break;
        case SIGBUS:  sig_name = "SIGBUS (Bus Error)"; break;
    }
    fprintf(stderr, "\n\n"
        "============================================\n"
        "  CONTEST2 CRASHED: %s (signal %d)\n"
        "============================================\n"
        "  This is a bug. Check the output above for\n"
        "  the last operation before the crash.\n"
        "============================================\n", sig_name, sig);
    // Re-raise the signal with default handler to get a core dump
    signal(sig, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Install crash signal handlers for debugging
    signal(SIGSEGV, crash_signal_handler);
    signal(SIGABRT, crash_signal_handler);
    signal(SIGFPE,  crash_signal_handler);
    signal(SIGBUS,  crash_signal_handler);

    // Spin the node in a background thread. This is absolutely required for MoveIt!
    // Without this, the MoveGroupInterface constructor will deadlock waiting for the action server.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() {
        executor.spin();
    });
    // NOTE: Do NOT detach — we join this thread at shutdown to avoid crashes.

    // Parse command-line flags FIRST (needed before URDF loading)
    bool debug_mode = false;
    bool no_arm_mode = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--debug") {
            debug_mode = true;
        } else if (std::string(argv[i]) == "--no-arm") {
            no_arm_mode = true;
        }
    }
    // Also check environment variable (bulletproof — survives ros2 run + gnome-terminal)
    if (const char* env = std::getenv("MIE443_NO_ARM")) {
        if (std::string(env) == "1") {
            no_arm_mode = true;
            RCLCPP_INFO(node->get_logger(), "[env] MIE443_NO_ARM=1 detected → no-arm mode.");
        }
    }
    RCLCPP_INFO(node->get_logger(), "Flags: debug=%s, no_arm=%s",
        debug_mode ? "true" : "false", no_arm_mode ? "true" : "false");

    // Load arm poses from YAML config (edit arm_poses.yaml — no rebuild needed!)
    loadConfigFromYaml(node->get_logger());

    // Load the arm URDF and SRDF only when arm is enabled
    if (!no_arm_mode) {
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

        node->declare_parameter("robot_description_kinematics.arm.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.005);
        node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_timeout", 0.2);
        node->declare_parameter("robot_description_kinematics.arm.position_only_ik", false);
    } else {
        RCLCPP_INFO(node->get_logger(), "[--no-arm] Skipping arm URDF/SRDF/kinematics loading.");
    }

    if (debug_mode) {
        runDebugMode(node);
        executor.cancel();
        if (spinner_thread.joinable()) {
            spinner_thread.join();
        }
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "============================================");
    if (no_arm_mode) {
        RCLCPP_INFO(node->get_logger(), "   CONTEST 2 - NAV-ONLY MODE (no arm)");
    } else {
        RCLCPP_INFO(node->get_logger(), "         CONTEST 2 - MAIN MODE");
    }
    RCLCPP_INFO(node->get_logger(), "============================================");

    // Robot pose object + subscriber
    // IMPORTANT: AMCL publishes /amcl_pose with transient_local durability (latched).
    // We MUST use transient_local on the subscriber too, otherwise we miss the
    // already-published pose and the 10s wait loop times out.



    RobotPose robotPose(0, 0, 0);
    auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();


    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        amcl_qos,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );






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

    // Wait for AMCL to publish a valid initial pose (up to 10s)
    RCLCPP_INFO(node->get_logger(), "Waiting for AMCL pose (up to 10s)...");
    {
        auto wait_start = std::chrono::steady_clock::now();
        bool got_pose = false;
        while (rclcpp::ok()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - wait_start).count();
            // Check if AMCL has updated pose from default (0,0,0)
            if (robotPose.x != 0.0f || robotPose.y != 0.0f || robotPose.phi != 0.0f) {
                got_pose = true;
                break;
            }
            if (elapsed > 10) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!got_pose) {
            RCLCPP_WARN(node->get_logger(), "AMCL pose not received after 10s! Start position may be wrong.");
            RCLCPP_WARN(node->get_logger(), "Make sure you set 2D Pose Estimate in RViz before launching.");
        }
    }

    // FIX: Save starting position so we can return here (not hardcoded 0,0,0)
    float start_x = robotPose.x;
    float start_y = robotPose.y;
    float start_phi = robotPose.phi;
    RCLCPP_INFO(node->get_logger(), "Starting position saved: x=%.2f, y=%.2f, phi=%.2f",
                start_x, start_y, start_phi);

    // Create persistent objects — arm & tag detector only when arm is enabled
    std::unique_ptr<ArmController> arm;
    if (!no_arm_mode) {
        arm = std::make_unique<ArmController>(node);
        RCLCPP_INFO(node->get_logger(), "ArmController initialized.");
    } else {
        RCLCPP_INFO(node->get_logger(), "[--no-arm] Skipping ArmController.");
    }
    // AprilTagDetector is always initialized (only needs TF2, not URDF/MoveIt)
    auto tag_detector = std::make_unique<AprilTagDetector>(node);
    RCLCPP_INFO(node->get_logger(), "AprilTagDetector initialized.");
    YoloInterface yolo(node);
    RCLCPP_INFO(node->get_logger(), "YoloInterface initialized.");
    Navigation nav(node);
    RCLCPP_INFO(node->get_logger(), "Navigation initialized.");

    // State tracking (regular variables, not static)
    bool initialized = false;
    std::string manipulable_object_name = "";
    float manipulable_confidence = -1.0f;  // -1 = not detected (defaulted)
    // Scene detection data: object name, confidence, predefined box coords, robot pose at detection
    struct SceneDetection {
        std::string name;
        float confidence;      // YOLO confidence (-1 if unavailable)
        float box_x, box_y, box_phi;    // predefined coordinates from coords.xml
        float robot_x, robot_y, robot_phi;  // robot pose when detection was made
    };
    std::vector<SceneDetection> detected_scene_objects;
    // AprilTag diagnostic data: {box_index, tag_id, distance, angle_deg, x, y, z}
    struct AprilTagInfo { int box_idx; int tag_id; float dist; float angle_deg; float x; float y; float z; };
    std::vector<AprilTagInfo> detected_apriltags;
    size_t boxes_visited = 0;
    std::vector<bool> visited_boxes(boxes.coords.size(), false);
    bool manipulable_picked = false;
    bool object_placed = false;
    bool returning_home = false;
    bool finished = false;

    // *** Wait for Nav2 to be fully ready (costmap populated) ***
    // Without this, goals are accepted but immediately ABORTED because the
    // planner has no costmap data to plan with.
    if (!nav.waitUntilReady(30.0)) {
        RCLCPP_ERROR(node->get_logger(),
            "Nav2 not ready after 30s! Proceeding anyway, but navigation may fail.");
    }

    // Contest countdown timer — starts AFTER Nav2 is confirmed ready
    auto start_time = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "============================================");
    RCLCPP_INFO(node->get_logger(), "  Starting contest - 300 seconds begins NOW");
    RCLCPP_INFO(node->get_logger(), "============================================");

  try {
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
        //          (SKIPPED in --no-arm mode)
        // =====================================================================
        if (!initialized) {
            if (no_arm_mode || !arm) {
                RCLCPP_INFO(node->get_logger(), "=== PHASE 1: SKIPPED (%s) ===",
                    no_arm_mode ? "--no-arm mode" : "arm not initialized");
                RCLCPP_INFO(node->get_logger(), "[%lus] Proceeding directly to navigation.", secondsElapsed);
                initialized = true;
                continue;
            }

            RCLCPP_INFO(node->get_logger(), "=== PHASE 1: Detect & Pick Up Manipulable Object ===");
            
            // Step 1: Move arm to LOOK pose so wrist camera can see the top plate
            RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to LOOK pose...", secondsElapsed);
            bool look_ok = moveArmPose(arm.get(), node->get_logger(),
                                    Config::ARM_LOOK_JOINTS,
                                    Config::ARM_LOOK_X, Config::ARM_LOOK_Y, Config::ARM_LOOK_Z,
                                    Config::ARM_LOOK_QX, Config::ARM_LOOK_QY, Config::ARM_LOOK_QZ, Config::ARM_LOOK_QW);
            if (!look_ok) {
                RCLCPP_WARN(node->get_logger(), "[%lus] LOOK pose failed! Trying detection anyway...", secondsElapsed);
            }
            
            // Step 2: Detect manipulable object using wrist camera (saves image with bbox)
            RCLCPP_INFO(node->get_logger(), "[%lus] Waiting for wrist camera to stabilize...", secondsElapsed);
            std::this_thread::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(node->get_logger(), "[%lus] Running YOLO detection on wrist camera...", secondsElapsed);
            for (int attempt = 1; attempt <= Config::YOLO_MAX_RETRIES; ++attempt) {
                // true = save annotated image with bounding box
                manipulable_object_name = yolo.getObjectName(CameraSource::WRIST, true);
                if (!manipulable_object_name.empty()) {
                    manipulable_confidence = yolo.getConfidence();
                    RCLCPP_INFO(node->get_logger(), "[%lus] Detected '%s' (conf=%.2f) on attempt %d/%d (image saved)",
                                secondsElapsed, manipulable_object_name.c_str(), manipulable_confidence, attempt, Config::YOLO_MAX_RETRIES);
                    break;
                }
                if (attempt < Config::YOLO_MAX_RETRIES) {
                    RCLCPP_WARN(node->get_logger(), "[%lus] YOLO attempt %d/%d returned empty. Retrying...",
                                secondsElapsed, attempt, Config::YOLO_MAX_RETRIES);
                    std::this_thread::sleep_for(std::chrono::duration<double>(Config::YOLO_RETRY_DELAY));
                }
            }
            
            if (manipulable_object_name.empty()) {
                RCLCPP_WARN(node->get_logger(), "[%lus] WARNING: No object detected after %d attempts.",
                            secondsElapsed, Config::YOLO_MAX_RETRIES);
                manipulable_object_name = "cup";
                manipulable_confidence = -1.0f;  // defaulted, not actually detected
                RCLCPP_WARN(node->get_logger(), "[%lus] Defaulting to '%s' (confidence: N/A).", secondsElapsed, manipulable_object_name.c_str());
            } else {
                RCLCPP_INFO(node->get_logger(), "[%lus] *** Manipulable object: '%s' ***", secondsElapsed, manipulable_object_name.c_str());
            }

            // Step 3: Open gripper, move to PICKUP, close gripper
            RCLCPP_INFO(node->get_logger(), "[%lus] Opening gripper...", secondsElapsed);
            arm->openGripper();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to PICKUP pose...", secondsElapsed);
            bool pickup_ok = moveArmPose(arm.get(), node->get_logger(),
                                    Config::ARM_PICKUP_JOINTS,
                                    Config::ARM_PICKUP_X, Config::ARM_PICKUP_Y, Config::ARM_PICKUP_Z,
                                    Config::ARM_PICKUP_QX, Config::ARM_PICKUP_QY, Config::ARM_PICKUP_QZ, Config::ARM_PICKUP_QW);
            if (!pickup_ok) {
                RCLCPP_WARN(node->get_logger(), "[%lus] PICKUP pose failed! Grasping from current position.", secondsElapsed);
            }
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Closing gripper to grasp object...", secondsElapsed);
            arm->closeGripper();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Step 4: Move to CARRY pose for navigation
            RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to CARRY pose...", secondsElapsed);
            bool carry_ok = moveArmPose(arm.get(), node->get_logger(),
                                    Config::ARM_CARRY_JOINTS,
                                    Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                                    Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
            if (!carry_ok) {
                RCLCPP_WARN(node->get_logger(), "[%lus] CARRY pose failed! Continuing with arm in current position.", secondsElapsed);
            }
            
            RCLCPP_INFO(node->get_logger(), "[%lus] *** Object picked up — ready to navigate! ***", secondsElapsed);
            manipulable_picked = true;
            
            initialized = true;
            continue;
        }

        // =====================================================================
        // PHASE 2: Navigate to each box, detect scene objects, match & drop
        // =====================================================================
        if (boxes_visited < boxes.coords.size()) {
            // Time budget check — reserve time to return home
            int time_remaining = 300 - (int)secondsElapsed;
            if (time_remaining < Config::TIME_RESERVE_SECONDS) {
                RCLCPP_WARN(node->get_logger(),
                    "[%lus] Only %ds remaining! Skipping remaining boxes to return home.",
                    secondsElapsed, time_remaining);
                boxes_visited = boxes.coords.size();
                continue;
            }

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
            // coords.xml phi points AWAY from the object/bin (direction robot came from).
            // Flip by π so the robot's front camera faces the object.
            float raw_phi = boxes.coords[current_box_index][2];
            float phi = raw_phi + M_PI;
            if (phi > M_PI) phi -= 2.0 * M_PI;

            RCLCPP_INFO(node->get_logger(), "==================================================");
            RCLCPP_INFO(node->get_logger(), "=== PHASE 2: Box %d (%zu/%zu) ===",
                        current_box_index, boxes_visited, boxes.coords.size());
            RCLCPP_INFO(node->get_logger(), "[%lus] Target: x=%.2f, y=%.2f, phi=%.2f (Dist: %.2fm)",
                        secondsElapsed, x, y, phi, min_dist);

            // Navigate directly to the coords.xml position.
            // The instructor confirmed: these coordinates ARE the robot's viewing pose,
            // with the robot facing the object and AprilTag nearly in front.
            // No standoff offset — go exactly where specified.
            RCLCPP_INFO(node->get_logger(), "[%lus] Robot current pose: x=%.2f, y=%.2f, phi=%.2f",
                        secondsElapsed, robotPose.x, robotPose.y, robotPose.phi);
            
            // Per-goal timeout: capped so robot doesn't circle forever near one box.
            // Also capped by remaining contest time (minimum 10s floor).
            double nav_timeout = std::min(Config::NAV_TIMEOUT_MAX, std::max(10.0, (double)(300 - (int)secondsElapsed)));
            bool nav_succeeded = nav.moveToGoal(x, y, phi, nav_timeout);
            if (!nav_succeeded) {
                RCLCPP_WARN(node->get_logger(), "[%lus] Navigation to box %d timed out/failed. Attempting backup recovery...", 
                            secondsElapsed, current_box_index);
                // Back up 0.3m to escape costmap boundary / stuck situation, then retry
                if (nav.backup(0.3, 0.1, 10.0)) {
                    RCLCPP_INFO(node->get_logger(), "[%lus] Backup succeeded. Retrying navigation to box %d...",
                                secondsElapsed, current_box_index);
                    double retry_timeout = std::min(Config::NAV_TIMEOUT_MAX * 0.5, std::max(10.0, (double)(300 - (int)secondsElapsed)));
                    nav_succeeded = nav.moveToGoal(x, y, phi, retry_timeout);
                    if (!nav_succeeded) {
                        RCLCPP_WARN(node->get_logger(), "[%lus] Retry navigation also failed. Treating as arrived — proceeding with detection.",
                                    secondsElapsed);
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(), "[%lus] Backup failed. Treating as arrived — proceeding with detection.",
                                secondsElapsed);
                }
            }
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Arrived at box %d (%.2f, %.2f, %.1fdeg). Taking 2 pictures...",
                        secondsElapsed, current_box_index, x, y, phi * 180.0 / M_PI);
            
            // Take 2 OAK-D pictures from the arrival heading — check for BOTH object and AprilTag
            std::string scene_object_name;
            int best_tag_id = -1;
            float best_tag_dist = 0.0f;
            float best_tag_angle = 0.0f;

            // Track the robot pose when the object was first detected
            // (used as fallback heading if no AprilTag found)
            float obj_detect_x = 0.0f, obj_detect_y = 0.0f, obj_detect_phi = 0.0f;
            bool have_obj_detect_pose = false;

            for (int pic = 1; pic <= 2; ++pic) {
                // YOLO detection
                if (scene_object_name.empty()) {
                    scene_object_name = yolo.getObjectName(CameraSource::OAKD, true);
                    if (!scene_object_name.empty()) {
                        RCLCPP_INFO(node->get_logger(), "[%lus] OAK-D pic %d/2: detected '%s'",
                                    secondsElapsed, pic, scene_object_name.c_str());
                        if (!have_obj_detect_pose) {
                            obj_detect_x = robotPose.x;
                            obj_detect_y = robotPose.y;
                            obj_detect_phi = robotPose.phi;
                            have_obj_detect_pose = true;
                        }
                    } else {
                        RCLCPP_INFO(node->get_logger(), "[%lus] OAK-D pic %d/2: no object.",
                                    secondsElapsed, pic);
                    }
                }
                // AprilTag detection
                if (best_tag_id < 0) {
                    auto visible_tags = tag_detector->getVisibleTags(Config::CANDIDATE_TAG_IDS);
                    for (int tid : visible_tags) {
                        auto tp_opt = tag_detector->getTagPose(tid);
                        if (tp_opt.has_value()) {
                            auto& tp = tp_opt.value();
                            float dist = std::sqrt(tp.position.x * tp.position.x +
                                                   tp.position.y * tp.position.y);
                            float angle = std::atan2(tp.position.y, tp.position.x);
                            RCLCPP_INFO(node->get_logger(),
                                "[%lus] [AprilTag] ID=%d | dist=%.3fm | angle=%.1fdeg",
                                secondsElapsed, tid, dist, angle * 180.0 / M_PI);
                            detected_apriltags.push_back({current_box_index, tid, dist,
                                angle * 180.0f / (float)M_PI,
                                (float)tp.position.x, (float)tp.position.y, (float)tp.position.z});
                            if (best_tag_id < 0) {
                                best_tag_id = tid;
                                best_tag_dist = dist;
                                best_tag_angle = angle;
                            }
                        }
                    }
                }
                // Early exit if both found
                if (!scene_object_name.empty() && best_tag_id >= 0) break;
                if (pic < 2) std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // If AprilTag visible but no object, take 2 extra OAK-D pictures
            // Tag confirms we're at the right spot — give YOLO extra chances before spinning
            if (scene_object_name.empty() && best_tag_id >= 0) {
                RCLCPP_INFO(node->get_logger(),
                    "[%lus] AprilTag ID=%d visible but no object. Taking 2 extra pictures...",
                    secondsElapsed, best_tag_id);
                for (int extra = 1; extra <= 2; ++extra) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    std::string obj = yolo.getObjectName(CameraSource::OAKD, true);
                    if (!obj.empty()) {
                        scene_object_name = obj;
                        RCLCPP_INFO(node->get_logger(), "[%lus] Extra pic %d: detected '%s'!",
                                    secondsElapsed, extra, obj.c_str());
                        if (!have_obj_detect_pose) {
                            obj_detect_x = robotPose.x;
                            obj_detect_y = robotPose.y;
                            obj_detect_phi = robotPose.phi;
                            have_obj_detect_pose = true;
                        }
                        break;
                    } else {
                        RCLCPP_WARN(node->get_logger(), "[%lus] Extra pic %d: still no object.",
                                    secondsElapsed, extra);
                    }
                }
            }
            
            // If either is missing after 2 pictures, spin in small steps searching for BOTH
            if (scene_object_name.empty() || best_tag_id < 0) {
                RCLCPP_WARN(node->get_logger(), "[%lus] After 2 pics: object=%s, tag=%s. Spinning to search...",
                            secondsElapsed,
                            scene_object_name.empty() ? "NONE" : scene_object_name.c_str(),
                            best_tag_id >= 0 ? ("ID=" + std::to_string(best_tag_id)).c_str() : "NONE");
                
                int object_found_step = -1;  // track which step found the object
                
                for (int spin_step = 1; spin_step <= Config::SCENE_SPIN_STEPS; ++spin_step) {
                    RCLCPP_INFO(node->get_logger(), "[%lus] Search spin step %d/%d (%.0f deg)...",
                                secondsElapsed, spin_step, Config::SCENE_SPIN_STEPS,
                                Config::SCENE_SPIN_INCREMENT * 180.0 / M_PI);
                    
                    if (!nav.spin(Config::SCENE_SPIN_INCREMENT)) {
                        RCLCPP_WARN(node->get_logger(), "[%lus] Nav2 Spin failed, using moveToGoal rotation...", secondsElapsed);
                        float new_phi = robotPose.phi + Config::SCENE_SPIN_INCREMENT;
                        while (new_phi > M_PI) new_phi -= 2.0 * M_PI;
                        while (new_phi < -M_PI) new_phi += 2.0 * M_PI;
                        nav.moveToGoal(robotPose.x, robotPose.y, new_phi);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    
                    bool tag_found_this_step = false;
                    
                    // Check for object
                    if (scene_object_name.empty()) {
                        scene_object_name = yolo.getObjectName(CameraSource::OAKD, true);
                        if (!scene_object_name.empty()) {
                            RCLCPP_INFO(node->get_logger(), "[%lus] Object '%s' found at spin step %d!",
                                        secondsElapsed, scene_object_name.c_str(), spin_step);
                            object_found_step = spin_step;
                            if (!have_obj_detect_pose) {
                                obj_detect_x = robotPose.x;
                                obj_detect_y = robotPose.y;
                                obj_detect_phi = robotPose.phi;
                                have_obj_detect_pose = true;
                            }
                        }
                    }
                    // Check for AprilTag
                    if (best_tag_id < 0) {
                        auto spin_tags = tag_detector->getVisibleTags(Config::CANDIDATE_TAG_IDS);
                        for (int tid : spin_tags) {
                            auto tp_opt = tag_detector->getTagPose(tid);
                            if (tp_opt.has_value()) {
                                auto& tp = tp_opt.value();
                                float dist = std::sqrt(tp.position.x * tp.position.x +
                                                       tp.position.y * tp.position.y);
                                float angle = std::atan2(tp.position.y, tp.position.x);
                                RCLCPP_INFO(node->get_logger(),
                                    "[%lus] [AprilTag] ID=%d found at spin step %d | dist=%.3fm | angle=%.1fdeg",
                                    secondsElapsed, tid, spin_step, dist, angle * 180.0 / M_PI);
                                detected_apriltags.push_back({current_box_index, tid, dist,
                                    angle * 180.0f / (float)M_PI,
                                    (float)tp.position.x, (float)tp.position.y, (float)tp.position.z});
                                if (best_tag_id < 0) {
                                    best_tag_id = tid;
                                    best_tag_dist = dist;
                                    best_tag_angle = angle;
                                    tag_found_this_step = true;
                                }
                            }
                        }
                    }
                    
                    // If AprilTag was NEWLY found THIS step but no object yet,
                    // take 2 extra pictures — the tag confirms this is a good angle
                    if (scene_object_name.empty() && tag_found_this_step) {
                        RCLCPP_INFO(node->get_logger(),
                            "[%lus] Spin step %d: NEW AprilTag but no object. Taking 2 extra pictures...",
                            secondsElapsed, spin_step);
                        for (int extra = 1; extra <= 2; ++extra) {
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                            std::string obj = yolo.getObjectName(CameraSource::OAKD, true);
                            if (!obj.empty()) {
                                scene_object_name = obj;
                                RCLCPP_INFO(node->get_logger(), "[%lus] Spin extra pic %d: detected '%s'!",
                                            secondsElapsed, extra, obj.c_str());
                                if (!have_obj_detect_pose) {
                                    obj_detect_x = robotPose.x;
                                    obj_detect_y = robotPose.y;
                                    obj_detect_phi = robotPose.phi;
                                    have_obj_detect_pose = true;
                                }
                                break;
                            } else {
                                RCLCPP_WARN(node->get_logger(), "[%lus] Spin extra pic %d: still no object.",
                                            secondsElapsed, extra);
                            }
                        }
                    }

                    // If object found THIS step but no tag, take extra tag readings
                    // (tag & object are on the same face, so tag should be visible NOW)
                    if (best_tag_id < 0 && object_found_step == spin_step) {
                        RCLCPP_INFO(node->get_logger(),
                            "[%lus] Object found but no tag at step %d. Taking %d extra tag readings...",
                            secondsElapsed, spin_step, 5);
                        for (int retry = 1; retry <= 5; ++retry) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(800));
                            auto retry_tags = tag_detector->getVisibleTags(Config::CANDIDATE_TAG_IDS);
                            for (int tid : retry_tags) {
                                auto tp_opt = tag_detector->getTagPose(tid);
                                if (tp_opt.has_value()) {
                                    auto& tp = tp_opt.value();
                                    float dist = std::sqrt(tp.position.x * tp.position.x +
                                                           tp.position.y * tp.position.y);
                                    float angle = std::atan2(tp.position.y, tp.position.x);
                                    RCLCPP_INFO(node->get_logger(),
                                        "[%lus] [AprilTag] retry %d: ID=%d | dist=%.3fm | angle=%.1fdeg",
                                        secondsElapsed, retry, tid, dist, angle * 180.0 / M_PI);
                                    detected_apriltags.push_back({current_box_index, tid, dist,
                                        angle * 180.0f / (float)M_PI,
                                        (float)tp.position.x, (float)tp.position.y, (float)tp.position.z});
                                    if (best_tag_id < 0) {
                                        best_tag_id = tid;
                                        best_tag_dist = dist;
                                        best_tag_angle = angle;
                                    }
                                }
                            }
                            if (best_tag_id >= 0) break;
                        }
                    }
                    
                    // Early exit if both found
                    if (!scene_object_name.empty() && best_tag_id >= 0) {
                        RCLCPP_INFO(node->get_logger(),
                            "[%lus] Both object and tag found at spin step %d. Done searching.",
                            secondsElapsed, spin_step);
                        break;
                    }
                }
            }
            
            // If EITHER is still missing, decide whether to skip or attempt a no-tag drop
            if (scene_object_name.empty() || best_tag_id < 0) {
                // Special case: object found, matches manipulatable, carrying it, no tag.
                // Fall through to MATCH FOUND for no-tag drop instead of skipping.
                bool attempt_no_tag_drop = false;
                if (!scene_object_name.empty() && best_tag_id < 0 &&
                    manipulable_picked && !object_placed && !manipulable_object_name.empty() &&
                    scene_object_name == manipulable_object_name &&
                    !no_arm_mode && arm) {
                    attempt_no_tag_drop = true;
                    RCLCPP_WARN(node->get_logger(),
                        "[%lus] Object '%s' matches manipulatable but NO AprilTag. Will attempt no-tag drop.",
                        secondsElapsed, scene_object_name.c_str());
                }

                if (!attempt_no_tag_drop) {
                    RCLCPP_WARN(node->get_logger(),
                        "[%lus] Skipping box %d: object=%s, tag=%s. Moving to next.",
                        secondsElapsed, current_box_index,
                        scene_object_name.empty() ? "NONE" : scene_object_name.c_str(),
                        best_tag_id >= 0 ? ("ID=" + std::to_string(best_tag_id)).c_str() : "NONE");
                    if (!scene_object_name.empty()) {
                        detected_scene_objects.push_back({scene_object_name, yolo.getConfidence(),
                            x, y, phi, (float)robotPose.x, (float)robotPose.y, (float)robotPose.phi});
                    }
                    continue;
                }
            }
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Detected scene object: '%s' (conf=%.2f) and AprilTag ID=%d at box %d",
                        secondsElapsed, scene_object_name.c_str(), yolo.getConfidence(), best_tag_id, current_box_index);
            detected_scene_objects.push_back({scene_object_name, yolo.getConfidence(),
                x, y, phi, (float)robotPose.x, (float)robotPose.y, (float)robotPose.phi});
            
            // In no-arm mode, log everything and move on
            if (no_arm_mode || !arm) {
                RCLCPP_INFO(node->get_logger(), "[%lus] [--no-arm] Object='%s', Tag=%s. Moving to next box.",
                    secondsElapsed, scene_object_name.c_str(),
                    best_tag_id >= 0 ? ("ID=" + std::to_string(best_tag_id)).c_str() : "NONE");
                continue;
            }

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
            
            // AprilTag was already searched during spin-and-detect above.
            // If no tag, use the robot's heading when the object was detected as fallback.
            if (best_tag_id < 0) {
                RCLCPP_WARN(node->get_logger(),
                    "[%lus] No AprilTag found — using object-detection heading for drop.",
                    secondsElapsed);

                if (have_obj_detect_pose) {
                    // Rotate to the ABSOLUTE heading where we saw the object.
                    // Use moveToGoal (not nav.spin) because robotPose.phi from
                    // AMCL can be stale after multiple spin steps, causing a
                    // relative rotation to overshoot/undershoot.  moveToGoal
                    // uses the controller's own TF lookups for accurate heading.
                    RCLCPP_INFO(node->get_logger(),
                        "[%lus] No-tag drop: Rotating to object-detection heading (%.1fdeg) via moveToGoal",
                        secondsElapsed, obj_detect_phi * 180.0 / M_PI);
                    nav.moveToGoal(robotPose.x, robotPose.y, obj_detect_phi, 10.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                } else {
                    RCLCPP_WARN(node->get_logger(),
                        "[%lus] No saved detection pose — dropping from current heading.",
                        secondsElapsed);
                }

                // No forward drive — drop directly from current position
                // (no AprilTag means we can't judge distance to bin accurately)

                // === DROP SEQUENCE (no-tag fallback) ===
                RCLCPP_INFO(node->get_logger(), "[%lus] === NO-TAG DROP: DROPPING OBJECT ===" , secondsElapsed);

                RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to DROP pose...", secondsElapsed);
                bool drop_ok = moveArmPose(arm.get(), node->get_logger(),
                                        Config::ARM_DROP_JOINTS,
                                        Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z,
                                        Config::ARM_DROP_QX, Config::ARM_DROP_QY, Config::ARM_DROP_QZ, Config::ARM_DROP_QW);
                if (!drop_ok) {
                    RCLCPP_WARN(node->get_logger(), "[%lus] DROP pose failed! Opening gripper from current position.", secondsElapsed);
                }

                RCLCPP_INFO(node->get_logger(), "[%lus] Opening gripper to release object...", secondsElapsed);
                arm->openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));

                RCLCPP_INFO(node->get_logger(), "[%lus] Returning arm to CARRY pose...", secondsElapsed);
                moveArmPose(arm.get(), node->get_logger(),
                                        Config::ARM_CARRY_JOINTS,
                                        Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                                        Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);

                RCLCPP_INFO(node->get_logger(), "[%lus] *** Object placed (no-tag fallback)! ***", secondsElapsed);
                object_placed = true;
                continue;
            }
            
            // === MAP-FRAME BIN ALIGNMENT ===
            // One-shot: read tag in base_link frame, convert to MAP frame using
            // current robot pose, then Phase A (rotate to face bin) + Phase B
            // (drive to drop distance).  Tag is only read once; all subsequent
            // motion uses absolute map coordinates so stale angles can't cause
            // the robot to drive away from the bin.
            float bin_map_x = 0.0f, bin_map_y = 0.0f;
            bool have_bin_position = false;

            // Try a fresh tag reading first (tag may still be in view)
            auto alignment_tag = tag_detector->getTagPose(best_tag_id);
            if (alignment_tag.has_value()) {
                float tag_x = alignment_tag->position.x;   // forward in base_link
                float tag_y = alignment_tag->position.y;   // left in base_link
                float tag_dist = std::sqrt(tag_x * tag_x + tag_y * tag_y);
                float tag_angle = std::atan2(tag_y, tag_x);
                bin_map_x = robotPose.x + tag_dist * std::cos(robotPose.phi + tag_angle);
                bin_map_y = robotPose.y + tag_dist * std::sin(robotPose.phi + tag_angle);
                have_bin_position = true;
                RCLCPP_INFO(node->get_logger(),
                    "[%lus] Fresh tag %d: dist=%.3f angle=%.1fdeg → Bin MAP (%.3f, %.3f)",
                    secondsElapsed, best_tag_id, tag_dist, tag_angle * 180.0 / M_PI,
                    bin_map_x, bin_map_y);
            } else {
                // Tag not currently visible — spin slowly to re-acquire it
                RCLCPP_WARN(node->get_logger(),
                    "[%lus] Tag %d not visible. Spinning to re-acquire...",
                    secondsElapsed, best_tag_id);
                for (int search = 0; search < 6; ++search) {
                    nav.spin(M_PI / 6.0, 8.0);   // 30° per step, up to 180°
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    auto retry = tag_detector->getTagPose(best_tag_id);
                    if (retry.has_value()) {
                        float tx = retry->position.x;
                        float ty = retry->position.y;
                        float td = std::sqrt(tx * tx + ty * ty);
                        float ta = std::atan2(ty, tx);
                        bin_map_x = robotPose.x + td * std::cos(robotPose.phi + ta);
                        bin_map_y = robotPose.y + td * std::sin(robotPose.phi + ta);
                        have_bin_position = true;
                        RCLCPP_INFO(node->get_logger(),
                            "[%lus] Re-acquired tag %d at step %d → Bin MAP (%.3f, %.3f)",
                            secondsElapsed, best_tag_id, search + 1,
                            bin_map_x, bin_map_y);
                        break;
                    }
                }
            }

            if (!have_bin_position) {
                RCLCPP_WARN(node->get_logger(),
                    "[%lus] Could not acquire tag %d. Dropping from current position.",
                    secondsElapsed, best_tag_id);
            } else {
                // Compute alignment in MAP frame
                float dx_bin = bin_map_x - robotPose.x;
                float dy_bin = bin_map_y - robotPose.y;
                float dist_to_bin = std::sqrt(dx_bin * dx_bin + dy_bin * dy_bin);
                float angle_to_bin = std::atan2(dy_bin, dx_bin);  // absolute MAP heading
                float angle_error = angle_to_bin - robotPose.phi;
                while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
                while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

                RCLCPP_INFO(node->get_logger(),
                    "[%lus] Bin: %.3fm away, heading=%.1fdeg, error=%.1fdeg",
                    secondsElapsed, dist_to_bin, angle_to_bin * 180.0 / M_PI,
                    angle_error * 180.0 / M_PI);

                // Phase A: Rotate in place to face the bin
                if (std::abs(angle_error) > 0.05) {   // ~3° threshold
                    RCLCPP_INFO(node->get_logger(),
                        "[%lus] Phase A: Rotating %.1fdeg to face bin",
                        secondsElapsed, angle_error * 180.0 / M_PI);
                    nav.spin(angle_error, 10.0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                // Phase B: Drive forward/backward to drop distance
                // Recompute from updated pose after rotation
                float dx2 = bin_map_x - robotPose.x;
                float dy2 = bin_map_y - robotPose.y;
                float dist2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
                float angle2 = std::atan2(dy2, dx2);
                float drive_dist = dist2 - Config::DESIRED_DROP_DISTANCE;

                // Temporarily tighten Nav2 goal tolerance so small moves
                // (5-15cm) aren't swallowed by the normal tolerance.
                bool tolerance_tightened = false;
                if (std::abs(drive_dist) > 0.03) {
                    try {
                        auto tol_client = std::make_shared<rclcpp::SyncParametersClient>(
                            node, "/controller_server");
                        if (tol_client->wait_for_service(std::chrono::seconds(2))) {
                            tol_client->set_parameters({
                                rclcpp::Parameter("general_goal_checker.xy_goal_tolerance", Config::ALIGNMENT_XY_TOLERANCE)
                            });
                            tolerance_tightened = true;
                            RCLCPP_INFO(node->get_logger(),
                                "[%lus] Tightened goal tolerance to %.2fm for alignment",
                                secondsElapsed, Config::ALIGNMENT_XY_TOLERANCE);
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(node->get_logger(),
                            "[%lus] Could not tighten tolerance: %s", secondsElapsed, e.what());
                    }
                }

                if (drive_dist > 0.03) {
                    float gx = robotPose.x + drive_dist * std::cos(angle2);
                    float gy = robotPose.y + drive_dist * std::sin(angle2);
                    float gyaw = angle2;
                    while (gyaw > M_PI) gyaw -= 2.0 * M_PI;
                    while (gyaw < -M_PI) gyaw += 2.0 * M_PI;
                    RCLCPP_INFO(node->get_logger(),
                        "[%lus] Phase B: Drive %.3fm → goal MAP (%.3f, %.3f, %.1fdeg)",
                        secondsElapsed, drive_dist, gx, gy, gyaw * 180.0 / M_PI);
                    nav.moveToGoal(gx, gy, gyaw, 15.0);
                } else if (drive_dist < -0.03) {
                    float backup = -drive_dist;
                    float gx = robotPose.x - backup * std::cos(angle2);
                    float gy = robotPose.y - backup * std::sin(angle2);
                    float gyaw = angle2;
                    while (gyaw > M_PI) gyaw -= 2.0 * M_PI;
                    while (gyaw < -M_PI) gyaw += 2.0 * M_PI;
                    RCLCPP_WARN(node->get_logger(),
                        "[%lus] Phase B: Too close (%.3fm past). Backing up %.3fm → MAP (%.3f, %.3f)",
                        secondsElapsed, backup, backup, gx, gy);
                    nav.moveToGoal(gx, gy, gyaw, 15.0);
                } else {
                    RCLCPP_INFO(node->get_logger(),
                        "[%lus] At drop distance (%.3fm). No drive needed.",
                        secondsElapsed, dist2);
                }

                // Restore normal goal tolerance for subsequent navigation
                if (tolerance_tightened) {
                    try {
                        auto tol_client = std::make_shared<rclcpp::SyncParametersClient>(
                            node, "/controller_server");
                        if (tol_client->wait_for_service(std::chrono::seconds(2))) {
                            tol_client->set_parameters({
                                rclcpp::Parameter("general_goal_checker.xy_goal_tolerance", 0.25)
                            });
                            RCLCPP_INFO(node->get_logger(),
                                "[%lus] Restored goal tolerance to 0.25m",
                                secondsElapsed);
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(node->get_logger(),
                            "[%lus] Could not restore tolerance: %s", secondsElapsed, e.what());
                    }
                }
            }

            // Phase C: Final fixed forward drive toward the bin before dropping
            {
                float fwd_x = robotPose.x + Config::FORWARD_DRIVE_BEFORE_DROP * std::cos(robotPose.phi);
                float fwd_y = robotPose.y + Config::FORWARD_DRIVE_BEFORE_DROP * std::sin(robotPose.phi);
                RCLCPP_INFO(node->get_logger(),
                    "[%lus] Phase C: Final forward drive %.2fm → MAP (%.3f, %.3f)",
                    secondsElapsed, Config::FORWARD_DRIVE_BEFORE_DROP, fwd_x, fwd_y);
                nav.moveToGoal(fwd_x, fwd_y, robotPose.phi, 15.0);
            }

            // === DROP SEQUENCE ===
            RCLCPP_INFO(node->get_logger(), "[%lus] === DROPPING OBJECT INTO BIN ===", secondsElapsed);
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Moving arm to DROP pose...", secondsElapsed);
            bool drop_ok = moveArmPose(arm.get(), node->get_logger(),
                                    Config::ARM_DROP_JOINTS,
                                    Config::ARM_DROP_X, Config::ARM_DROP_Y, Config::ARM_DROP_Z,
                                    Config::ARM_DROP_QX, Config::ARM_DROP_QY, Config::ARM_DROP_QZ, Config::ARM_DROP_QW);
            if (!drop_ok) {
                RCLCPP_WARN(node->get_logger(), "[%lus] DROP pose failed! Opening gripper from current position.", secondsElapsed);
            }
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Opening gripper to release object...", secondsElapsed);
            arm->openGripper();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            RCLCPP_INFO(node->get_logger(), "[%lus] Returning arm to CARRY pose...", secondsElapsed);
            moveArmPose(arm.get(), node->get_logger(),
                                    Config::ARM_CARRY_JOINTS,
                                    Config::ARM_CARRY_X, Config::ARM_CARRY_Y, Config::ARM_CARRY_Z,
                                    Config::ARM_CARRY_QX, Config::ARM_CARRY_QY, Config::ARM_CARRY_QZ, Config::ARM_CARRY_QW);
            
            RCLCPP_INFO(node->get_logger(), "[%lus] *** Object placed in bin successfully! ***", secondsElapsed);
            object_placed = true;
            
            continue;
        }

        // =====================================================================
        // PHASE 3: Signal return home (actual navigation done after loop)
        // =====================================================================
        if (!returning_home) {
            RCLCPP_INFO(node->get_logger(), "==================================================");
            RCLCPP_INFO(node->get_logger(), "=== PHASE 3: All boxes visited — returning home ===");
            returning_home = true;
            finished = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ==========================================================================
    // POST-LOOP: ALWAYS return home and save results (even after timeout)
    // ==========================================================================
    auto final_now = std::chrono::system_clock::now();
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(final_now - start_time).count();

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "============================================");
        RCLCPP_WARN(node->get_logger(), "  CONTEST TIME LIMIT REACHED (%lus)!", secondsElapsed);
        RCLCPP_WARN(node->get_logger(), "  Visited %zu/%zu boxes before timeout.", boxes_visited, boxes.coords.size());
        RCLCPP_WARN(node->get_logger(), "============================================");
    }

    RCLCPP_INFO(node->get_logger(), "==================================================");
    RCLCPP_INFO(node->get_logger(), "[%lus] Navigating back to start (%.2f, %.2f, %.2f)...",
                secondsElapsed, start_x, start_y, start_phi);
    double return_timeout = std::min(45.0, std::max(10.0, 330.0 - (double)secondsElapsed));
    if (rclcpp::ok() && nav.moveToGoal(start_x, start_y, start_phi, return_timeout)) {
        RCLCPP_INFO(node->get_logger(), "Successfully returned to start.");
    } else {
        RCLCPP_WARN(node->get_logger(), "Failed to return to start position.");
    }

    // Update time after return attempt
    auto results_now = std::chrono::system_clock::now();
    uint64_t total_seconds = std::chrono::duration_cast<std::chrono::seconds>(results_now - start_time).count();

    // Write results
    RCLCPP_INFO(node->get_logger(), "Writing results to contest2_results.txt...");
    std::ofstream outfile("contest2_results.txt");
    if (outfile.is_open()) {
        outfile << "=== CONTEST 2 RESULTS ===\n";
        outfile << "Mode: " << (no_arm_mode ? "NAV-ONLY (no arm)" : "FULL (with arm)") << "\n";
        outfile << "Manipulable Object: " << (no_arm_mode ? "N/A (no-arm mode)" : (manipulable_object_name.empty() ? "NOT DETECTED" : manipulable_object_name));
        if (!no_arm_mode && !manipulable_object_name.empty()) {
            if (manipulable_confidence >= 0.0f) {
                outfile << " (confidence: " << std::fixed << std::setprecision(2) << manipulable_confidence << ")";
            } else {
                outfile << " (confidence: N/A - defaulted)";
            }
        }
        outfile << "\n";
        outfile << "Object Placed: " << (no_arm_mode ? "N/A" : (object_placed ? "YES" : "NO")) << "\n";
        outfile << "Boxes Visited: " << boxes_visited << "/" << boxes.coords.size() << "\n";
        outfile << "\nDetected Scene Objects:\n";
        for (size_t i = 0; i < detected_scene_objects.size(); ++i) {
            auto& det = detected_scene_objects[i];
            outfile << "  " << (i + 1) << ". Object: " << det.name;
            if (det.confidence >= 0.0f) {
                outfile << " (confidence: " << std::fixed << std::setprecision(2) << det.confidence << ")";
            } else {
                outfile << " (confidence: N/A)";
            }
            outfile << "\n";
            outfile << "     Predefined coords: (x=" << std::defaultfloat << det.box_x
                    << ", y=" << det.box_y << ", phi=" << det.box_phi << ")\n";
            outfile << "     Robot pose at detection: (x=" << det.robot_x
                    << ", y=" << det.robot_y << ", phi=" << det.robot_phi << ")\n";
        }
        if (!detected_apriltags.empty()) {
            outfile << "\nAprilTag Detections:\n";
            for (size_t i = 0; i < detected_apriltags.size(); ++i) {
                auto& t = detected_apriltags[i];
                outfile << "  Box " << t.box_idx << ": Tag ID=" << t.tag_id
                        << " | dist=" << t.dist << "m"
                        << " | angle=" << t.angle_deg << "deg"
                        << " | pos=(" << t.x << ", " << t.y << ", " << t.z << ")\n";
            }
        } else {
            outfile << "\nAprilTag Detections: NONE\n";
        }
        outfile << "\nTime Elapsed: " << total_seconds << " seconds\n";
        outfile.close();
        RCLCPP_INFO(node->get_logger(), "Results saved.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to open results file for writing.");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 complete. Total time: %lu seconds.", total_seconds);

  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(),
        "\n============================================\n"
        "  UNCAUGHT EXCEPTION: %s\n"
        "============================================\n"
        "  The contest loop crashed unexpectedly.\n"
        "  Check the log output above for context.\n"
        "============================================",
        e.what());
  } catch (...) {
    RCLCPP_FATAL(node->get_logger(),
        "\n============================================\n"
        "  UNCAUGHT UNKNOWN EXCEPTION\n"
        "============================================");
  }

    // Graceful shutdown: cancel executor first, then join spinner thread
    executor.cancel();
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
