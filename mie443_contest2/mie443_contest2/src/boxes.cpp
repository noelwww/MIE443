#include "mie443_contest2/boxes.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

bool Boxes::load_coords() {
    std::string filePath = ament_index_cpp::get_package_share_directory("mie443_contest2") +
                           std::string("/boxes_database/coords.xml");
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if(fs.isOpened()) {
        cv::FileNode node;
        cv::FileNodeIterator it, end;
        std::vector<float> coordVec;
        std::string coords_xml[5] = {"coordinate1", "coordinate2", "coordinate3", "coordinate4",
                                     "coordinate5"};
        for(int i = 0; i < 5; ++i) {
            node = fs[coords_xml[i]];
            if(node.type() != cv::FileNode::SEQ) {
                std::cout << "XML ERROR: Data in " << coords_xml[i]
                          << " is improperly formatted - check input.xml" << std::endl;
            } else {
                it = node.begin();
                end = node.end();
                coordVec = std::vector<float>();
                for(int j = 0; it != end; ++it, ++j) {
                    coordVec.push_back((float)*it);
                }
                if(coordVec.size() == 3) {
                    coords.push_back(coordVec);
                } else {
                    std::cout << "XML ERROR: Data in " << coords_xml[i]
                              << " is improperly formatted - check input.xml" << std::endl;
                }
            }
        }
        if(coords.size() == 0) {
            std::cout << "XML ERROR: Coordinate data is improperly formatted - check input.xml"
                  << std::endl;
            return false;
        }
    } else {
        std::cout << "Could not open XML - check FilePath in " << filePath << std::endl;
        return false;
    }
    return true;
}
