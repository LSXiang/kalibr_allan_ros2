#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <matrix.h>
#include <mat.h>

int main(int argc, char **argv) {

  // Debug message
  RCLCPP_INFO(rclcpp::get_logger(__func__), "Starting up");

  // Check if there is a path to a dataset
  if(argc < 3) {
    RCLCPP_ERROR(
        rclcpp::get_logger(__func__), "Error please specify a rosbag file");
    RCLCPP_ERROR(
        rclcpp::get_logger(__func__),
        "Command Example: rosrun bagconvert bagconvert <rosbag> <topic>");
    return EXIT_FAILURE;
  }

  // Startup this node
  rclcpp::init(argc, argv);

  // Parse the input
  std::string pathBag = argv[1];
  std::string imuTopic = argv[2];

  // Get path
  boost::filesystem::path p(pathBag);
  std::string pathParent = p.parent_path().string();
  std::string pathMat;
  if (!pathParent.empty()) {
    pathMat = pathParent + "/" + p.stem().string() + ".mat";
  } else {
    pathMat = p.stem().string() + ".mat";
  }

  // Load rosbag here, and find messages we can play
  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(pathBag);

  // Debug
  RCLCPP_INFO(rclcpp::get_logger(__func__), "BAG Path is: %s", pathBag.c_str());
  RCLCPP_INFO(rclcpp::get_logger(__func__), "MAT Path is: %s", pathMat.c_str());
  RCLCPP_INFO(rclcpp::get_logger(__func__), "Reading in rosbag file...");

  // Create the matlab mat file
  MATFile *pmat = matOpen(pathMat.c_str(), "w");
  if (pmat == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger(__func__),
                 "Error could not create the mat file");
    return(EXIT_FAILURE);
  }

  // Our data vector
  std::vector<double> dataIMU = std::vector<double>();

  // Step through the rosbag and send to algo methods
  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
  while (bag_reader.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = bag_reader.read_next();

    // Handle IMU message
    if (msg->topic_name == imuTopic) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      sensor_msgs::msg::Imu::SharedPtr imu_msg =
          std::make_shared<sensor_msgs::msg::Imu>();

      serialization.deserialize_message(&serialized_msg, imu_msg.get());

      dataIMU.push_back(rclcpp::Time(imu_msg->header.stamp).seconds());
      dataIMU.push_back(imu_msg->linear_acceleration.x);
      dataIMU.push_back(imu_msg->linear_acceleration.y);
      dataIMU.push_back(imu_msg->linear_acceleration.z);
      dataIMU.push_back(imu_msg->angular_velocity.x);
      dataIMU.push_back(imu_msg->angular_velocity.y);
      dataIMU.push_back(imu_msg->angular_velocity.z);
    }

  }

  // Debug message
  RCLCPP_INFO(rclcpp::get_logger(__func__), "Done processing bag");

  // ====================================================================
  // ==========              IMU DATA                  ==================
  // ====================================================================
  mxArray *pa1 = mxCreateDoubleMatrix(dataIMU.size() / 7, 7, mxREAL);
  if (pa1 == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger(__func__),
                 "%s : Out of memory on line %d\n", __FILE__, __LINE__);
    RCLCPP_ERROR(rclcpp::get_logger(__func__),
                 "Unable to create mxArray.\n");
    return EXIT_FAILURE;
  }
  // Correctly copy data over (column-wise)
  double* pt1 = mxGetPr(pa1);
  for(size_t i=0; i<dataIMU.size(); i+=7) {
    pt1[i/7] = dataIMU.at(i);
    pt1[(i + dataIMU.size())/7] = dataIMU.at(i+1);
    pt1[(i + 2*dataIMU.size())/7] = dataIMU.at(i+2);
    pt1[(i + 3*dataIMU.size())/7] = dataIMU.at(i+3);
    pt1[(i + 4*dataIMU.size())/7] = dataIMU.at(i+4);
    pt1[(i + 5*dataIMU.size())/7] = dataIMU.at(i+5);
    pt1[(i + 6*dataIMU.size())/7] = dataIMU.at(i+6);
  }
  // Add it to the matlab mat file
  int status = matPutVariable(pmat, "data_imu", pa1);
  if (status != 0) {
    RCLCPP_ERROR(rclcpp::get_logger(__func__),
                 "%s :  Error using matPutVariable on line %d\n",
                 __FILE__, __LINE__);
    return EXIT_FAILURE;
  }
  // Cleanup
  mxDestroyArray(pa1);
  RCLCPP_INFO(rclcpp::get_logger(__func__), "Done processing IMU data");

  // Close the mat file
  if (matClose(pmat) != 0) {
    RCLCPP_INFO(rclcpp::get_logger(__func__), "Error closing the mat file");
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}


