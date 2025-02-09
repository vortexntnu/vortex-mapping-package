For global and local costmap generation: ros2 launch blueye_nav nav_launch.py use_sim_time:=False

For making a map: ros2 launch blueye_mapping online_async_launch.py use_sim_time:=False





void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
        {
            if (!camera_info_received_)
            {
                RCLCPP_WARN(this->get_logger(), "Camera info not yet received. Skipping depth image processing.");
                return;
            }

            // Convert ROS Image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }


            int width = cv_ptr->image.cols;
            int height = cv_ptr->image.rows;
            int mid_row = height / 2; // Get middle row index

            // Create a PointCloud2 object
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cloud->header.frame_id = "base_link";
            cloud->height = cv_ptr->image.rows;
            cloud->width = cv_ptr->image.cols;
            cloud->is_dense = false;
            cloud->points.resize(cloud->height * cloud->width);


            // // Create LaserScan message
            // auto laser_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
            // laser_scan->header.stamp = msg->header.stamp;
            // laser_scan->header.frame_id = "base_link";
            // laser_scan->range_min = 0.1;
            // laser_scan->range_max = 10.0; // Adjust based on sensor capabilities
            // laser_scan->ranges.resize(width, std::numeric_limits<float>::quiet_NaN());

            // Populate the point cloud
            // for (int v = 0; v < cv_ptr->image.rows; ++v)
            // {
            //     for (int u = 0; u < cv_ptr->image.cols; ++u)
            //     {
            //         float depth = cv_ptr->image.at<float>(v, u);
            //         if (std::isnan(depth) || depth <= 0.0)
            //         {
            //             continue; // Skip invalid points
            //         }

            //         pcl::PointXYZ &pt = cloud->points[v * cv_ptr->image.cols + u];
            //         pt.x = (u - cx_) * depth / fx_;
            //         pt.y = (v - cy_) * depth / fy_;
            //         pt.z = depth;
            //     }

                
            // }


            // Extract points from the middle row and convert to LaserScan
            for (int u = 0; u < width; ++u)
            {
                float depth = cv_ptr->image.at<float>(mid_row, u);
                if (std::isnan(depth) || depth <= 0.0)
                {
                    continue; // Skip invalid points
                }

                pcl::PointXYZ &pt = cloud->points[u];
                pt.x = (u - cx_) * depth / fx_;
                pt.y = (mid_row - cy_) * depth / fy_;
                pt.z = depth;

                // laser_scan->ranges[u] = std::sqrt(pt.z * pt.z); // Use existing coordinates
                // laser_scan->ranges[u] = pt.x/cos(atan2(pt.y, pt.x));
                // laser_scan->angles
            }





            // cv::Mat depth_image = cv_ptr->image;

            // // int width = depth_image.cols;
            // // int height = depth_image.rows;

            // // Define LaserScan parameters
            // double angle_min = -M_PI / 4.0; // -45 degrees
            // double angle_max = M_PI / 4.0;  // 45 degrees
            // int num_rays = 180;              // Number of beams
            // std::vector<float> scan_ranges(num_rays, std::numeric_limits<float>::infinity());

            // // Step through the depth image to convert it to LaserScan data
            // int row = height / 2;  // Process middle row (you can modify this for more rows)

            // for (int i = 0; i < num_rays; ++i) {
            //     int x = static_cast<int>((i / static_cast<double>(num_rays - 1)) * (width - 1));
            //     float depth = depth_image.at<float>(row, x) * 0.001f; // Convert mm to meters (if needed)
            //     if (depth > 0.1f && depth < 10.0f) {  // Only use valid depth values
            //         float angle = angle_min + (i / static_cast<double>(num_rays - 1)) * (angle_max - angle_min);
            //         // scan_ranges[i] = depth;  // Store the depth value for the laser scan
                
            //     for (int u = 0; u < width; ++u)
            //         {
            //             pcl::PointXYZ &pt = cloud->points[u];
            //             pt.x = (u - cx_) * depth / fx_;
            //             pt.y = (mid_row - cy_) * depth / fy_;
            //             pt.z = depth;
            //             scan_ranges[i] =  std::sqrt(pt.z * pt.z + pt.y * pt.y);
            //         }
            //     }
            // }





            // Convert PCL PointCloud to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.stamp = msg->header.stamp;
            output.header.frame_id = "base_link";

            // Publish the point cloud
            point_cloud_publisher_->publish(output);

            // // ---------------------------------------------
            // // Convert the point cloud to a LaserScan message
            // // ---------------------------------------------








            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(output, *cloud);










            // LaserScan configuration
            double angle_min_ = -M_PI / 2;  
            double angle_max_ = M_PI / 2;   
            double angle_increment_ = M_PI / 180.0; // 1-degree resolution
            double range_min_ = 0.02;
            double range_max_ = 100.0;
            double min_z_ = -0.05; 
            double max_z_ = 0.05;  



            // Create the LaserScan message
            auto laser_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
            laser_scan->header.stamp = msg->header.stamp;
            laser_scan->header.frame_id = "base_link";
            laser_scan->angle_min = angle_min_;
            laser_scan->angle_max = angle_max_;
            laser_scan->angle_increment = (angle_max_ - angle_min_) / 179;
            laser_scan->range_min = 0.1;
            laser_scan->range_max = 10.0;
            // laser_scan->ranges = scan_ranges;
            laser_scan->ranges.clear();



            // // Loop over each point in the point cloud
            // for (float angle = laser_scan->angle_min; angle <= laser_scan->angle_max; angle += laser_scan->angle_increment)
            // {
            
            //     float min_range = laser_scan->range_max;
            //     bool point_found = false;

            //     // Loop through the points in the cloud and find the closest one for the current angle
            //     for (const auto &point : cloud->points)
            //     {
            //         float angle_to_point = std::atan2(point.z, point.y);  // Angle from the robot to the point
            //         if (angle_to_point >= angle - (laser_scan->angle_increment / 2) && angle_to_point <= angle + (laser_scan->angle_increment / 2))
            //         {
            //             float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            //             if (distance < min_range)
            //             {
            //                 min_range = distance;
            //                 point_found = true;
                            
            //             }
            //         }
            //     }

            //     // If no point is found in the scan range, set the range to Inf (indicating no obstacle)
            //     if (!point_found)
            //     {
            //         laser_scan->ranges.push_back(std::numeric_limits<float>::infinity());
            //     }
            //     else
            //     {
            //         laser_scan->ranges.push_back(min_range);
            //     }
            // }





            // if (cloud->points.empty())
            // {
            //     RCLCPP_WARN(this->get_logger(), "Converted PointCloud2 contains no valid points!");
            //     return;
            // }
            // for (size_t i = 0; i < cloud->points.size(); ++i)
            // {
            //     const auto &point = cloud->points[i];

            //     // Print only valid points where x, y, and z are above 0
            //     if (point.x > 0.0 && point.y > 0.0 && point.z > 0.0)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "Valid Point %ld: x=%.2f, y=%.2f, z=%.2f", 
            //                     i, point.x, point.y, point.z);
            //     }
            // }

            // for (size_t i = 0; i < output.data.size(); i += 16) // Each point has 16 bytes
            // {
            //     float* point = reinterpret_cast<float*>(&output.data[i]);

            //     float x = point[0];  // x coordinate
            //     float y = point[1];  // y coordinate
            //     float z = point[2];  // z coordinate

            //     // Print valid points where x, y, and z are above 0
            //     if (x > 0.0 && y > 0.0 && z > 0.0)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "cloud Valid Point %ld: x=%.2f, y=%.2f, z=%.2f", 
            //                     i / 16, x, y, z);
            //     }
            // }

            // Compute the number of rays
            int num_rays = static_cast<int>((angle_max_ - angle_min_) / angle_increment_);
            std::vector<float> ranges(num_rays, std::numeric_limits<float>::infinity());

            for (const auto &point : cloud->points)
            {
                
                // if (point.z < min_z_ || point.z > max_z_) continue; // Filter by height

                // double angle = atan2(point.y, point.x);
                double angle = atan2(point.x, point.z);
                // if (angle < angle_min_ || angle > angle_max_) continue;

                int index = static_cast<int>((angle - angle_min_) / angle_increment_);
                double range = sqrt(point.x * point.x + point.z * point.z);
                // if(range > 0.00) RCLCPP_INFO(this->get_logger(), "cloud Valid Point %.2f, angle: %.2f", point.x, point.z);

                if (range >= range_min_ && range <= range_max_)
                {
                    ranges[index] = std::min(ranges[index], static_cast<float>(range));
                }
                // ranges[index] = std::min(ranges[index], static_cast<float>(range));
                // ranges[index] = std::min(ranges[index], static_cast<float>(point.x/np.cos(angle)));
            }

            // Fill LaserScan message
            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            scan_msg->header = msg->header;
            scan_msg->header.frame_id = "base_link";
            scan_msg->angle_min = angle_min_;
            scan_msg->angle_max = angle_max_;
            scan_msg->angle_increment = angle_increment_;
            scan_msg->range_min = range_min_;
            scan_msg->range_max = range_max_;
            scan_msg->ranges = laser_scan->ranges;


            laser_scan->ranges = ranges;

            // std::stringstream ss;
            // ss << "LaserScan Ranges: ";
            // for (float range : scan_msg->ranges) {
            //     ss << range << " ";
            // }
            // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());


            

            // Publish LaserScan message
            // scan_pub_->publish(*scan_msg);
            laserscan_publisher_->publish(*laser_scan);
            // RCLCPP_INFO(this->get_logger(), "Published LaserScan");






            
        }
