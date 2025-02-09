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

            // Create a PointCloud2 object
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cloud->header.frame_id = "Orca/Dcam";
            cloud->height = cv_ptr->image.rows;
            cloud->width = cv_ptr->image.cols;
            cloud->is_dense = false;
            cloud->points.resize(cloud->height * cloud->width);

            // Populate the point cloud
            for (int v = 0; v < cv_ptr->image.rows; ++v)
            {
                for (int u = 0; u < cv_ptr->image.cols; ++u)
                {
                    float depth = cv_ptr->image.at<float>(v, u);
                    if (std::isnan(depth) || depth <= 0.0)
                    {
                        continue; // Skip invalid points
                    }

                    pcl::PointXYZ &pt = cloud->points[v * cv_ptr->image.cols + u];
                    pt.x = (u - cx_) * depth / fx_;
                    pt.y = (v - cy_) * depth / fy_;
                    pt.z = depth;
                }
            }

            // Convert PCL PointCloud to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.stamp = msg->header.stamp;
            output.header.frame_id = "Orca/Dcam";

            // Publish the point cloud
            point_cloud_publisher_->publish(output);

            // // ---------------------------------------------
            // // Convert the point cloud to a LaserScan message
            // // ---------------------------------------------


            // LaserScan configuration
            double angle_min_ = -M_PI / 4;  
            double angle_max_ = M_PI / 4;   
            double angle_increment_ = M_PI / 180.0; // 1-degree resolution
            double range_min_ = 0.000002;
            double range_max_ = 100000.0;
            double min_z_ = -0.05; 
            double max_z_ = 0.05;  





            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(output, *cloud);



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

                double angle = atan2(point.y, point.x);
                // if (angle < angle_min_ || angle > angle_max_) continue;

                int index = static_cast<int>((angle - angle_min_) / angle_increment_);
                double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                if (range >= range_min_ && range <= range_max_)
                {
                    ranges[index] = std::min(ranges[index], static_cast<float>(range));
                }
                // ranges[index] = std::min(ranges[index], static_cast<float>(range));
            }

            // Fill LaserScan message
            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            scan_msg->header = msg->header;
            scan_msg->header.frame_id = "Orca/Dcam";
            scan_msg->angle_min = angle_min_;
            scan_msg->angle_max = angle_max_;
            scan_msg->angle_increment = angle_increment_;
            scan_msg->range_min = range_min_;
            scan_msg->range_max = range_max_;
            scan_msg->ranges = ranges;


            laserscan_publisher_->publish(*scan_msg);
                 
        }

