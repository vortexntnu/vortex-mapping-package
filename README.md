For global and local costmap generation: ros2 launch blueye_nav nav_launch.py use_sim_time:=False

For making a map: ros2 launch blueye_mapping online_async_launch.py use_sim_time:=False

    void publish_footprint_transform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";        
        transform_stamped.child_frame_id = "base_footprint";    

        // Updated translation parameters
        transform_stamped.transform.translation.x = 0.0;  
        transform_stamped.transform.translation.y = 0.0;  
        transform_stamped.transform.translation.z = 0.0;  

        // Updated rotation parameters (quaternion)
        transform_stamped.transform.rotation.x = 0.0;  
        transform_stamped.transform.rotation.y = 0.0; 
        transform_stamped.transform.rotation.z = 0.0; 
        transform_stamped.transform.rotation.w = 1.0; 

        // Send the transform
        static_tf_broadcaster_->sendTransform(transform_stamped);

    }
