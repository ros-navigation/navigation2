// (check if same as before to pass, density param, route frame param for population, node to store, publish 'path' itself)

    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = route_frame_;