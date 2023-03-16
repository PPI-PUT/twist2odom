# twist_to_odom
<!-- Required -->
<!-- Package description -->
Converts TwistWithCovarianceStamped to Odometry.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to twist_to_odom
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch twist_to_odom twist_to_odom.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name                    | Type                                           | Description    |
| ----------------------- | ---------------------------------------------- | -------------- |
| `twist_with_covariance` | geometry_msgs::msg::TwistWithCovarianceStamped | Vehicle twist. |


### Output

| Name       | Type                    | Description       |
| ---------- | ----------------------- | ----------------- |
| `odometry` | nav_msgs::msg::Odometry | Vehicle odometry. |


### Parameters

| Name         | Type | Description                   |
| ------------ | ---- | ----------------------------- |
| `publish_tf` | bool | Publish TF odom -> base_link. |


## References / External links
<!-- Optional -->
