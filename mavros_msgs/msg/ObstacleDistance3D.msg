# Obstacle_Distance_3D msg
# A copy of <a href="https://mavlink.io/en/messages/ardupilotmega.html#Obstacle distance 3D">mavlink GPS_RTK message</a>

std_msgs/Header header # timestamp.
uint8 sensor_type           # Class id of the distance sensor type.
uint8 frame                 # Coordinate frame of reference..
uint16 obstacle_id          # Unique ID given to each obstacle .
geometry_msgs/Vector3 position  # [m] X, Y, Z position of the obstacle
float32 min_distance        # [m] Minimum distance the sensor can measure.
float32 max_distance        # [m] Maximum distance the sensor can measure.
