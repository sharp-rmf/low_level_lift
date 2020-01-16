#!/bin/bash
echo "publishing message to close door" 
ros2 topic pub /dry_contact_lift_sensor/door_request_topic rmf_msgs/LiftRequest "{lift_name: "LF001", session_id: "ABC", request_type: 1, destination_floor: "L1", door_state: 0}" --once
echo "Done publishing!!"
