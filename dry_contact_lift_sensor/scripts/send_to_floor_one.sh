#!/bin/bash
echo "publishing message to call lift to floor 1 with Lift ID LF001" 
ros2 topic pub /dry_contact_lift_sensor/lift_request_topic rmf_msgs/LiftRequest "{lift_name: "LF001", session_id: "ABC", request_type: 1, destination_floor: "L1", door_state: 2}" --once
echo "Done publishing!!"
