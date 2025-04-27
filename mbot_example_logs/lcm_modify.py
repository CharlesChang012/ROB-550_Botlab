import lcm
import sys
import os
from math import sin, cos
from mbot_lcm_msgs.pose2D_t import pose2D_t  # Import your generated message type

# Define input and output directories
input_dir = "botlab"
output_dir = "botlab_w25"

# Create output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

# Define the ground truth channel name
ground_truth_channel = "GROUND_TRUTH_POSE"

def process_log_file(input_log_path, output_log_path):
    print(f"Processing {input_log_path} -> {output_log_path}")
    
    # Open the input log file
    log_in = lcm.EventLog(input_log_path, mode="r")
    if not log_in:
        print(f"Failed to open {input_log_path}")
        
    # Open the output log file
    log_out = lcm.EventLog(output_log_path, mode="w")
    if not log_out:
        print(f"Failed to open {output_log_path}")
        log_in.close()
        
    # Variables to store the offset
    offset_x = None
    offset_y = None
    offset_theta = None
    
    # Process each event in the log
    for event in log_in:
        # Write non-pose events directly to the output log
        if event.channel != ground_truth_channel:
            log_out.write_event(event.timestamp, event.channel, event.data)
            continue
            
        # Decode the pose2D_t message
        msg = pose2D_t.decode(event.data)
        
        # If this is the first ground truth pose, set the offset
        if offset_x is None:
            offset_x = msg.x
            offset_y = msg.y
            offset_theta = msg.theta
            print(f"Offset set to: x={offset_x}, y={offset_y}, theta={offset_theta}")
            
        # Adjust the pose by subtracting the offset
        adjusted_msg = pose2D_t()
        adjusted_msg.utime = msg.utime  # Keep utime unchanged
        adjusted_msg.x = msg.x - offset_x
        adjusted_msg.y = msg.y - offset_y
        adjusted_msg.theta = msg.theta - offset_theta
        
        # Rotate the trajectory according to the initial theta offset
        adjusted_msg.x, adjusted_msg.y = (adjusted_msg.x*cos(-offset_theta) + adjusted_msg.y*-sin(-offset_theta)), (adjusted_msg.x*sin(-offset_theta) + adjusted_msg.y*cos(-offset_theta))
        
        # Encode and write the adjusted message
        adjusted_data = adjusted_msg.encode()
        log_out.write_event(event.timestamp, event.channel, adjusted_data)
        
    # Close the logs
    log_in.close()
    log_out.close()
    
    print(f"Adjusted log written to {output_log_path}")

# Main function to process all log files
def main():
    # Find all .log files in the input directory
    log_files = [f for f in os.listdir(input_dir) if f.endswith('.log')]
    print(log_files)
    if not log_files:
        print(f"No .log files found in {input_dir}")
        return
    
    # Process each log file
    for log_file in log_files:
        input_path = os.path.join(input_dir, log_file)
        
        # Create output filename with _w25 suffix
        base_name = os.path.splitext(log_file)[0]
        output_file = f"{base_name}_w25.log"
        output_path = os.path.join(output_dir, output_file)
        
        # Process the log file
        process_log_file(input_path, output_path)

if __name__ == "__main__":
    main()