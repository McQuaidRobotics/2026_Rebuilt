import json
import math
import os


FIELD_HEIGHT_METERS = 8.042651656968106 

def flip_angle(rad_angle):
    """Flips an angle in radians over the horizontal X axis."""
    if rad_angle is None:
        return None
    flipped = -rad_angle
    return math.atan2(math.sin(flipped), math.cos(flipped))

def process_single_data(data):
    """Applies the horizontal mirror transformation to the JSON data structure."""
    # 1. Flip Snapshots Waypoints
    if "snapshot" in data and "waypoints" in data["snapshot"]:
        for wp in data["snapshot"]["waypoints"]:
            wp["y"] = FIELD_HEIGHT_METERS - wp["y"]
            wp["heading"] = flip_angle(wp["heading"])

    # 2. Flip Expression Params Waypoints
    if "params" in data and "waypoints" in data["params"]:
        for wp in data["params"]["waypoints"]:
            wp["y"]["val"] = FIELD_HEIGHT_METERS - wp["y"]["val"]
            wp["heading"]["val"] = flip_angle(wp["heading"]["val"])
            
            if wp["y"]["exp"].strip().endswith("m"):
                wp["y"]["exp"] = f"{wp['y']['val']} m"
            if wp["heading"]["exp"].strip().endswith("rad"):
                wp["heading"]["exp"] = f"{wp['heading']['val']} rad"
            elif wp["heading"]["exp"].strip().endswith("deg"):
                deg_val = math.degrees(wp["heading"]["val"])
                wp["heading"]["exp"] = f"{deg_val} deg"

    # 3. Flip Trajectory Samples
    if "trajectory" in data and "samples" in data["trajectory"]:
        for sample in data["trajectory"]["samples"]:
            sample["y"] = FIELD_HEIGHT_METERS - sample["y"]
            sample["heading"] = flip_angle(sample["heading"])
            
            if "vy" in sample: sample["vy"] = -sample["vy"]
            if "ay" in sample: sample["ay"] = -sample["ay"]
            if "omega" in sample: sample["omega"] = -sample["omega"]
            if "alpha" in sample: sample["alpha"] = -sample["alpha"]
            
            if "fy" in sample and isinstance(sample["fy"], list):
                sample["fy"] = [-force for force in sample["fy"]]
                
    return data

def bulk_mirror_trajectories(directory_path="."):
    """Scans the directory for *_LEFT*.traj files and generates missing *_RIGHT*.traj twins."""
    print(f"Scanning directory: {os.path.abspath(directory_path)}")
    
    # List all files in the target directory
    all_files = os.listdir(directory_path)
    traj_files = [f for f in all_files if f.endswith('.traj')]
    
    left_files_processed = 0
    new_files_generated = 0

    for file_name in traj_files:
        # Check if the file is a 'LEFT' trajectory
        if "LEFT" in file_name:
            left_files_processed += 1
            
            # Determine the expected name for the 'RIGHT' twin
            right_file_name = file_name.replace("LEFT", "RIGHT")
            
            # Paths to the actual files
            left_path = os.path.join(directory_path, file_name)
            right_path = os.path.join(directory_path, right_file_name)
            
            # Check if the right file already exists
            if os.path.exists(right_path):
                print(f"-> Skipped: {right_file_name} already exists.")
                continue
                
            print(f"-> Mirroring: {file_name} ---> {right_file_name}")
            
            try:
                # Read left data
                with open(left_path, 'r') as f:
                    data = json.load(f)
                
                # Internal name updates inside the file structure if applicable
                if "name" in data and "LEFT" in data["name"]:
                    data["name"] = data["name"].replace("LEFT", "RIGHT")
                
                # Transform data
                mirrored_data = process_single_data(data)
                
                # Write to the new right file
                with open(right_path, 'w') as f:
                    json.dump(mirrored_data, f, indent=2)
                    
                new_files_generated += 1
                
            except Exception as e:
                print(f"❌ Error processing {file_name}: {e}")

    print("\n--- Summary ---")
    print(f"Found 'LEFT' paths: {left_files_processed}")
    print(f"Newly generated 'RIGHT' paths: {new_files_generated}")

if __name__ == "__main__":
    # Run in current folder. Change '.' to a subfolder string like 'deploy/choreo' if needed.
    bulk_mirror_trajectories('.')