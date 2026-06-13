import json
import math
import os

# Adjust this to match your specific FRC field height configuration (e.g., 2026 REEFSCAPE or standard 8.02)
FIELD_HEIGHT_METERS = 8.042651656968106 

def flip_angle(rad_angle):
    """Flips an angle in radians over the horizontal X axis."""
    if rad_angle is None:
        return None
    # Normalize angle to (-pi, pi] and invert sign
    flipped = -rad_angle
    return math.atan2(math.sin(flipped), math.cos(flipped))

def flip_traj_file(input_path, output_path=None):
    """
    Reads a Choreo .traj file, mirrors it horizontally, and saves a new .traj file.
    If no output_path is provided, it auto-names it with a '_flipped' suffix.
    """
    if not input_path.endswith('.traj'):
        print("Warning: Input file does not have a .traj extension, but we'll try to process it anyway.")
        
    if output_path is None:
        base, ext = os.path.splitext(input_path)
        output_path = f"{base}_flipped{ext}"

    # Open and parse the .traj file as JSON data
    with open(input_path, 'r') as f:
        data = json.load(f)

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

    # 3. Flip Trajectory Samples (The physical generation paths)
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

    # Write the modified data back into a valid .traj file
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
        
    print(f"Successfully mirrored path! Saved to: {output_path}")

# Quick script execution example
if __name__ == "__main__":
    # Put this script in the same folder as your path or specify the absolute path
    flip_traj_file('C:\\Software\\FRC\\2026\\2026_Rebuilt\src\\main\\deploy\\choreo\\OP_LEFT_1.traj') 
    # This automatically outputs 'OP_LEFT_1_flipped.traj'