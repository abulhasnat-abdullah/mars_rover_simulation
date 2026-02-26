# generate_poles_closed_box.py (MODIFIED to match target XML)
import os

# === USER SETTINGS ===

# Range of ArUco Tag IDs to generate models for
START_ID = 0
END_ID_INCLUSIVE = 50  # Generate models for IDs 0, 1, ..., 50

# --- Local Output Configuration ---
# Directory where the script will create subdirectories for each model
LOCAL_OUTPUT_DIR = "."
# Use a base name reflecting the new structure
MODEL_DIR_BASE_NAME = "aruco_pole"  # Match target naming style

# --- Gazebo Path Configuration (Used *inside* the SDF) ---
# Directory containing the textures (adjust if your structure differs)
GAZEBO_TEXTURE_DIR_NAME = "aruco_pole_textures"

# ========================

# --- SDF DIMENSIONS (METERS) ---
# Directly match the dimensions from the target XML example
POLE_RADIUS = 0.01
POLE_LENGTH = 0.345
BOX_WIDTH = 0.15    # X-dimension of the box
BOX_DEPTH = 0.15    # Y-dimension of the box (matches width in target collision)
BOX_HEIGHT = 0.20   # Z-dimension of the box (height of the side panels)
PANEL_THICKNESS = 0.001  # Thickness for all visual panels

# --- Derived Dimensions (Calculated automatically) ---
BOX_HALF_WIDTH = BOX_WIDTH / 2.0
BOX_HALF_DEPTH = BOX_DEPTH / 2.0
BOX_HALF_HEIGHT = BOX_HEIGHT / 2.0

POLE_CENTER_Z = POLE_LENGTH / 2.0  # Center of the pole geometry for its pose
BOX_BOTTOM_Z = POLE_LENGTH          # Z coordinate where the bottom of the box sits (top of pole)
BOX_CENTER_Z = BOX_BOTTOM_Z + BOX_HALF_HEIGHT  # Z coordinate of the collision box center

# Calculate visual panel poses (absolute coordinates)
PANEL_POSE_FRONT_X = BOX_HALF_WIDTH
PANEL_POSE_BACK_X = -BOX_HALF_WIDTH
PANEL_POSE_RIGHT_Y = BOX_HALF_DEPTH
PANEL_POSE_LEFT_Y = -BOX_HALF_DEPTH
PANEL_POSE_TOP_Z = BOX_CENTER_Z + BOX_HALF_HEIGHT
PANEL_POSE_BOTTOM_Z = BOX_CENTER_Z - BOX_HALF_HEIGHT

# --- Materials ---
# Material for the ArUco texture panels
PANEL_AMBIENT = "1 1 1 1"
PANEL_DIFFUSE = "1 1 1 1"
PANEL_SPECULAR = "0 0 0 1"

# Material for the Top/Bottom panels (matching target XML)
TOP_BOTTOM_AMBIENT = "0.590 0.571 0.560 1.0"
TOP_BOTTOM_DIFFUSE = "0.843 0.816 0.800 1"
TOP_BOTTOM_SPECULAR = "0.1 0.1 0.1 1"


# --- SDF Template ---
# Matches the structure of the provided target XML
SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.7">
  <!-- Auto-generated SDF for ArUco Pole with Tag {marker_id} -->
  <!-- Box visual faces are {box_width:.1f} wide/deep, {box_height:.1f} high. Box is closed top/bottom. -->
  <model name="{model_name}"> <!-- Descriptive model name -->
    <static>true</static>
    <link name="aruco_pole_link">

      <!-- === Pole === -->
      <collision name="pole_collision">
        <pose>0 0 {pole_center_z:.5f} 0 0 0</pose>
        <geometry><cylinder><radius>{pole_radius:.5f}</radius><length>{pole_length:.5f}</length></cylinder></geometry>
      </collision>
      <visual name="pole_visual">
        <pose>0 0 {pole_center_z:.5f} 0 0 0</pose>
        <geometry><cylinder><radius>{pole_radius:.5f}</radius><length>{pole_length:.5f}</length></cylinder></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
      </visual>

      <!-- === Main Marker Box COLLISION === -->
      <!-- Defines the physical boundary -->
      <collision name="box_collision">
        <pose>0 0 {box_center_z:.5f} 0 0 0</pose>
        <geometry><box><size>{box_width:.5f} {box_depth:.5f} {box_height:.5f}</size></box></geometry>
      </collision>

      <!-- === Textured Side Panels (Visuals only) === -->
      <!-- Sized and positioned to match the collision box faces -->

      <!-- Front Face Panel (+X) -->
      <visual name="box_face_front_panel">
        <pose>{panel_pose_front_x:.5f} 0 {box_center_z:.5f} 0 0 0</pose>
        <!-- Size: Thickness(X), Width(Y=Depth), Height(Z) -->
        <geometry><box><size>{panel_thickness:.5f} {box_depth:.5f} {box_height:.5f}</size></box></geometry>
        <material>
          <ambient>{panel_ambient}</ambient><diffuse>{panel_diffuse}</diffuse><specular>{panel_specular}</specular>
          <pbr><metal>
            <albedo_map>model://{gazebo_texture_dir_name}/materials/textures/{texture_filename}</albedo_map>
            <metalness>0.0</metalness><roughness>1.0</roughness>
          </metal></pbr>
        </material>
      </visual>

      <!-- Back Face Panel (-X) -->
      <visual name="box_face_back_panel">
        <pose>{panel_pose_back_x:.5f} 0 {box_center_z:.5f} 0 0 0</pose>
         <!-- Size: Thickness(X), Width(Y=Depth), Height(Z) -->
        <geometry><box><size>{panel_thickness:.5f} {box_depth:.5f} {box_height:.5f}</size></box></geometry>
         <material>
           <ambient>{panel_ambient}</ambient><diffuse>{panel_diffuse}</diffuse><specular>{panel_specular}</specular>
           <pbr><metal>
             <albedo_map>model://{gazebo_texture_dir_name}/materials/textures/{texture_filename}</albedo_map>
             <metalness>0.0</metalness><roughness>1.0</roughness>
           </metal></pbr>
         </material>
      </visual>

      <!-- Right Face Panel (+Y) -->
      <visual name="box_face_right_panel">
        <pose>0 {panel_pose_right_y:.5f} {box_center_z:.5f} 0 0 0</pose>
         <!-- Size: Width(X), Thickness(Y), Height(Z) -->
        <geometry><box><size>{box_width:.5f} {panel_thickness:.5f} {box_height:.5f}</size></box></geometry>
         <material>
           <ambient>{panel_ambient}</ambient><diffuse>{panel_diffuse}</diffuse><specular>{panel_specular}</specular>
           <pbr><metal>
             <albedo_map>model://{gazebo_texture_dir_name}/materials/textures/{texture_filename}</albedo_map>
             <metalness>0.0</metalness><roughness>1.0</roughness>
           </metal></pbr>
         </material>
      </visual>

      <!-- Left Face Panel (-Y) -->
      <visual name="box_face_left_panel">
        <pose>0 {panel_pose_left_y:.5f} {box_center_z:.5f} 0 0 0</pose>
        <!-- Size: Width(X), Thickness(Y), Height(Z) -->
        <geometry><box><size>{box_width:.5f} {panel_thickness:.5f} {box_height:.5f}</size></box></geometry>
         <material>
           <ambient>{panel_ambient}</ambient><diffuse>{panel_diffuse}</diffuse><specular>{panel_specular}</specular>
           <pbr><metal>
             <albedo_map>model://{gazebo_texture_dir_name}/materials/textures/{texture_filename}</albedo_map>
             <metalness>0.0</metalness><roughness>1.0</roughness>
           </metal></pbr>
         </material>
      </visual>

      <!-- === Top and Bottom Caps (Visuals only, No Texture) === -->

      <!-- Top Face Panel (+Z) -->
      <visual name="box_face_top_panel">
        <pose>0 0 {panel_pose_top_z:.5f} 0 0 0</pose>
        <!-- Size: Width(X), Length(Y=Depth), Thickness(Z) -->
        <geometry><box><size>{box_width:.5f} {box_depth:.5f} {panel_thickness:.5f}</size></box></geometry>
        <material>
          <ambient>{top_bottom_ambient}</ambient>
          <diffuse>{top_bottom_diffuse}</diffuse>
          <specular>{top_bottom_specular}</specular>
        </material>
      </visual>

      <!-- Bottom Face Panel (-Z) -->
      <visual name="box_face_bottom_panel">
        <pose>0 0 {panel_pose_bottom_z:.5f} 0 0 0</pose>
        <!-- Size: Width(X), Length(Y=Depth), Thickness(Z) -->
        <geometry><box><size>{box_width:.5f} {box_depth:.5f} {panel_thickness:.5f}</size></box></geometry>
         <material>
           <ambient>{top_bottom_ambient}</ambient>
           <diffuse>{top_bottom_diffuse}</diffuse>
           <specular>{top_bottom_specular}</specular>
         </material>
      </visual>

    </link>
  </model>
</sdf>
"""


if __name__ == "__main__":

    os.makedirs(LOCAL_OUTPUT_DIR, exist_ok=True)
    print(f"--- Generating SDF files with FIXED dimensions ---")
    print(f"Target Visual Box Face Size: Width/Depth={BOX_WIDTH:.3f}m, Height={BOX_HEIGHT:.3f}m")
    print(f"Pole: Radius={POLE_RADIUS:.3f}m, Length={POLE_LENGTH:.3f}m")
    print(f"Collision Box Center Height: {BOX_CENTER_Z:.3f}m")
    print(f"Generating model folders and SDF files locally in: '{LOCAL_OUTPUT_DIR}/'")
    print("-" * 30)
    print(f"Generating definitions for ArUco tags {START_ID} to {END_ID_INCLUSIVE}...")
    print("-" * 30)

    generated_count = 0
    error_count = 0

    for i in range(START_ID, END_ID_INCLUSIVE + 1):
        # Construct model directory and final model name
        model_dir_name = f"{MODEL_DIR_BASE_NAME}_{i}"
        model_name = f"{MODEL_DIR_BASE_NAME}_{i}"  # This is the name used inside the SDF <model> tag
        local_model_path = os.path.join(LOCAL_OUTPUT_DIR, model_dir_name)

        try:
            os.makedirs(local_model_path, exist_ok=True)
            texture_filename = f"aruco_tag_{i}.png"  # Texture filename convention
            print(f"Processing marker ID: {i} -> '{local_model_path}/'")

            # Format the template with all calculated and loop-specific values
            sdf_content = SDF_TEMPLATE.format(
                # Loop specific
                marker_id=i,
                texture_filename=texture_filename,
                model_name=model_name,  # Use the specific model name for this ID
                # General config
                gazebo_texture_dir_name=GAZEBO_TEXTURE_DIR_NAME,
                # Dimensions
                pole_radius=POLE_RADIUS,
                pole_length=POLE_LENGTH,
                pole_center_z=POLE_CENTER_Z,
                box_width=BOX_WIDTH,
                box_depth=BOX_DEPTH,
                box_height=BOX_HEIGHT,
                box_center_z=BOX_CENTER_Z,
                panel_thickness=PANEL_THICKNESS,
                # Panel Poses
                panel_pose_front_x=PANEL_POSE_FRONT_X,
                panel_pose_back_x=PANEL_POSE_BACK_X,
                panel_pose_right_y=PANEL_POSE_RIGHT_Y,
                panel_pose_left_y=PANEL_POSE_LEFT_Y,
                panel_pose_top_z=PANEL_POSE_TOP_Z,
                panel_pose_bottom_z=PANEL_POSE_BOTTOM_Z,
                # Materials
                panel_ambient=PANEL_AMBIENT,
                panel_diffuse=PANEL_DIFFUSE,
                panel_specular=PANEL_SPECULAR,
                top_bottom_ambient=TOP_BOTTOM_AMBIENT,
                top_bottom_diffuse=TOP_BOTTOM_DIFFUSE,
                top_bottom_specular=TOP_BOTTOM_SPECULAR
            )

            sdf_save_path = os.path.join(local_model_path, "model.sdf")
            with open(sdf_save_path, "w") as f:
                f.write(sdf_content)

            # Create model.config file
            config_content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name> <!-- Match the name in model.sdf -->
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>Generated Script</name>
    <email>user@example.com</email>
  </author>

  <description>
    An ArUco pole marker with tag ID {i}. Box visual faces are {BOX_WIDTH:.1f}m wide/deep x {BOX_HEIGHT:.1f}m high. Closed box structure.
  </description>
</model>
"""
            config_save_path = os.path.join(local_model_path, "model.config")
            with open(config_save_path, "w") as f:
                f.write(config_content)

            generated_count += 1

        except Exception as e:
            print(f"  ERROR processing marker ID {i}: {e}")
            import traceback
            traceback.print_exc()
            error_count += 1

    print("-" * 30)
    print("Finished generating SDF and config files locally.")
    print(f"  Successfully generated: {generated_count}")
    print(f"  Errors encountered: {error_count}")
    print("\nREMEMBER:")
    print(" - This script generates models with FIXED dimensions matching the target example.")
    print(f" - Box visual faces: {BOX_WIDTH:.1f}m wide/deep, {BOX_HEIGHT:.1f}m high.")
    print(" - Structure uses 1 collision box and 6 thin visual panels.")
    print("\nNEXT STEPS:")
    print("1. Ensure you have the necessary ArUco texture files (e.g., 'aruco_tag_0.png', ...)")
    print("   in a location accessible to Gazebo.")
    print("2. Manually copy all generated model folders")
    print(f"   from '{LOCAL_OUTPUT_DIR}/' (e.g., '{MODEL_DIR_BASE_NAME}_{START_ID}', ...)")
    print("   to your Gazebo models path (e.g., ~/.gz/models/ or a path listed in $GZ_SIM_RESOURCE_PATH).")
    print("3. Create the texture directory structure IN THE SAME Gazebo models path if it doesn't exist:")
    print(f"   <GAZEBO_MODELS_PATH>/{GAZEBO_TEXTURE_DIR_NAME}/materials/textures/")
    print("4. Copy your ArUco texture PNG files into that `textures` subdirectory.")
    print(f"   Example: `cp <path_to_textures>/*.png ~/.gz/models/{GAZEBO_TEXTURE_DIR_NAME}/materials/textures/`")
    print("-" * 30)
