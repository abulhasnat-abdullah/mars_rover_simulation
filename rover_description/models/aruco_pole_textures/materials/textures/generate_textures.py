import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os

# === USER SETTINGS ===
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
MARKER_DICT = cv2.aruco.DICT_4X4_100
CANVAS_SIZE = (500, 700)        # (width, height) - adjusted to match reference ratio
SIDE_COLOR = (203, 195, 195)      # Dark gray side strips
BG_COLOR = (235, 231, 225)   # Light gray background
OUTPUT_DIR = "."  # Directory to save the signs

# --- Layout Adjustments ---
TEXT_TOP_FRACTION = 0.20      # Move text higher (original was ~0.25)
MARKER_SIZE_FRACTION = 0.85  # Slightly smaller marker relative to middle width (original was 0.9)
MARKER_BOTTOM_PADDING = 50  # Pixels of padding below the marker
# ========================

def make_sign(
    marker_id,
    text=None,
    canvas_size=CANVAS_SIZE,
    side_color=SIDE_COLOR,
    bg_color=BG_COLOR,
    font_path=FONT_PATH,
    marker_dict_type=MARKER_DICT,
    text_top_fraction=TEXT_TOP_FRACTION,
    marker_size_fraction=MARKER_SIZE_FRACTION,
    marker_bottom_padding=MARKER_BOTTOM_PADDING
):
    """Generates an ArUco sign image with text, adjusted layout."""

    if text is None:
        text = str(marker_id)

    W, H = canvas_size
    # 1) Make canvas with light gray background
    canvas = Image.new("RGB", (W, H), bg_color)

    # 2) Add dark gray side strips
    side_width = int(W * 0.125)
    middle_width = W - (2 * side_width)
    left_strip = Image.new("RGB", (side_width, H), side_color)
    canvas.paste(left_strip, (0, 0))
    right_strip = Image.new("RGB", (side_width, H), side_color)
    canvas.paste(right_strip, (W - side_width, 0))

    # 3) Add text in top part of middle area (Positioned Higher)
    try:
        # Adjust font size if needed, or keep as is
        font_size = int(H * 0.15)  # Keep font size relative to height
        font = ImageFont.truetype(font_path, font_size)

        try:
            bbox = font.getbbox(text)  # left, top, right, bottom
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            text_y_offset = -bbox[1]  # Offset for baseline alignment
        except AttributeError:
            tw, th = font.getsize(text)
            text_y_offset = 0

        # Center text horizontally
        tx = (W - tw) // 2
        # Position text vertically higher up (using text_top_fraction)
        ty = int(H * text_top_fraction) + text_y_offset  # Adjusted vertical position
        draw = ImageDraw.Draw(canvas)
        draw.text((tx, ty), text, font=font, fill=(0, 0, 0))

    except IOError:
        print(f"Error: Font file not found at {font_path}. Text will not be added.")
    except Exception as e:
        print(f"Error loading or using font: {e}. Text will not be added.")

    # 4) Generate the ArUco marker (Slightly Smaller)
    # Use marker_size_fraction for sizing
    marker_size_px = int(middle_width * marker_size_fraction)

    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(marker_dict_type)
    except AttributeError:
        print("Error: cv2.aruco module not found or incompatible. Is OpenCV Contrib installed?")
        return None
    except Exception as e:
        print(f"Error getting ArUco dictionary: {e}")
        return None

    # Create marker
    marker_img_bw = None
    try:
        if hasattr(cv2.aruco, 'generateImageMarker'):
            marker_img_bw = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
        elif hasattr(cv2.aruco, 'drawMarker'):
            marker_img_bw = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)
        else:
            print("Error: Suitable ArUco marker generation function not found.")
            return None
    except cv2.error as e:
        print(f"OpenCV Error generating marker ID {marker_id} (size {marker_size_px}): {e}")
        return None
    except Exception as e:
        print(f"Error generating marker ID {marker_id}: {e}")
        return None

    if marker_img_bw is None:
        print(f"Failed to generate marker image for ID {marker_id}.")
        return None

    # Convert marker to PIL format (white pattern on black background)
    marker_img_rgb = np.zeros((marker_size_px, marker_size_px, 3), dtype=np.uint8)
    marker_img_rgb[marker_img_bw > 0] = [255, 255, 255]
    marker_pil = Image.fromarray(marker_img_rgb)

    # Position marker with padding at the bottom
    mx = (W - marker_size_px) // 2
    # Calculate 'my' so that marker bottom edge is 'marker_bottom_padding' pixels from canvas bottom
    my = H - marker_bottom_padding - marker_size_px  # Calculate top based on bottom padding

    # Ensure marker doesn't overlap text (basic check)
    # This might need adjustment if text/marker sizes change significantly
    # text_bottom_approx = int(H * text_top_fraction) + font_size # Approximate text bottom
    # if my < text_bottom_approx + 10: # If marker top is too close to text bottom
    #      print(f"Warning: Marker may overlap text for ID {marker_id}. Consider adjusting padding/sizes.")
    # You could force 'my' lower here, but it might violate bottom padding.
    # my = text_bottom_approx + 10

    # Ensure marker is not placed above the canvas top
    if my < 0:
        print(f"Warning: Calculated marker position is too high for ID {marker_id}. Adjusting.")
        my = 0  # Place at top if calculation goes negative

    canvas.paste(marker_pil, (mx, my))

    return canvas

# Main execution block (no changes needed here from previous version)
if __name__ == "__main__":
    start_id = 0
    end_id = 50  # Inclusive

    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Created directory: {OUTPUT_DIR}")

    print(f"Generating ArUco signs for IDs {start_id} to {end_id}...")

    generated_count = 0
    error_count = 0
    for i in range(start_id, end_id + 1):
        print(f"Generating sign for ID: {i}")
        # Pass the layout parameters (or rely on defaults set above)
        sign_image = make_sign(
            marker_id=i,
            # Optional: override defaults per-image if needed
            # text_top_fraction=0.18,
            # marker_bottom_padding=60
        )

        if sign_image:
            filename = os.path.join(OUTPUT_DIR, f"aruco_tag_{i}.png")
            try:
                sign_image.save(filename)
                generated_count += 1
            except Exception as e:
                print(f"Error saving image '{filename}': {e}")
                error_count += 1
        else:
            print(f"Skipping ID {i} due to generation error.")
            error_count += 1

    print("\nGeneration complete.")
    print(f"Successfully generated: {generated_count} signs.")
    if error_count > 0:
        print(f"Errors encountered: {error_count}")
    print(f"Signs saved in directory: '{OUTPUT_DIR}'")
