from PIL import Image
import imageio.v3 as iio
import numpy as np
import os

def create_gif_from_images_by_filename(image_dir: str, output_gif_path: str, fps: int = 10, hold_frames: dict = None) -> None:
    """
    Combines images into a GIF with customizable durations for specific frames based on filenames.

    Args:
        image_dir (str): Directory containing the images to combine.
        output_gif_path (str): Path to save the output GIF file.
        fps (int): Frames per second for the GIF.
        hold_frames (dict): Dictionary where keys are filenames (with extension) to hold, 
                            and values are durations in seconds.
    """
    # Get a sorted list of all image file paths, including subdirectories
    image_files = sorted(
        [
            os.path.join(root, file)
            for root, _, files in os.walk(image_dir)
            for file in files
            if file.endswith(('.png', '.jpg', '.jpeg'))
        ]
    )

    # Check if there are any images to process
    if not image_files:
        raise ValueError("No images found in the directory.")

    print(f"Found {len(image_files)} images.")

    # Determine the target size based on the first image
    first_image = Image.open(image_files[0])
    target_size = first_image.size  # (width, height)

    # Resize all images to the target size
    frames = []
    filenames = []
    for image_file in image_files:
        img = Image.open(image_file)
        if img.size != target_size:
            img = img.resize(target_size, Image.Resampling.LANCZOS)  # Resize to match the first image
        frames.append(np.array(img))
        filenames.append(os.path.basename(image_file))  # Store just the filename for matching

    print(f"Filenames of images: {filenames}")

    # Create durations for each frame
    default_duration = 1 / fps
    durations = [default_duration] * len(frames)

    # Adjust durations for specific filenames
    if hold_frames:
        for filename, hold_duration in hold_frames.items():
            print(f"Checking for filename: {filename} in {filenames}")
            if filename in filenames:
                index = filenames.index(filename)  # Find the index of the filename
                durations[index] = hold_duration
                print(f"Setting duration for {filename} to {hold_duration}s at index {index}")

    # Write the GIF with custom durations
    iio.imwrite(output_gif_path, frames, format='gif', duration=durations)

    print(f"GIF created at {output_gif_path}")



file_dir = os.path.dirname(os.path.realpath(__file__))
output_gif_path = os.path.join(file_dir, "output.gif")
create_gif_from_images_by_filename(file_dir, output_gif_path, hold_frames = {"X_1.png": 10, "X_2.png": 5})