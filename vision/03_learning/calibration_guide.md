# Camera Calibration Guide

## Purpose
Calibration is essential for accurate stereo vision and depth measurement. This process:
1. Removes lens distortion
2. Determines camera parameters
3. Aligns stereo images for accurate disparity calculation

## Equipment Needed
- Chessboard pattern (9x6 inner corners, 30mm squares)
- Good lighting (avoid shadows)
- Flat surface for chessboard
- Tripod (recommended for stability)

## Calibration Process

### Step 1: Capture Calibration Images
```bash
# Command to capture images (will be used on Jetson)
python3 capture_calibration.py --output_dir calibration_images --count 30

Capture 20-30 images from different angles
Ensure chessboard covers entire image area
Vary distance and orientation

Step 2: Run Calibration
# Stereo calibration command
python3 stereo_calibration.py --left_dir left_images --right_dir right_images

This will:
Detect chessboard corners
Calculate intrinsic parameters
Compute stereo rectification matrices
Save calibration data to file

Step 3: Verify Calibration

# Test rectification quality
python3 test_rectification.py --calibration_file calibration_data.yml

Check epipolar lines are horizontal
Verify reprojection error < 0.5 pixels
Test with real-world objects

Key Parameters to Save

Camera matrices (fx, fy, cx, cy)
Distortion coefficients
Rotation and translation matrices
Rectification maps

Troubleshooting
Poor corner detection: Improve lighting, use higher contrast pattern
High reprojection error: Capture more images, ensure pattern is flat
Rectification failure: Check camera alignment, recapture images


---

## Task 2: Setup Git dan GitHub (10 menit)

### Install Git untuk Windows
1. Download dari: https://git-scm.com/download/win
2. Jalankan installer dengan default settings
3. Buka Command Prompt dan verifikasi:
   ```cmd
   git --version


