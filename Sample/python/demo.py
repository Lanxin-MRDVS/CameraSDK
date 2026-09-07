import time
import cv2
import numpy as np
import os
from LxCameraSDK.lx_camera_define import *
from LxCameraSDK.lx_camera_api import LxCamera

# Initialize camera
camera = LxCamera('D:\Program Files\Lanxin-MRDVS\SDK\lib\win_x64\LxCameraApi.dll')


def check_ret(_ret):
    """Check function call return status"""
    if _ret != LX_STATE.LX_SUCCESS:
        print(error_info.get(_ret, "Unknown error"))
        exit(1)

def visualize(image):
    """Visualize processed depth and amplitude images"""
    image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
    image = np.uint8(image)
    image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
    return image

def main():
    # Create save directories
    base_dir = "D:/test"
    rgb_dir = os.path.join(base_dir, "RGB")
    depth_dir = os.path.join(base_dir, "Depth")
    amp_dir = os.path.join(base_dir, "Amplitude")
    
    # Ensure directories exist
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(amp_dir, exist_ok=True)
    print(f"Images will be saved to:\nRGB: {rgb_dir}\nDepth: {depth_dir}\nAmp: {amp_dir}")

    # Set log output level and path
    camera.DcSetInfoOutput(0,True,"")

    # Get API version
    api_version = camera.DcGetApiVersion()
    print(f"API Version: {api_version}")

    # Get camera list
    success, dev_list, dev_num = camera.DcGetDeviceList()
    check_ret(success)
    print(f"Found {dev_num} devices")
    if dev_num == 0:
        print("No devices found")
        exit(1)

    # Try to open camera by SN number
    tgt_dev = None    
    for dev in dev_list:
        try:            
            # Open camera by IP address
            if camera.DcOpenDevice(LX_OPEN_MODE.OPEN_BY_IP, "192.168.100.86")[0] == LX_STATE.LX_SUCCESS:
                tgt_dev = dev
                break
        except Exception as e:
            print(f"Error processing device: {e}")
            continue


    print(f"Camera model: {tgt_dev.name.decode('utf-8').strip()}")

    # Configure 3D depth stream switch
    success = camera.DcSetBoolValue(tgt_dev.handle, LX_CAMERA_FEATURE.LX_BOOL_ENABLE_3D_DEPTH_STREAM, True)
    check_ret(success)
    # Configure 3D amplitude stream
    success = camera.DcSetBoolValue(tgt_dev.handle, LX_CAMERA_FEATURE.LX_BOOL_ENABLE_3D_AMP_STREAM, True)
    check_ret(success)
    # Configure 2D image
    success = camera.DcSetBoolValue(tgt_dev.handle, LX_CAMERA_FEATURE.LX_BOOL_ENABLE_2D_STREAM, True)
    check_ret(success)
    print("Enabled RGB, Depth and Amplitude streams")

    # Start data stream
    success = camera.DcStartStream(tgt_dev.handle)
    check_ret(success)
    print("Stream started successfully")


    # Initialize statistics variables
    rgb_time_diffs = []
    depth_time_diffs = []
    amp_time_diffs = []
    start_time = time.time()
    duration = 300.0  # Statistics duration: 5 seconds
    is_statistics_done = False  # Mark if statistics is complete

    # Middle timestamp of previous frame (initialized as None)
    last_rgb_mid_time = None
    last_depth_mid_time = None
    last_amp_mid_time = None

    try:
        print(f"Starting capture ({duration} seconds)...")
        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= duration and not is_statistics_done:
                # Calculate average frame rate
                # Depth frame rate
                if len(depth_time_diffs) > 0:
                    avg_depth_diff = sum(depth_time_diffs) / len(depth_time_diffs)
                    depth_fps = 1000 / avg_depth_diff if avg_depth_diff > 0 else 0
                else:
                    depth_fps = 0
                # Amplitude frame rate
                if len(amp_time_diffs) > 0:
                    avg_amp_diff = sum(amp_time_diffs) / len(amp_time_diffs)
                    amp_fps = 1000 / avg_amp_diff if avg_amp_diff > 0 else 0
                else:
                    amp_fps = 0
                # RGB frame rate
                if len(rgb_time_diffs) > 0:
                    avg_rgb_diff = sum(rgb_time_diffs) / len(rgb_time_diffs)
                    rgb_fps = 1000 / avg_rgb_diff if avg_rgb_diff > 0 else 0
                else:
                    rgb_fps = 0
                
                print(f"Depth: Total {len(depth_time_diffs)} frames, estimated frame rate {depth_fps:.2f} fps")
                print(f"Amp: Total {len(amp_time_diffs)} frames, estimated frame rate {amp_fps:.2f} fps")
                print(f"RGB: Total {len(rgb_time_diffs)} frames, estimated frame rate {rgb_fps:.2f} fps")
                is_statistics_done = True
                break

            # Get new frame
            success, data_ptr = camera.getFrame(tgt_dev.handle)
            if success !=  LX_STATE.LX_SUCCESS:
                print(success)

            #check_ret(success)

            # Generate middle timestamp for current frame (milliseconds)
            mid_time_ms = int(current_time * 1000)

            # Process and save depth image (including point cloud conversion)
            success, depth_image = camera.getDepthImage(data_ptr)
            if success == LX_STATE.LX_SUCCESS:
                # 1. Get point cloud using new method: first set command to get new data
                #state = camera.DcSetCmd(tgt_dev.handle, LX_CAMERA_FEATURE.LX_CMD_GET_NEW_FRAME)
                #check_ret(state)
                state, points = camera.getPointCloud(tgt_dev.handle)
                if state == LX_STATE.LX_SUCCESS and points is not None:
                    # Get depth image dimensions
                    frame_height, frame_width = depth_image.shape[:2]
                    # Example: Get 3D data at coordinate (100,100)
                    y_row, x_col = 100, 100
                    if 0 <= y_row < frame_height and 0 <= x_col < frame_width:
                        x = points[x_col, y_row, 0]
                        y = points[x_col, y_row, 1]
                        z = points[x_col, y_row, 2]
                        print(f"Point cloud data (100,100): X={x:.6f}, Y={y:.6f}, Z={z:.6f}")
                    else:
                        print("Point cloud coordinates out of image range")
                else:
                    print("Failed to get point cloud data")

                # 2. Save and display depth image
                timestamp = int(current_time * 1000)  # Millisecond timestamp
                depth_vis = visualize(depth_image)
                depth_save_path = os.path.join(depth_dir, f"depth_{timestamp}.jpg")
                #cv2.imwrite(depth_save_path, depth_vis)
                depth_show = cv2.resize(depth_vis, (640, 480))
                cv2.imshow('Depth Image', depth_show)

                # 3. After point cloud processing, calculate depth frame rate
                if last_depth_mid_time is not None:
                    time_diff = mid_time_ms - last_depth_mid_time
                    if time_diff > 0:
                        depth_time_diffs.append(time_diff)
                last_depth_mid_time = mid_time_ms

            # Get and display amplitude image
            success, amp_image = camera.getAmpImage(data_ptr)
            if success == LX_STATE.LX_SUCCESS:
                timestamp = int(current_time * 1000)  # Millisecond timestamp
                amp_vis = visualize(amp_image)
                amp_save_path = os.path.join(amp_dir, f"amp_{timestamp}.jpg")
                #cv2.imwrite(amp_save_path, amp_vis)
                # Display image
                amp_show = cv2.resize(amp_vis, (640, 480))
                cv2.imshow('Amplitude Image', amp_show)
                # Calculate frame interval
                if last_amp_mid_time is not None:
                    time_diff = mid_time_ms - last_amp_mid_time
                    if time_diff > 0:
                        amp_time_diffs.append(time_diff)
                last_amp_mid_time = mid_time_ms

            # Get and display RGB image
            success, rgb_image = camera.getRGBImage(data_ptr)
            if success == LX_STATE.LX_SUCCESS:
                timestamp = int(current_time * 1000)  # Millisecond timestamp
                rgb_save_path = os.path.join(rgb_dir, f"rgb_{timestamp}.jpg")
                #cv2.imwrite(rgb_save_path, rgb_image)
                # Display image
                rgb_show = cv2.resize(rgb_image, (640, 480))
                cv2.imshow('RGB Image', rgb_show)
                # Calculate frame interval
                if last_rgb_mid_time is not None:
                    time_diff = mid_time_ms - last_rgb_mid_time
                    if time_diff > 0:
                        rgb_time_diffs.append(time_diff)
                last_rgb_mid_time = mid_time_ms

            # Check for early exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("User quit early!\n")
                break

    finally:
        # Clean up resources
        camera.DcStopStream(tgt_dev.handle)
        camera.DcCloseDevice(tgt_dev.handle)  # Close camera device
        cv2.destroyAllWindows()
        print("\nClosed device!")

if __name__ == "__main__":
    main() 