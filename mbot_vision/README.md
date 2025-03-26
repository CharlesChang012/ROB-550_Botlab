# mbot_vision

## Description
A collection of computer vision examples designed for MBot.

## install
```bash
pip install ultralytics --break-system-packages
pip install --no-cache-dir "ncnn" --break-system-packages
echo 'export PYTHONPATH=$PYTHONPATH:/home/mbot/.local/bin' >> ~/.bashrc
source ~/.bashrc
```

## Files:
- `video_streamer.py`, forward video stream to browser
- `save_image.py`, used to save image for camera calibration
- `camera_calibration.py`, standard opencv code to find camera matrix and distortion coefficients
- `tag_cone_detection.py`, forward video stream to browser with apriltag and cone detection enabled
- `tag_cone_lcm_publisher.py`, publish cone lcm message over "MBOT_CONE_ARRAY" channel, and publish apriltag lcm message over "MBOT_APRILTAG_ARRAY" channel
- `tag_cone_lcm_subscriber.py`, as name stated, subscribe to both of the detections

## Authors and maintainers
The current maintainer of this project is Shaw Sun. Please direct all questions regarding support, contributions, and issues to the maintainer.