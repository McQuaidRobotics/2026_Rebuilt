# 360 VR Driver System - Technical Guide

This guide outlines the system architecture and implementation steps for a 360-degree VR driving system for an FRC robot using an Insta360 camera.

## 1. System Architecture

```text
[Insta360 Camera] --(USB UVC)--> [Coprocessor (Orange Pi 5)]
                                      |
                                      V
                                [WebRTC Streamer]
                                      |
                                  (Radio/Wi-Fi)
                                      |
                                      V
[VR Headset (Quest 3)] <---(WebXR Client)--- [Driver Station]
         |                                      ^
         +-----------(NetworkTables)-----------+
```

## 2. Hardware Requirements

### A. Camera: Insta360 X3 / X4

* **Mode:** Must support "Webcam" or "UVC" mode for direct USB video output.
* **Mounting:** Use a vibration-dampened mount at the highest point of the robot. Ensure the "stitch line" is not pointing directly at critical field elements.

### B. Coprocessor: Orange Pi 5 (Recommended)

* **Why:** The Rockchip RK3588 has powerful hardware encoders (H.264/H.265) capable of processing 4K 360 video with minimal latency.
* **Connection:** Connect the Insta360 to a USB 3.0 port.

### C. VR Headset: Meta Quest 2 / 3 / Pro

* **Connectivity:** Must be on the same network as the robot.

## 3. Coprocessor Software Setup

On the Orange Pi (running Ubuntu/Debian), use `webrtc-streamer` for the lowest possible latency.

1.**Install dependencies:**
    ```bash
    sudo apt-get update
    sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-bad gstreamer1.0-plugins-good
    ```

2.**Run WebRTC Streamer:**
    Download the latest release of [webrtc-streamer](https://github.com/mpromonet/webrtc-streamer) and run it pointing to the Insta360 device:
    ```bash
    ./webrtc-streamer -v /dev/video0
    ```
    *Note: `/dev/video0` is typically the UVC device name for the Insta360.*

## 4. Robot Telemetry (`VRSystem`)

The robot publishes heading data to NetworkTables under the `VRTelemetry` table. The VR client uses this to:
1.**Stabilize the View:** Keep the field "stationary" even if the robot is spinning.
2.**Overlay HUD:** Show a mini-map, battery voltage, and match time.

## 5. VR Client (WebXR)

The easiest way to view the 360 stream in VR is a simple WebXR page using **A-Frame**.

### Sample HTML (Save as `index.html` on a local server)

```html
<!DOCTYPE html>
<html>
  <head>
    <script src="https://aframe.io/releases/1.4.0/aframe.min.js"></script>
  </head>
  <body>
    <a-scene>
      <a-assets>
        <!-- The WebRTC stream URL from the coprocessor -->
        <video id="videoStream" autoplay crossorigin="anonymous" src="http://10.TE.AM.XX:8000/webrtc"></video>
      </a-assets>

      <!-- 360 Video Sphere -->
      <a-videosphere src="#videoStream" rotation="0 -90 0"></a-videosphere>

      <!-- HUD Mini-map (Optional) -->
      <a-entity id="hud" position="0 0 -2">
        <a-text value="HEADING: 0" id="headingText" position="-1 1 0" color="green"></a-text>
      </a-entity>

      <a-entity camera look-controls></a-entity>
    </a-scene>

    <script>
      // Example: Connect to NetworkTables via pynetworktables2js 
      // or use a WebSocke proxy to update the 'headingText' and stabilize rotation.
    </script>
  </body>
</html>
```

## 6. Optimization Tips

* **Bandwidth:** FRC fields limit bandwidth to 7Mbps. You MUST use H.264 or H.265 hardware encoding on the coprocessor to fit a 4K 360 stream into this limit. You may need to drop to 1080p if the network is congested.
* **Latency:** WebRTC is peer-to-peer and designed for sub-100ms latency. Ensure your coprocessor and headset are on a high-speed 5GHz or 6GHz (Wi-Fi 6E) link if possible (during practice).
* **Stitching:** The Insta360 handles stitching internally in Webcam mode, which is ideal.
