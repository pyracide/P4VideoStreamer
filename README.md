# P4VideoStreamer (ESP32-P4 + C6 RTSP Video Streamer)
*By Sam Gray*

An embedded C/C++ capture-encode-network pipeline built for the ESP32-P4-EYE platform (ESP32-P4 + ESP32-C6). The app operates as a headless, low-latency edge video streamer, ingesting camera feeds from a MIPI-CSI sensor, encoding them via the P4 hardware H.264 encoder, and streaming via RTSP over Wi-Fi via the ESP32-C6 coprocessor.

---

## Overview

* **MIPI-CSI Video Capture:** Ingests live video from an OV2710 camera sensor via MIPI-CSI, using the hardware Pixel Processing Accelerator (PPA) for zero-CPU scaling, cropping, and colorspace conversion.
* **Hardware H.264 Encoding:** Encodes live video frames via the ESP32-P4 dedicated hardware H.264 encoder ($848 \times 480$ @ 25 FPS, 600 kbps default bitrate).
* **RTSP Stream:** Custom lightweight RTSP server (Port 554) supporting RTSP TCP interleaved transport.
* **ESP32-C6 Hosted Network Layer:** The P4 has no network capabilities so high-speed Wi-Fi transmission is handled by the onboard ESP32-C6 coprocessor.

---

## Core Features

* **Direct Memory Access (DMA):** Streams raw YUV video frames directly into cache-aligned PSRAM via hardware DMA (3 ping-pong buffers)
* **Zero-Copy Frame Processing:** Dual-core processing (Core 0 for RTSP server & network sockets, Core 1 for PPA scaling & H.264 encoding) linked via 2-element zero-copy queues.
* **Dynamic Bitrate Control:** Supports live bitrate adjustments and encoder re-initialization on-the-fly without dropping connection.
* **RTP Payload Pacing:** Microsecond packet pacing (`esp_rom_delay_us`) to optimize frame distribution over the C6 SDIO bus and prevent LwIP socket congestion.
* **UDP Telemetry Channel:** Secondary UDP telemetry server for receiving haptic feedback commands.
* **Resilient Wi-Fi State Machine:** Automatic reconnection logic.


---

## Key Files

* **`app_livestream.c`**: Core RTSP server, packetizer (FU-A fragmentation), H.264 hardware encoder manager, Wi-Fi state machine
* **`app_livestream.h`**: RTSP state definitions, stream parameters, public API declarations
* **`app_video_stream.c`**: Camera driver interface, frame buffer allocation in PSRAM,  MIPI-CSI video loop
* **`app_video.c`**: video device initialization, sensor configuration
* **`app_video_utils.c`**: Hardware PPA scaling, cropping, YUV420 color space conversion
* **`app_control.c`**: UDP control server for haptics
* **`main.c`**: System entry point initializing PSRAM, network stack, camera pipeline, FreeRTOS streaming tasks.

---

## Project Dependencies

* `espressif/esp_h264`
* `espressif/esp_video`
* `espressif/esp_wifi_remote` *(ESP32-C6 Hosted Interface)*
* `espressif/esp_cam_sensor`
* `mbedtls` *(Base64 SDP parameter encoding)*
