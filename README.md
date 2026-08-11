# P4VideoStreamer (ESP32-P4 + C6 RTSP Video Streamer)
*By Sam Gray*

An embedded C/C++ capture-encode-network pipeline built for the ESP32-P4-EYE platform (ESP32-P4 + ESP32-C6). The app operates as a headless, ultra-low-latency edge video streaming node, ingesting camera feeds from a MIPI-CSI sensor, encoding them via the P4 hardware H.264 encoder, and offloading real-time RTSP/RTP video streams over Wi-Fi via the ESP32-C6 coprocessor.

---

## Overview

* **MIPI-CSI Video Ingestion & Hardware Scaling:** Ingests live video from an OV2710 camera sensor via MIPI-CSI, using the hardware Pixel Processing Accelerator (PPA) for zero-CPU scaling, cropping, and colorspace conversion.
* **Hardware H.264 Encoding:** Encodes live video frames via the ESP32-P4 dedicated hardware H.264 encoder ($848 \times 480$ @ 25 FPS, 600 kbps default bitrate).
* **RTSP / RTP Streaming Engine:** Custom lightweight RTSP server (Port 554) supporting RTP over UDP unicast and RTSP TCP interleaved transport (RFC 2326 / RFC 3984).
* **ESP32-C6 Hosted Network Layer:** Offloads high-speed Wi-Fi transmission to the onboard ESP32-C6 coprocessor running at maximum TX power (20 dBm) with power-save disabled for continuous throughput.
* **Headless & Decoupled Architecture:** Bypasses display, SD storage, and local UI loops using FreeRTOS queues and true wall-clock RTP timestamps (90kHz clock) to minimize glass-to-glass streaming latency.

---

## Core Features

* **Direct Memory Access (DMA) Pipeline:** MIPI-CSI camera controller streams raw YUV video frames directly into cache-aligned PSRAM via hardware DMA, enabling zero-CPU buffer transfers to the encoder.
* **Zero-Copy Frame Processing:** Decoupled dual-core processing (Core 0 for RTSP server & network sockets, Core 1 for PPA scaling & H.264 encoding) linked via 2-element zero-copy queues.
* **Dynamic Bitrate Control:** Supports live bitrate adjustments and encoder parameter re-initialization on-the-fly without dropping client connection states.
* **RTP Payload Pacing:** Custom microsecond packet pacing (`esp_rom_delay_us`) to optimize frame distribution over the C6 SDIO bus and prevent LwIP socket congestion.
* **UDP Control & Telemetry Channel:** Secondary UDP control server for receiving haptic feedback commands.
* **Resilient Wi-Fi State Machine:** Automatic reconnection logic..


---

## Key Files

* **`app_livestream.c`**: Core RTSP server, RTP packetizer (FU-A fragmentation), H.264 hardware encoder manager, and Wi-Fi state machine.
* **`app_livestream.h`**: RTSP state definitions, stream configuration parameters, and public API declarations.
* **`app_video_stream.c`**: Camera driver interface, frame buffer allocation in PSRAM, and MIPI-CSI acquisition loop.
* **`app_video.c`**: V4L2 video device initialization, sensor configuration, and memory-mapped buffer handling.
* **`app_video_utils.c`**: Hardware PPA scaling, cropping, and YUV420 color space conversion utilities.
* **`app_control.c`**: UDP control server for remote status packets, telemetry, and system control signals.
* **`main.c`**: System entry point initializing PSRAM, network stack, camera pipeline, and FreeRTOS streaming tasks.

---

## Project Dependencies

* `espressif/esp_h264`
* `espressif/esp_video`
* `espressif/esp_wifi_remote` *(ESP32-C6 Hosted Interface)*
* `espressif/esp_cam_sensor`
* `mbedtls` *(Base64 SDP parameter encoding)*
