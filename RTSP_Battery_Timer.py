import os
import sys
import time
import threading
from datetime import datetime

# Configure OpenCV/FFmpeg to use TCP for RTSP stream connection.
# Force lower timeout values (stimeout in microseconds: 2000000 = 2 seconds) 
# so FFmpeg internally gives up faster if the socket is closed.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|stimeout;2000000|timeout;2000000"

try:
    import cv2
    import numpy as np
except ImportError:
    print("=" * 70)
    print(" ERROR: OpenCV is not installed!")
    print(" To run this RTSP battery test viewer, please install opencv-python:")
    print("     pip install opencv-python numpy")
    print("=" * 70)
    input("\nPress Enter to exit...")
    sys.exit(1)

class VideoStreamReader:
    """
    A thread-safe, non-blocking RTSP stream reader.
    By reading frames in a separate background thread, we ensure the main GUI
    and battery timer never freeze or block, even when OpenCV/FFmpeg hangs 
    internally during stream disconnection.
    """
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.cap = None
        self.frame = None
        self.ret = False
        self.last_update_time = 0.0
        self.stopped = False
        self.connected_once = False
        self.lock = threading.Lock()
        
        # Start connection & capture thread
        self.thread = threading.Thread(target=self._update_loop, name="RTSP_Reader_Thread", daemon=True)
        self.thread.start()

    def _update_loop(self):
        # Open capture inside background thread to prevent GUI startup lag
        self.cap = cv2.VideoCapture(self.rtsp_url)
        
        while not self.stopped:
            ret, frame = self.cap.read()
            
            with self.lock:
                if self.stopped:
                    break
                self.ret = ret
                if ret:
                    self.frame = frame
                    self.last_update_time = time.time()
                    self.connected_once = True
            
            # If the stream failed to read, sleep briefly to prevent CPU spinning
            if not ret:
                time.sleep(0.01)

    def read(self):
        """Returns the last read status, frame copy, and arrival timestamp."""
        with self.lock:
            if self.frame is not None:
                return self.ret, self.frame.copy(), self.last_update_time
            return False, None, 0.0

    def stop(self):
        """Stops the thread and releases the OpenCV VideoCapture object."""
        with self.lock:
            self.stopped = True
        if self.cap is not None:
            self.cap.release()

def format_duration(seconds):
    """Formats a duration in seconds to HH:MM:SS.cc (hours, minutes, seconds, centiseconds)."""
    if seconds < 0:
        seconds = 0.0
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    c = int((seconds - int(seconds)) * 100)
    return f"{h:02d}:{m:02d}:{s:02d}.{c:02d}"

def draw_overlay(frame, text_lines, status_color, status_text, bg_alpha=0.6):
    """Draws a legible translucent telemetry overlay panel."""
    h, w, _ = frame.shape
    
    # Box dimensions
    box_width = 380
    box_height = len(text_lines) * 28 + 40
    
    x1, y1 = 15, 15
    x2, y2 = x1 + box_width, y1 + box_height
    
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (20, 20, 20), -1)
    cv2.addWeighted(overlay, bg_alpha, frame, 1.0 - bg_alpha, 0, frame)
    
    cv2.rectangle(frame, (x1, y1), (x2, y2), (60, 60, 60), 1)
    cv2.line(frame, (x1 + 10, y1 + 35), (x2 - 10, y1 + 35), (80, 80, 80), 1)
    cv2.circle(frame, (x1 + 20, y1 + 22), 6, status_color, -1)
    
    cv2.putText(frame, status_text, (x1 + 35, y1 + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA)
    
    y_offset = y1 + 62
    for line_label, line_val, val_color in text_lines:
        cv2.putText(frame, line_label, (x1 + 20, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.putText(frame, line_val, (x1 + 150, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, val_color, 1, cv2.LINE_AA)
        y_offset += 26

def draw_fullscreen_disconnected(frame, final_time_str, elapsed_seconds):
    """Draws a prominent full-screen frozen state overlay when stream is lost."""
    h, w, _ = frame.shape
    
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, h), (10, 10, 15), -1)
    cv2.addWeighted(overlay, 0.8, frame, 0.20, 0, frame)
    
    # Red border
    cv2.rectangle(frame, (20, 20), (w - 20, h - 20), (0, 0, 220), 2)
    
    # Status
    text_disconnected = "STREAM DISCONNECTED"
    text_size_disc = cv2.getTextSize(text_disconnected, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
    disc_x = (w - text_size_disc[0]) // 2
    cv2.putText(frame, text_disconnected, (disc_x, h // 2 - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
    
    # Battery Timer Info
    text_timer = f"FINAL DURATION: {final_time_str}"
    text_size_timer = cv2.getTextSize(text_timer, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
    timer_x = (w - text_size_timer[0]) // 2
    cv2.putText(frame, text_timer, (timer_x, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
    
    # Precise numerical details
    hours = elapsed_seconds / 3600.0
    text_details = f"Total Time: {elapsed_seconds:.3f} seconds ({hours:.4f} hours)"
    text_size_details = cv2.getTextSize(text_details, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0]
    details_x = (w - text_size_details[0]) // 2
    cv2.putText(frame, text_details, (details_x, h // 2 + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    
    text_action = "Timer Frozen for Battery Measurement. Press 'q' to Exit."
    text_size_action = cv2.getTextSize(text_action, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    action_x = (w - text_size_action[0]) // 2
    cv2.putText(frame, text_action, (action_x, h // 2 + 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)

def get_rtsp_url():
    """Retrieves target IP/URL from command arguments or user prompt."""
    print("=" * 70)
    print("               ESP32-P4 RTSP BATTERY TEST VIEWER (THREADED)")
    print("=" * 70)
    print("This utility monitors your wearable H.264 RTSP stream, recording")
    print("exact connected time. When the stream goes offline, the timer freezes")
    print("instantly. Timeout latency is mathematically subtracted for zero lag.")
    print("-" * 70)
    
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg.startswith("rtsp://"):
            return arg
        else:
            return f"rtsp://{arg}:554/stream"

    default_ip = "192.168.1.100"
    print("Enter the ESP32-P4 IP address or full RTSP URL.")
    user_input = input(f"Device IP or RTSP URL [Default: {default_ip}]: ").strip()
    
    if not user_input:
        return f"rtsp://{default_ip}:554/stream"
    elif user_input.startswith("rtsp://"):
        return user_input
    else:
        return f"rtsp://{user_input}:554/stream"

def main():
    rtsp_url = get_rtsp_url()
    
    print("\nStarting Threaded RTSP Capture Subsystem...")
    print(f"Target:    {rtsp_url}")
    print("Transport: TCP Interleaved (Forced Mode)")
    print("Controls:  Press 'q' in the window to exit.")
    print("-" * 70)
    
    # Initialize background reader thread
    reader = VideoStreamReader(rtsp_url)
    
    is_connected = False
    start_time = None
    elapsed_time = 0.0
    
    # Compensated duration tracks time up to the exact last frame received,
    # completely subtracting the disconnection timeout delay from the final results.
    last_frame_received_time = None
    last_valid_elapsed_time = 0.0
    
    # Timeout threshold: if no new frame is received for 2.0s, declare disconnect
    DISCONNECT_TIMEOUT = 2.0
    
    placeholder_frame = np.zeros((480, 848, 3), dtype=np.uint8)
    display_frame = placeholder_frame.copy()
    
    fps_start_time = time.time()
    fps_counter = 0
    current_fps = 0.0
    
    window_name = "ESP32-P4 RTSP Battery Life Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 848, 480)
    
    last_retry_print = time.time()
    frozen = False
    final_time_str = "00:00:00.00"
    
    try:
        while True:
            current_time = time.time()
            
            # Read latest stream status from thread-safe background buffer
            ret, frame, last_update_time = reader.read()
            
            if ret and last_update_time > 0:
                # We have a valid frame update
                if not is_connected and not frozen:
                    is_connected = True
                    start_time = current_time - elapsed_time
                    print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Stream successfully established!")
                    print("Battery life measurement timer active.")
                
                if is_connected and not frozen:
                    last_frame_received_time = last_update_time
                    elapsed_time = current_time - start_time
                    # Calculate duration exactly matching the time of this frame
                    last_valid_elapsed_time = last_frame_received_time - start_time
                    
                    fps_counter += 1
                    if current_time - fps_start_time >= 1.0:
                        current_fps = fps_counter / (current_time - fps_start_time)
                        fps_counter = 0
                        fps_start_time = current_time
                
                # Update visual display with the new frame
                display_frame = frame
                
            else:
                # No active frame returned this tick
                if is_connected:
                    # Evaluate time since last frame arrived in the background
                    time_since_last_frame = current_time - last_frame_received_time
                    
                    if time_since_last_frame >= DISCONNECT_TIMEOUT:
                        # DECLARE DISCONNECTION:
                        # Freeze the timer at last_valid_elapsed_time to eliminate latency error
                        is_connected = False
                        frozen = True
                        elapsed_time = last_valid_elapsed_time
                        final_time_str = format_duration(elapsed_time)
                        
                        # Stop reader thread to release resources
                        reader.stop()
                        
                        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Stream connection LOST!")
                        print("=" * 70)
                        print("                    CONNECTION DISCONNECTED")
                        print(f" Final streaming duration: {final_time_str}")
                        print(f" Total time in seconds:   {elapsed_time:.3f} s")
                        print(" Battery life measurement successfully frozen and logged.")
                        print("=" * 70)
                    else:
                        # We are stalling but haven't hit the full disconnect timeout yet
                        # Draw orange stalling alert borders
                        cv2.rectangle(display_frame, (10, 10), (display_frame.shape[1] - 10, display_frame.shape[0] - 10), (0, 165, 255), 2)
                        cv2.putText(display_frame, f"Connection stalling... Freezing in {DISCONNECT_TIMEOUT - time_since_last_frame:.1f}s", 
                                    (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)
                else:
                    # We are either frozen or waiting to connect initially
                    if frozen:
                        # Draw full-screen disconnected status overlay on top of the final frame
                        draw_fullscreen_disconnected(display_frame, final_time_str, elapsed_time)
                    else:
                        # Initial connection search overlay
                        display_frame = placeholder_frame.copy()
                        dots = "." * (int(current_time * 2) % 4)
                        cv2.putText(display_frame, f"Connecting to RTSP stream{dots}", (50, 200),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 1, cv2.LINE_AA)
                        cv2.putText(display_frame, f"URL: {rtsp_url}", (50, 240),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
                        cv2.putText(display_frame, "Ensure the wearable is powered and connected to the same network.", (50, 280),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1, cv2.LINE_AA)
                        
                        if current_time - last_retry_print >= 5.0:
                            print(f"[{datetime.now().strftime('%H:%M:%S')}] Attempting connection...")
                            last_retry_print = current_time
            
            # Draw telemetry overlays during active streaming
            if is_connected and not frozen:
                time_str = format_duration(elapsed_time)
                telemetry = [
                    ("Timer Elapsed:", time_str, (0, 255, 0)),
                    ("Seconds Count:", f"{elapsed_time:.2f} s", (240, 240, 240)),
                    ("Stream FPS:", f"{current_fps:.1f}", (240, 240, 240)),
                    ("Transport:", "TCP (Interleaved)", (0, 220, 225)),
                ]
                draw_overlay(display_frame, telemetry, (0, 255, 0), "CONNECTED & RECORDING")
                
                # Periodically update the console status line
                if int(elapsed_time) % 10 == 0 and int(elapsed_time) != int(elapsed_time - 0.05):
                    print(f"Running... Current duration: {format_duration(last_valid_elapsed_time)} ({last_valid_elapsed_time:.1f} s)", end="\r", flush=True)
            
            # Show standard output frame
            cv2.imshow(window_name, display_frame)
            
            # Key polling: 30ms sleep if frozen (saves CPU), 1ms during active playback
            wait_interval = 30 if frozen else 1
            key = cv2.waitKey(wait_interval) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
                
    except KeyboardInterrupt:
        print("\nExiting via user keyboard interrupt.")
        if is_connected and not frozen:
            elapsed_time = last_valid_elapsed_time
            final_time_str = format_duration(elapsed_time)
            
    finally:
        # Stop background thread and release OpenCV resources safely
        reader.stop()
        cv2.destroyAllWindows()
        
        print("\n" + "=" * 70)
        print("                           TEST SESSION SUMMARY")
        print("=" * 70)
        print(f"Target URL:          {rtsp_url}")
        print(f"Final Timer Value:   {final_time_str if frozen or not is_connected else format_duration(elapsed_time)}")
        print(f"Exact Time (Seconds): {elapsed_time:.3f} seconds")
        print(f"Status at Exit:      {'DISCONNECTED (Timer frozen for battery measurement)' if frozen or not is_connected else 'ACTIVE (Manually stopped)'}")
        print("=" * 70)
        input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()
