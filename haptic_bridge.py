#!/usr/bin/env python3
"""
Haptic Test Bridge — WebSocket ↔ UDP relay + HTTP server for the HTML test tool.

Usage:
    pip install websockets
    python haptic_bridge.py [--esp-ip 192.168.1.100] [--esp-port 8282] [--ws-port 9000]

Then open http://localhost:9000 in your browser.

The bridge:
  1. Serves haptic_test_tool.html over HTTP on the same port
  2. Accepts WebSocket connections from the HTML tool
  3. Forwards each WebSocket message as a UDP packet to the ESP32-P4
  4. Relays received UDP responses back (if any)
"""

import argparse
import asyncio
import json
import os
import socket
import sys
from pathlib import Path

try:
    import websockets
    # websockets >= 13.0 uses asyncio submodule
    from websockets.asyncio.server import serve
    NEW_WEBSOCKETS = True
except ImportError:
    try:
        from websockets.server import serve
        NEW_WEBSOCKETS = False
    except ImportError:
        print("=" * 60)
        print(" ERROR: 'websockets' package is not installed.")
        print(" Install it with:  pip install websockets")
        print("=" * 60)
        sys.exit(1)

# ── Configuration ──────────────────────────────────────────────

DEFAULT_ESP_IP = "192.168.1.100"
DEFAULT_ESP_PORT = 8282
DEFAULT_WS_PORT = 9876
DEFAULT_WS_HOST = "127.0.0.1"

# ── Globals ────────────────────────────────────────────────────

udp_sock = None
esp_addr = None
clients = set()


# ── HTTP server for the HTML file ──────────────────────────────

async def http_handler(path, request_headers):
    """Serve the HTML test tool on GET / requests."""
    # This is called by websockets' process_request hook
    pass


# ── WebSocket handler ──────────────────────────────────────────

async def ws_handler(websocket):
    """Handle a single WebSocket client connection."""
    clients.add(websocket)
    remote = websocket.remote_address
    print(f"[WS] Client connected: {remote}")

    try:
        async for message in websocket:
            # Forward every WebSocket text message as a UDP packet
            udp_sock.sendto(message.encode("utf-8"), esp_addr)
            # Echo back confirmation to all clients (for the log panel)
            ack = json.dumps({"type": "sent", "cmd": message})
            await asyncio.gather(
                *[c.send(ack) for c in clients if c.state.name == "OPEN"],
                return_exceptions=True,
            )
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.discard(websocket)
        print(f"[WS] Client disconnected: {remote}")


# ── Simple HTTP file server via websockets process_request ─────

def make_process_request(html_path):
    """Create a process_request handler that serves the HTML tool."""
    html_bytes = html_path.read_bytes()

    async def process_request(connection, request):
        if request.path == "/" or request.path == "/index.html":
            return connection.respond(200, f"OK\r\nContent-Type: text/html\r\nContent-Length: {len(html_bytes)}\r\n\r\n".encode() + html_bytes if False else "")

        # Just return None to let websockets handle WS upgrade
        return None

    return process_request


# ── Main ───────────────────────────────────────────────────────

async def main(args):
    global udp_sock, esp_addr

    esp_addr = (args.esp_ip, args.esp_port)

    # Create UDP socket for forwarding to ESP32
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setblocking(False)

    # Resolve HTML file path (same directory as this script)
    script_dir = Path(__file__).parent
    html_path = script_dir / "haptic_test_tool.html"
    if not html_path.exists():
        print(f"[ERROR] Cannot find {html_path}")
        sys.exit(1)

    html_bytes = html_path.read_bytes()
    html_content_type = "text/html; charset=utf-8"

    # Custom HTTP handler using version-compatible websockets API
    if NEW_WEBSOCKETS:
        async def process_request(connection, request):
            """Intercept HTTP requests to serve the HTML file (websockets >= 13.0)."""
            # Check if this is a WebSocket upgrade request
            is_websocket = request.headers.get("Upgrade", "").lower() == "websocket"
            if is_websocket:
                return None  # Let websockets perform the handshake

            # Serve the HTML tool for HTTP requests to / or /index.html
            if request.path in ("/", "/index.html"):
                response = connection.respond(200, html_bytes.decode("utf-8"))
                response.headers["Content-Type"] = html_content_type
                response.headers["Content-Length"] = str(len(html_bytes))
                return response

            # Return a 404 for any other HTTP requests (e.g. /favicon.ico)
            return connection.respond(404, "Not Found")
    else:
        import http
        async def process_request(path, request_headers):
            """Intercept HTTP requests to serve the HTML file (websockets < 13.0)."""
            # Check if this is a WebSocket upgrade request
            is_websocket = request_headers.get("Upgrade", "").lower() == "websocket"
            if is_websocket:
                return None  # Let websockets perform the handshake

            if path in ("/", "/index.html"):
                headers = [
                    ("Content-Type", html_content_type),
                    ("Content-Length", str(len(html_bytes)))
                ]
                return http.HTTPStatus.OK, headers, html_bytes

            return http.HTTPStatus.NOT_FOUND, [("Content-Type", "text/plain")], b"Not Found"

    print("=" * 60)
    print("       HAPTIC TEST BRIDGE")
    print("=" * 60)
    print(f"  ESP32-P4 target:  {args.esp_ip}:{args.esp_port} (UDP)")
    print(f"  WebSocket server: ws://{args.ws_host}:{args.ws_port}")
    print(f"  HTML test tool:   http://{args.ws_host}:{args.ws_port}")
    print(f"  HTML file:        {html_path}")
    print("=" * 60)
    print("  Open the URL above in your browser to start testing.")
    print("  Press Ctrl+C to stop.\n")

    async with serve(
        ws_handler,
        args.ws_host,
        args.ws_port,
        process_request=process_request,
    ) as server:
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Haptic Test Bridge")
    parser.add_argument(
        "--esp-ip",
        default=DEFAULT_ESP_IP,
        help=f"ESP32-P4 IP address (default: {DEFAULT_ESP_IP})",
    )
    parser.add_argument(
        "--esp-port",
        type=int,
        default=DEFAULT_ESP_PORT,
        help=f"ESP32-P4 UDP port (default: {DEFAULT_ESP_PORT})",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=DEFAULT_WS_PORT,
        help=f"WebSocket/HTTP port (default: {DEFAULT_WS_PORT})",
    )
    parser.add_argument(
        "--ws-host",
        default=DEFAULT_WS_HOST,
        help=f"WebSocket/HTTP bind host (default: {DEFAULT_WS_HOST})",
    )
    args = parser.parse_args()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down.")
