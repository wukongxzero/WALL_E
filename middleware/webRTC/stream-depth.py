import pyrealsense2 as rs
import numpy as np
import cv2

import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from fastapi import WebSocket, WebSocketDisconnect
from av import VideoFrame
import fractions

from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from aiortc import RTCConfiguration, RTCIceServer

#  INITIALIZE HARDWARE ONCE GLOBALLY
GLOBAL_PIPELINE = rs.pipeline()
GLOBAL_CONFIG = rs.config()
GLOBAL_CONFIG.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
GLOBAL_CONFIG.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

ice_config = RTCConfiguration(
    iceServers=[
        RTCIceServer(urls="stun:stun.l.google.com:19302"),
        RTCIceServer(urls="stun:stun1.l.google.com:19302"),
    ]
)


# Helper function to handle ALL blocking C-extension / hardware code safely in a thread
def get_realsense_depth_frame():
    try:
        frames = GLOBAL_PIPELINE.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())

        # Apply colormap on depth image (convert to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        return depth_colormap
    except Exception as e:
        print(f"Hardware grabbing error: {e}")
        return None


class CustomVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frame_count = 0

    async def recv(self):
        self.frame_count += 1
        try:
            # 🌟 Offload the entire hardware read + matrix manipulation to a worker thread
            depth_colormap = await asyncio.to_thread(get_realsense_depth_frame)

            if depth_colormap is None:
                await asyncio.sleep(0.01)  # Short throttle before retrying frame drop
                return await self.recv()

            # Build PyAV frame safely
            video_frame = VideoFrame.from_ndarray(depth_colormap, format="bgr24")
            video_frame.pts = self.frame_count
            video_frame.time_base = fractions.Fraction(1, 30)

            return video_frame
        except Exception as e:
            print(f"WebRTC track frame error: {e}")
            raise e


# FASTAPI web interface handling lifecycle cleanly
@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Lifespan started - Initializing RealSense Pipeline")
    GLOBAL_PIPELINE.start(GLOBAL_CONFIG)
    yield
    GLOBAL_PIPELINE.stop()
    print("RealSense camera pipeline stopped cleanly.")


app = FastAPI(lifespan=lifespan)
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")


@app.get("/")
async def root():
    return {"message": "Hello World"}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    pc = RTCPeerConnection(configuration=ice_config)
    video_sender = CustomVideoStreamTrack()
    pc.addTrack(video_sender)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state is {pc.connectionState}")

    try:
        while True:
            message = await websocket.receive_json()

            if message["type"] == "offer":
                print("Received offer from browser. Setting remote description...")
                offer = RTCSessionDescription(sdp=message["sdp"], type=message["type"])
                await pc.setRemoteDescription(offer)

                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)

                print("Sending answer to browser...")
                await websocket.send_json(
                    {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
                )

            elif message["type"] == "icecandidate":
                print("Received ICE candidate from browser.")

    except WebSocketDisconnect:
        print("Client disconnected from WebSocket.")
    except Exception as e:
        print(f"WebRTC Error occurred: {e}")
    finally:
        await pc.close()


@app.get("/stream", response_class=HTMLResponse)
def home(request: Request):
    return templates.TemplateResponse(
        request=request,
        name="video-stream.html",
    )
