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

import os

# 🌟 INITIALIZE WEBCAM GLOBALLY
# Index 0 is almost always the built-in laptop webcam on Linux
GLOBAL_CAP = None

ice_config = RTCConfiguration(
    iceServers=[
        RTCIceServer(urls="stun:stun.l.google.com:19302"),
        RTCIceServer(urls="stun:stun1.l.google.com:19302"),
    ]
)


class CustomVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frame_count = 0

    async def recv(self):
        self.frame_count += 1
        print(f"Sending frame {self.frame_count}")
        try:
            if GLOBAL_CAP is None or not GLOBAL_CAP.isOpened():
                print("Webcam device is not initialized or open.")
                await asyncio.sleep(0.033)  # Throttle to prevent spinning on failure
                return await self.recv()

            # 🌟 Fetch standard color frame from Vivobook webcam without blocking asyncio
            ret, frame = await asyncio.to_thread(GLOBAL_CAP.read)
            if not ret:
                print("Failed to grab frame from webcam. Retrying...")
                await asyncio.sleep(0.033)
                return await self.recv()

            # OpenCV captures frames in BGR format by default
            video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
            video_frame.pts = self.frame_count
            video_frame.time_base = fractions.Fraction(1, 30)

            return video_frame
        except Exception as e:
            print(f"Error occurred during frame capture: {e}")
            raise e

    def stop(self):
        super().stop()


# FASTAPI web interface
@asynccontextmanager
async def lifespan(app: FastAPI):
    global GLOBAL_CAP
    print("Lifespan started - Initializing Vivobook Webcam")

    # Open default video capture card
    GLOBAL_CAP = cv2.VideoCapture(0)

    # Configure capture parameters
    GLOBAL_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    GLOBAL_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    GLOBAL_CAP.set(cv2.CAP_PROP_FPS, 30)

    yield

    if GLOBAL_CAP is not None:
        GLOBAL_CAP.release()
    print("Webcam device released cleanly.")


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
