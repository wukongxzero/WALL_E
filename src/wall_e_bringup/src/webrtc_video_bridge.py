#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import asyncio
import json
import cv2
import numpy as np
import threading
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole
import fractions

class ROS2VideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frame_ = None
        self.lock_  = threading.Lock()

    def update_frame(self, frame):
        with self.lock_:
            self.frame_ = frame

    async def recv(self):
        from av import VideoFrame
        pts, time_base = await self.next_timestamp()
        with self.lock_:
            if self.frame_ is not None:
                img = self.frame_
            else:
                img = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = VideoFrame.from_ndarray(img, format='bgr24')
        frame.pts = pts
        frame.time_base = fractions.Fraction(1, 90000)
        return frame

class WebRTCVideoBridge(Node):
    def __init__(self):
        super().__init__('webrtc_video_bridge')
        self.bridge_      = CvBridge()
        self.video_track_ = ROS2VideoTrack()
        self.pcs_         = set()

        self.create_subscription(Image,
            '/camera/color/image_raw',
            self.image_callback, 10)

        self.get_logger().info("WebRTC video bridge ready on port 8766")

    def image_callback(self, msg):
        cv_image = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        self.video_track_.update_frame(cv_image)

async def offer(request):
    node = request.app['node']
    params = await request.json()
    offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

    pc = RTCPeerConnection()
    node.pcs_.add(pc)

    pc.addTrack(node.video_track_)

    @pc.on('connectionstatechange')
    async def on_state():
        if pc.connectionState in ('failed', 'closed'):
            await pc.close()
            node.pcs_.discard(pc)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type='application/json',
        text=json.dumps({
            'sdp':  pc.localDescription.sdp,
            'type': pc.localDescription.type
        })
    )

async def run_server(node):
    app = web.Application()
    app['node'] = node
    app.router.add_post('/offer', offer)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8766)
    await site.start()
    print("WebRTC video server on port 8766")
    await asyncio.sleep(3600)

def ros2_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCVideoBridge()

    t = threading.Thread(target=ros2_thread, args=(node,), daemon=True)
    t.start()

    asyncio.run(run_server(node))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
