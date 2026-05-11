import json
import asyncio
import tankstatus_wrapper
import time
import serial
import os
import glob
import logging
import httpx

# from physicalTankbot import PhysicalTankbot as tb
import ctypes

from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from ollama import AsyncClient


class InfoErrorFilter(logging.Filter):
    def filter(self, record):
        # Only allow INFO (20) and ERROR (40) or higher
        return record.levelno in (logging.INFO, logging.ERROR, logging.CRITICAL)


logger = logging.getLogger()
logger.setLevel(logging.INFO)
logging.getLogger("httpx").setLevel(logging.WARNING)
logging.getLogger("httpcore").setLevel(logging.WARNING)

# Apply the filter to all handlers
for handler in logger.handlers:
    handler.addFilter(InfoErrorFilter())

OLLAMA_HOST = "http://host.docker.internal:11434"
WEBUI_ENDPOINT = "http://host.docker.internal:8080/api/setTankStatus"
# Note: If gemma2:2b fails to use the tool, change this to "llama3.1"
MODEL_NAME = "llama3.1"
PORT = "/dev/ttyACM0"
PORT_PLATFORM = "/dev/ttyACM1"
BAUD = 115200
PKT_LEN = 8
CMD_HZ = 50
CMD_PERIOD = 1.0 / CMD_HZ
CENTER = 127

LOG_DIR = "/app/logs"
os.makedirs(LOG_DIR, exist_ok=True)


def get_next_log_file(directory):
    existing_files = glob.glob(os.path.join(directory, "logs*.txt"))
    indices = []
    for f in existing_files:
        try:
            # Extract 'X' from 'logsX.txt'
            name = os.path.basename(f)
            index = int(name.replace("logs", "").replace(".txt", ""))
            indices.append(index)
        except ValueError:
            continue

    next_index = max(indices) + 1 if indices else 1
    return os.path.join(directory, f"logs{next_index}.txt")


log_file_path = get_next_log_file(LOG_DIR)

# Configure Logger to print to both File and Console
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.FileHandler(log_file_path), logging.StreamHandler()],
)
logger = logging.getLogger("WALL-E")

logger.info(f"🚀 Logger initialized. Writing to {log_file_path}")


# ==========================================
# HARDWARE TOOL DEFINITION
# ==========================================
ser = serial.Serial(PORT, BAUD, timeout=0)
ser2 = serial.Serial(PORT_PLATFORM, BAUD, timeout=0)

ser.reset_input_buffer()
ser2.reset_input_buffer()


async def move_tank(left_speed: int, right_speed: int, duration=0) -> str:
    """
    Moves the tank by setting the left and right motor speeds.
    Speeds should be between 0 (full reverse) and 255 (full forward).
    duration should be how long the motors will run
    """
    ts = tankstatus_wrapper.TankStatusClass()
    ts.drive_left = ctypes.c_ubyte(left_speed).value
    ts.drive_right = ctypes.c_ubyte(right_speed).value
    packet = ts.make_into_bytes()
    ser.write(packet)

    logger.info(
        f"🕹️ Movement started, hardware called. Packet Generated: {packet.hex()}, for  {duration}"
    )
    if duration != 0:
        await asyncio.sleep(duration)
    ts.drive_left = ctypes.c_ubyte(CENTER).value
    ts.drive_right = ctypes.c_ubyte(CENTER).value
    packet = ts.make_into_bytes()

    ser.write(packet)
    logger.info(
        f"🕹️ Movement stopped, hardware called. Packet Generated: {packet.hex()}"
    )

    # --- HARDWARE EXECUTION GOES HERE ---
    # Example: tb.move(left_speed, right_speed)
    # Example: serial_port.write(packet)
    # ------------------------------------

    print(f"⚙️ EXECUTING TOOL: Motors set to L:{left_speed}, R:{right_speed}")
    print(f"🕹️ Packet Generated: {packet}")

    return f"Success! Tracks set to Left: {left_speed}, Right: {right_speed}"


move_tank_tool = {
    "type": "function",
    "function": {
        "name": "move_tank",
        "description": "Moves the robot by setting track speeds. 127 is stop, 255 is full forward, 0 is full reverse.",
        "parameters": {
            "type": "object",
            "properties": {
                "left_speed": {
                    "type": "integer",
                    "description": "Speed of left track (0-255). 0=Reverse, 127=Stop, 255=Forward.",
                    "minimum": 0,
                    "maximum": 255,
                },
                "right_speed": {
                    "type": "integer",
                    "description": "Speed of right track (0-255). 0=Reverse, 127=Stop, 255=Forward.",
                    "minimum": 0,
                    "maximum": 255,
                },
                "duration": {
                    "type": "number",
                    "description": "How long to run the motors in seconds.",
                    "minimum": 0.1,
                },
            },
            "required": ["left_speed", "right_speed", "duration"],
        },
    },
}


# ==========================================
# SERVER LIFESPAN (Runs on Startup)
# ==========================================
async def tank_status_updater():
    logger.info("🔄 Starting background Tank Status updater (60 Hz)...")

    ts = tankstatus_wrapper.TankStatusClass()
    ts.drive_left = CENTER
    ts.drive_right = CENTER

    # Use httpx.AsyncClient to efficiently reuse connections
    async with httpx.AsyncClient() as client:
        try:
            while True:
                try:
                    # 1. Generate the packet
                    # If you have real hardware telemetry, read it here.
                    # Otherwise, this sends a default/neutral packet.

                    # ts.drive_left += 1
                    # ts.drive_right -= 1
                    # ts.drive_left = ts.drive_left % 255
                    # ts.drive_right = ts.drive_right % 255
                    # so Im being very lazy, but ser is mega and ser2 is uno
                    ser = serial.Serial(PORT, 115200)

                    # Instantiate your wrapper

                    rx_status_base = tankstatus_wrapper.TankStatusClass()
                    rx_status_platform = tankstatus_wrapper.TankStatusClass()
                    rx_status_to_send = tankstatus_wrapper.TankStatusClass()

                    print("Listening to Arduino...")

                    while True:
                        # Read the exact number of bytes required for a packet
                        # Use rx_status.packetLength if exposed, otherwise hardcode your TANKSTATUS_PACKET_LENGTH
                        packet_length = 8

                        if ser.in_waiting >= packet_length:
                            # Read the binary bytes from the serial port
                            raw_bytes = ser.read(packet_length)
                            # Pass the bytes to your C++ wrapper to decode
                            rx_status_base.build_from_bytes(raw_bytes)
                            tankstatus_wrapper.TankStatusClass()
                            rx_status_base.change_flag = 1
                            rx_status_to_send.drive_left = rx_status_base.drive_left
                            rx_status_to_send.drive_right = rx_status_base.drive_right

                            ser.reset_input_buffer()

                        if ser2.in_waiting >= packet_length:
                            raw_bytes_platform = ser2.read(packet_length)
                            rx_status_platform.build_from_bytes(raw_bytes_platform)
                            rx_status_platform.change_flag = 1
                            rx_status_to_send.eulerXFloat = (
                                rx_status_platform.eulerXFloat
                            )
                            rx_status_to_send.eulerYFloat = (
                                rx_status_platform.eulerYFloat
                            )
                            rx_status_to_send.eulerZFloat = (
                                rx_status_platform.eulerZFloat
                            )

                            ser2.reset_input_buffer()

                        if (
                            rx_status_platform.change_flag == 1
                            and rx_status_base.change_flag == 1
                        ):
                            rx_status_platform.change_flag = 0
                            rx_status_base.change_flag = 0
                            packet = rx_status_to_send.make_into_bytes()
                            # logger.info(f"captured {packet}")
                            # 2. POST the binary data to the C# endpoint
                            response = await client.post(
                                WEBUI_ENDPOINT,
                                content=packet,  # 'content' is used for raw bytes in httpx
                                headers={"Content-Type": "application/octet-stream"},
                            )
                            if response.status_code != 200:
                                logger.warning(
                                    f"⚠️ WebUI returned status {response.status_code},{packet}"
                                )

                        await asyncio.sleep(0.1)
                except asyncio.CancelledError:
                    logger.info("🛑 Background task cancelled by FastAPI shutdown.")
        except Exception as e:
            logger.error(f"❌ Failed to reach WebUI: {e}")

        finally:
            if ser and ser.is_open:
                logger.info("🔌 Closing Arduino serial port...")
                ser.close()
            if ser2 and ser2.is_open:
                logger.info("🔌 Closing Arduino serial port...")
                ser2.close()

            # 3. Wait exactly .1 second before doing it again (60 time/sec)


@asynccontextmanager
async def lifespan(app: FastAPI):
    updater_task = asyncio.create_task(tank_status_updater())
    print(f"🔥 Warming up AI model '{MODEL_NAME}' into VRAM...")
    client = AsyncClient(host=OLLAMA_HOST)
    try:
        await client.chat(
            model=MODEL_NAME,
            messages=[
                {"role": "user", "content": 'System initialization ping. Reply "OK".'}
            ],
        )
        print("✅ AI Core warmed up and ready!")
    except Exception as e:
        print(f"⚠️ Failed to warm up AI: {e}")

    yield
    print("🛑 Server shutting down.")


app = FastAPI(lifespan=lifespan)


# ==========================================
# DRIVE ENDPOINT
# ==========================================
@app.websocket("/ws/drive")
async def drive_endpoint(websocket: WebSocket):
    await websocket.accept()
    # Create the object once, outside the loop
    ts = tankstatus_wrapper.TankStatusClass()
    logger.info("🚀 Tank Drive Connection Established")

    try:
        while True:
            # Add a timeout or check if the socket is still alive
            data = await websocket.receive_json()

            # Logic here...
            left = data.get("driveLeft", 127)
            right = data.get("driveRight", 127)

            ts.drive_left = left
            ts.drive_right = right
            ser.write(ts.make_into_bytes())

            logger.info(f"✅ Drive Command Processed: {left}, {right}")
            await move_tank(left, right)

    except WebSocketDisconnect:
        logger.info("🔌 Client disconnected normally.")
    except Exception as e:
        # This will tell us if it's a JSON error or something else!
        logger.error(f"❌ Unexpected Error: {type(e).__name__} - {e}")
    finally:
        # Reset motors for safety
        ts.drive_left = 127
        ts.drive_right = 127
        ser.write(ts.make_into_bytes())
        logger.info("🛑 Safety stop: Resetting motors to 127")


# ==========================================
# CHAT ENDPOINT
# ==========================================
@app.websocket("/ws/chat")
async def chat_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("🟢 Chat Client connected!")

    client = AsyncClient(host=OLLAMA_HOST)
    chat_history = [
        {
            "role": "system",
            "content": "You are a helpful robot named TankBot. You can move using your tracks. Keep answers brief and robotic.",
        }
    ]

    try:
        while True:
            message = await websocket.receive()

            if "text" in message:
                raw_data = message["text"]
            elif "bytes" in message:
                raw_data = message["bytes"].decode("utf-8")
            elif message.get("type") == "websocket.disconnect":
                break
            else:
                continue

            try:
                request = json.loads(raw_data)
                user_text = (
                    request.get("message", str(raw_data))
                    if isinstance(request, dict)
                    else str(request)
                )
            except (json.JSONDecodeError, TypeError):
                user_text = str(raw_data)

            print(f"👤 User: {user_text}")
            logger.info(f"👤 User: {user_text}")

            # add context
            ctrl_str = "when no time is specified assume duration is 1"
            chat_history.append({"role": "user", "content": user_text + ctrl_str})

            try:
                # 1. Ask Ollama, passing the tool definition
                response = await client.chat(
                    model=MODEL_NAME,
                    messages=chat_history,
                    stream=False,
                    tools=[move_tank_tool],  # <--- Tool provided here
                )

                # 2. Intercept Tool Calls (If AI decides to move)
                if response.message.tool_calls:
                    print("🤖 WALL-E is attempting to use a tool...")
                    logger.info(
                        f"🤖 WALL-E is attempting to use a tool...llama prompt:{chat_history}"
                    )

                    for tool in response.message.tool_calls:
                        if tool.function.name == "move_tank":
                            # Extract arguments safely
                            args = tool.function.arguments
                            left = args.get("left_speed", 200)
                            right = args.get("right_speed", 200)
                            duration = args.get("duration", 1)

                            # Execute the physical movement
                            tool_result = await move_tank(left, right, duration)

                            # Update history so the AI knows what happened
                            # chat_history.append(response.message)
                            # chat_history.append(
                            #    {"role": "tool", "content": tool_result}
                            # )

                    # 3. Ask Ollama to summarize what it just did
                    response = await client.chat(
                        model=MODEL_NAME, messages=chat_history, stream=False
                    )

                # 4. Extract final verbal response and send to user
                assistant_full_response = response["message"]["content"]

                if assistant_full_response:
                    print(f"🤖 WALL-E: {assistant_full_response}")
                    logger.info(f"🤖 WALL-E: {assistant_full_response}")

                    await websocket.send_text(
                        json.dumps(
                            {
                                "type": "full_response",
                                "content": assistant_full_response,
                            }
                        )
                    )

                    chat_history.append(
                        {"role": "assistant", "content": assistant_full_response}
                    )
                    await websocket.send_text(json.dumps({"type": "done"}))

                    print(f"🤖 Bot finished replying.")
                    logger.info(f"🤖 Bot finished replying.")

            except Exception as e:
                print(f"❌ Ollama Error: {e}")
                logger.error(f"❌ Ollama Error: {e}")
                await websocket.send_text(
                    json.dumps({"type": "error", "content": "AI Core Offline."})
                )

    except WebSocketDisconnect:
        print("🔴 Chat Client disconnected.")
    except Exception as e:
        print(f"⚠️ Unexpected Error in Chat: {e}")
