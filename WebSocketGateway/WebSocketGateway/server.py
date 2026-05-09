import json
import asyncio
import tankstatus_wrapper
# from physicalTankbot import PhysicalTankbot as tb

from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from ollama import AsyncClient

OLLAMA_HOST = "http://host.docker.internal:11434"
# Note: If gemma2:2b fails to use the tool, change this to "llama3.1"
MODEL_NAME = "llama3.1"


# ==========================================
# HARDWARE TOOL DEFINITION
# ==========================================
def move_tank(left_speed: int, right_speed: int) -> str:
    """
    Moves the tank by setting the left and right motor speeds.
    Speeds should be between -255 (full reverse) and 255 (full forward).
    """
    ts = tankstatus_wrapper.TankStatusClass()
    ts.drive_left = left_speed
    ts.drive_right = right_speed

    packet = ts.make_into_bytes()

    # --- HARDWARE EXECUTION GOES HERE ---
    # Example: tb.move(left_speed, right_speed)
    # Example: serial_port.write(packet)
    # ------------------------------------

    print(f"⚙️ EXECUTING TOOL: Motors set to L:{left_speed}, R:{right_speed}")
    print(f"🕹️ Packet Generated: {packet}")

    return f"Success! Tracks set to Left: {left_speed}, Right: {right_speed}"


# ==========================================
# SERVER LIFESPAN (Runs on Startup)
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
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
            "content": "You are a helpful robot named WALL-E. You can move using your tracks. Keep answers brief and robotic.",
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
            chat_history.append({"role": "user", "content": user_text})

            try:
                # 1. Ask Ollama, passing the tool definition
                response = await client.chat(
                    model=MODEL_NAME,
                    messages=chat_history,
                    stream=False,
                    tools=[move_tank],  # <--- Tool provided here
                )

                # 2. Intercept Tool Calls (If AI decides to move)
                if response.message.tool_calls:
                    print("🤖 WALL-E is attempting to use a tool...")

                    for tool in response.message.tool_calls:
                        if tool.function.name == "move_tank":
                            # Extract arguments safely
                            args = tool.function.arguments
                            left = args.get("left_speed", 200)
                            right = args.get("right_speed", 200)

                            # Execute the physical movement
                            tool_result = move_tank(left, right)

                            # Update history so the AI knows what happened
                            chat_history.append(response.message)
                            chat_history.append(
                                {"role": "tool", "content": tool_result}
                            )

                    # 3. Ask Ollama to summarize what it just did
                    response = await client.chat(
                        model=MODEL_NAME, messages=chat_history, stream=False
                    )

                # 4. Extract final verbal response and send to user
                assistant_full_response = response["message"]["content"]

                if assistant_full_response:
                    print(f"🤖 WALL-E: {assistant_full_response}")
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

            except Exception as e:
                print(f"❌ Ollama Error: {e}")
                await websocket.send_text(
                    json.dumps({"type": "error", "content": "AI Core Offline."})
                )

    except WebSocketDisconnect:
        print("🔴 Chat Client disconnected.")
    except Exception as e:
        print(f"⚠️ Unexpected Error in Chat: {e}")
