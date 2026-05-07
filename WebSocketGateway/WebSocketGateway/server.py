import json
import asyncio
import tankstatus_wrapper
#from physicalTankbot import PhysicalTankbot as tb

from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from ollama import AsyncClient

OLLAMA_HOST = 'http://host.docker.internal:11434'
MODEL_NAME = "gemma2:2b" 

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
            messages=[{'role': 'user', 'content': 'System initialization ping. Reply "OK".'}]
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
    chat_history = [{'role': 'system', 'content': 'You are a helpful robot named WALL-E. Keep answers brief and robotic.'}]

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
                user_text = request.get("message", str(raw_data)) if isinstance(request, dict) else str(request)
            except (json.JSONDecodeError, TypeError):
                user_text = str(raw_data)

            print(f"👤 User: {user_text}")
            chat_history.append({'role': 'user', 'content': user_text})
            
            try:
                # FIXED INDENTATION HERE
                response = await client.chat(
                    model=MODEL_NAME,
                    messages=chat_history,
                    stream=False
                )
                
                assistant_full_response = response['message']['content']

                if assistant_full_response:
                    print(f"🤖 WALL-E: {assistant_full_response}")
                    await websocket.send_text(json.dumps({
                        "type": "full_response",
                        "content": assistant_full_response
                    }))
                    
                    chat_history.append({'role': 'assistant', 'content': assistant_full_response})
                    await websocket.send_text(json.dumps({"type": "done"}))
                    print(f"🤖 Bot finished replying.")

            except Exception as e:
                print(f"❌ Ollama Error: {e}")
                await websocket.send_text(json.dumps({"type": "error", "content": "AI Core Offline."}))

    except WebSocketDisconnect:
        print("🔴 Chat Client disconnected.")
    except Exception as e:
        print(f"⚠️ Unexpected Error in Chat: {e}")

# ==========================================
# DRIVE ENDPOINT
# ==========================================
@app.websocket("/ws/drive")
async def drive_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("🟢 Drive Client connected!")
    
    tank_status = tankstatus_wrapper.TankStatusClass()

    try:
        while True:
            # Using receive() instead of receive_text() to handle Godot binary packets
            message = await websocket.receive()
            
            if "text" in message:
                raw_data = message["text"]
            elif "bytes":
                raw_data = message["bytes"].decode("utf-8")
            elif message.get("type") == "websocket.disconnect":
                break
            else:
                continue

            try:
                request = json.loads(raw_data)
                
                if "drive_right" in request:
                    tank_status.drive_right = int(request["drive_right"])
                    #tb.move(250,250);
                    await websocket.send_text(json.dumps({"type": "right"}))
                    
                if "drive_left" in request:
                    tank_status.drive_left = int(request["drive_left"])
                    #tb.move(250,250);
                    await websocket.send_text(json.dumps({"type": "left"}))
                    
                if "eulerXFloat" in request:
                    tank_status.eulerXFloat = float(request["eulerXFloat"])
                    await websocket.send_text(json.dumps({"type": "tiltX"}))

                if "eulerYFloat" in request:
                    tank_status.eulerYFloat = float(request["eulerYFloat"])
                    await websocket.send_text(json.dumps({"type": "tiltY"}))
                    
                if "eulerZFloat" in request:
                    tank_status.eulerZFloat = float(request["eulerZFloat"])
                    await websocket.send_text(json.dumps({"type": "tiltZ"}))

                packet = tank_status.make_into_bytes()
                print(f"🕹️ Drive Packet Generated: {packet}")
                
            except json.JSONDecodeError:
                print(f"⚠️ Invalid JSON on drive channel: {raw_data}")
            except Exception as e:
                print(f"❌ Error processing drive data: {e}")

    except WebSocketDisconnect:
        print("🔴 Drive Client disconnected.")

