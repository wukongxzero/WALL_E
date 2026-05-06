from ollama import Client
import asyncio
import websockets

import asyncio
import websockets

async def echo(websocket):
    # Iterate over incoming messages
    async for message in websocket:
        print(f"Received: {message}")
        await websocket.send(f"Echo: {message}")

async def main():
    # Start the server on localhost at port 8765
    async with websockets.serve(echo, "localhost", 8765):
        print("Server started on ws://localhost:8765")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())


# Replace '192.168.1.100' with your actual server IP address
client = Client(host='http://192.168.1.100:11434')

response = client.chat(model='gemma3', messages=[
  {
    'role': 'user',
    'content': 'Why is the sky blue?',
  },
])

print(response['message']['content'])

