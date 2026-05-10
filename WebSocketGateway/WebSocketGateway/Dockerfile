# Start the stage and name it 'socketserver' to match docker-compose targets
FROM python:3.10-slim AS socketserver

# Install system dependencies
RUN apt-get update && apt-get install -y \
  cmake \
  g++ \
  build-essential \
  git \
  python3-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app
FROM python:3.10-slim AS socketserver

RUN apt-get update && apt-get install -y cmake g++ build-essential git python3-dev pybind11-dev && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# 1. Build the C++ TankStatusProtocol 
# This path is now correct because our context is WALL_E_ROS2
COPY TankStatusProtocol /app/tankstatus
WORKDIR /app/tankstatus/builds
RUN rm -rf * && cmake .. && make

# 2. Setup the Python environment
WORKDIR /app/src
# Copy the specific Gateway folder from the new context
COPY ./WebSocketGateway /app/src/WebSocketGateway

# 3. Move the compiled library into the nested app folder
RUN find /app/tankstatus/builds -name "*.so" -exec cp {} /app/src/WebSocketGateway/WebSocketGateway/ \;

# 4. Install Python deps 
# Path adjusted to the new copy location
RUN pip install --no-cache-dir -r /app/src/WebSocketGateway/WebSocketGateway/requirements.txt || pip install fastapi uvicorn websockets

# 5. Runtime Config
ENV LD_LIBRARY_PATH="/app/src/WebSocketGateway/WebSocketGateway:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="/app/src/WebSocketGateway/WebSocketGateway:${PYTHONPATH}"

EXPOSE 8000

CMD ["uvicorn", "server:app", "--host", "0.0.0.0", "--port", "8000", "--app-dir", "/app/src/WebSocketGateway/WebSocketGateway"]
