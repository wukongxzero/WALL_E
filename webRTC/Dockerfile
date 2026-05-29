FROM python:3.10-slim-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
  curl \
  libusb-1.0-0-dev \
  libxcb-xinerama0 \
  libxcb1 \
  libxcb-cursor0 \
  libx11-xcb1 \
  && rm -rf /var/lib/apt/lists/*
RUN mkdir /app/
RUN mkdir -p /app/static
RUN mkdir -p /app/templates

WORKDIR /app/
COPY requirements.txt .
COPY stream-depth.py .
RUN pip install --no-cache-dir -r requirements.txt
EXPOSE 8082
EXPOSE 9999

ENTRYPOINT [ "uvicorn", "stream-depth:app", "--host", "0.0.0.0", "--port", "8082"]


