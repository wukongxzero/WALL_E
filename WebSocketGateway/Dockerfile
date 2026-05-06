#FROM python:3.9-slim
#FROM nvidia/cuda:12.0-base
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04


# Set environment variables to prevent .pyc files and enable buffering
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV PATH="/opt/conda/bin:${PATH}"
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN mkdir -p /app/
RUN mkdir /app/projects

RUN apt-get update && apt-get install -y ffmpeg build-essential cmake libopencv-dev pkg-config wget && rm -rf /var/lib/apt/lists/*
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p /opt/conda && \
    rm miniconda.sh && \
    conda clean -afy

COPY requirements.txt .
# Create a new environment from a yaml file (optional but recommended)



RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

RUN conda create -n myenv python=3.10 && conda clean -afy
#RUN conda install --file requirements.txt -y && conda clean -afy 

# Make RUN commands use the new environment:
SHELL ["conda", "run", "-n", "myenv", "/bin/bash", "-c"]
RUN conda run --live-stream -n myenv pip install -vvv --no-cache-dir -r requirements.txt

WORKDIR /app
#mount venv to venv in docker compose




RUN useradd -m dockerUser
# Ensure dockerUser owns the project directory
RUN chown -R dockerUser:dockerUser /app/projects

USER dockerUser
WORKDIR /app/projects

EXPOSE 8888

ENTRYPOINT ["conda", "run", "--no-capture-output", "-n", "myenv", "jupyter", "lab", "--ip=0.0.0.0", "--port=8888", "--no-browser"]
