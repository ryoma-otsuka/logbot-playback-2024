# 1. Base Image: Use NVIDIA CUDA base for GPU support
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

# 2. Environment Variables: Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set the working directory
WORKDIR /code

# 3. System Dependencies: Install essential tools and OpenCV dependencies
# Combined RUN commands and cleared cache to minimize image size
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    python3 \
    python3-distutils \
    git \
    ffmpeg \
    libsm6 \
    libxext6 \
    && curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py \
    && python3 get-pip.py \
    && rm get-pip.py \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 4. Python Packages: Install requirements
COPY ./requirements.txt /code/
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# 5. Configuration: Copy Jupyter Notebook config
# Personal Git settings and fstab mounting are removed for portability.
# Users should mount their data volumes at runtime.
COPY jupyter_notebook_config.py /root/.jupyter/jupyter_notebook_config.py

# 6. Default Command: Launch Jupyter Lab
# Token is disabled for convenience in local environments.
# For security, consider adding a token or password in production.
CMD ["jupyter", "lab", "--ip=0.0.0.0", "--allow-root", "--port=8888", "--no-browser"]