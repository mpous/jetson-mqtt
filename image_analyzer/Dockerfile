# Use an official NVIDIA L4T base image for compatibility with Jetson Orin NX
FROM nvcr.io/nvidia/l4t-base:r32.4.4

# Set the working directory in the container
WORKDIR /usr/src/app

# Set a default region for time zone configuration and other locale settings
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Andorra

# Install Python 3 and pip
# Install Python, pip, OpenCV, and other necessary packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3 python3-pip python3-opencv tzdata python3-setuptools && \
    # Automatically set the time zone
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    # Clean up
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install paho-mqtt

# Copy the mqtt_subscriber.py script into the container
COPY mqtt_subscriber.py .

# Run the script
CMD ["python3", "mqtt_subscriber.py"]
