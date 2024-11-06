FROM python:3.13

RUN apt-get update && \
    apt-get install -y python3-opencv libsm6 libxext6 libxrender1 libglib2.0-0 && \
    apt-get install -y v4l-utils && \
    apt-get install -y ffmpeg

RUN pip install opencv-python

WORKDIR /app

#COPY Capture_Img.py /app/Capture_Img.py
#COPY Calibrage.py /app/Calibrage.py
COPY . .

# Exécuter les scripts Python dans le répertoire de travail
CMD ["bash", "-c", "python3 /app/Calibrage.py"]