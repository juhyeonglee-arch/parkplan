# Hugging Face Spaces(Docker)용. 저장소 맨 위 폴더에서: docker build -f deploy/Dockerfile .
FROM python:3.12-slim
ENV PYTHONUNBUFFERED=1 PYTHONUTF8=1 \
    PARKPLAN_SERVER=1 PARKPLAN_HOST=0.0.0.0 PORT=7860 \
    PARKPLAN_HOME=/tmp/parkplan MPLCONFIGDIR=/tmp/mpl
RUN useradd -m -u 1000 user
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY parking_ai ./parking_ai
COPY examples ./examples
USER user
EXPOSE 7860
CMD ["python", "-m", "parking_ai", "--no-browser"]
