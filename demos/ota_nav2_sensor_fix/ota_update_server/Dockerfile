FROM python:3.11-slim

WORKDIR /app
COPY pyproject.toml ./
COPY ota_update_server ./ota_update_server
RUN pip install --no-cache-dir .

ENV OTA_ARTIFACTS_DIR=/artifacts
ENV OTA_HOST=0.0.0.0
ENV OTA_PORT=9000
EXPOSE 9000

CMD ["ota-update-server"]
