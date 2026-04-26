FROM python:3.11-slim

# Build context expected to be the demo root (so we can pull in artifacts/)
# rather than ota_update_server/. Compose wires this up.

WORKDIR /app
COPY ota_update_server/pyproject.toml ./
COPY ota_update_server/ota_update_server ./ota_update_server
RUN pip install --no-cache-dir .

# Bake the demo catalog + tarballs into the image so the container is
# self-contained. Bind-mounting artifacts/ at runtime is unreliable on
# WSL2 + Docker Desktop, so we ship them in the image instead.
COPY artifacts /artifacts

ENV OTA_ARTIFACTS_DIR=/artifacts
ENV OTA_HOST=0.0.0.0
ENV OTA_PORT=9000
EXPOSE 9000

CMD ["ota-update-server"]
