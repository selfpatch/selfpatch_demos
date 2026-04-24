# Bridge container for the Mosaico ingestion demo.
#
# Subscribes to medkit /faults/stream SSE, downloads each fault snapshot
# bag from the gateway REST API, and ingests it into mosaicod via the
# mosaicolabs Python SDK over Apache Arrow Flight.
#
# License-safe pattern: bridge is a separate Python process that talks
# the public Apache Arrow Flight protocol to an unmodified mosaicod
# Docker image. We do not link or modify mosaicod or its Rust crates.

FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1
ENV PIP_NO_CACHE_DIR=1
ENV PIP_DISABLE_PIP_VERSION_CHECK=1

COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# Sanity check at build time: import the SDK and the LaserScan ontology
# so we fail fast if the published wheel regresses on what we need.
RUN python -c "from mosaicolabs import MosaicoClient; from mosaicolabs.ros_bridge import RosbagInjector, ROSInjectionConfig; from mosaicolabs.models.futures.laser import LaserScan; print('mosaicolabs SDK + LaserScan ontology import OK')"

WORKDIR /app
COPY bridge.py /app/bridge.py

CMD ["python", "-u", "/app/bridge.py"]
