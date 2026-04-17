# Bridge container for the Mosaico ingestion demo.
#
# Subscribes to medkit /faults/stream SSE, downloads each fault snapshot
# bag from the gateway REST API, and ingests it into mosaicod via the
# mosaicolabs Python SDK over Apache Arrow Flight.
#
# License-safe pattern: bridge is a separate Python process that talks
# the public Apache Arrow Flight protocol to an unmodified mosaicod
# Docker image. We do not link or modify mosaicod or its Rust crates.
#
# SDK pinning: PR #368 (ROS adapters for futures ontology, including
# LaserScan) merged on 2026-04-13 as commit b3867be. The subsequent
# mosaicolabs==0.3.2 PyPI wheel (2026-04-15) ships with the
# `futures` subpackage missing from the wheel despite being in source
# on main, so `pip install mosaicolabs==0.3.2` cannot import LaserScan.
# We therefore install from the upstream repo at the PR #368 merge
# commit until a PyPI release with correct packaging ships.

FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1
ENV PIP_NO_CACHE_DIR=1
ENV PIP_DISABLE_PIP_VERSION_CHECK=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ARG MOSAICO_REPO=https://github.com/mosaico-labs/mosaico.git
ARG MOSAICO_PIN=b3867be
RUN git clone "${MOSAICO_REPO}" /opt/mosaico \
    && cd /opt/mosaico \
    && git checkout "${MOSAICO_PIN}" \
    && git rev-parse HEAD > /opt/mosaico/.pinned_sha

COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt \
    && pip install /opt/mosaico/mosaico-sdk-py

# Sanity check at build time: import the SDK and the LaserScan ontology
# so we fail fast if a future pin regresses on what we need.
RUN python -c "from mosaicolabs import MosaicoClient; from mosaicolabs.ros_bridge import RosbagInjector, ROSInjectionConfig; from mosaicolabs.models.futures.laser import LaserScan; print('mosaicolabs SDK + LaserScan ontology import OK')"

WORKDIR /app
COPY bridge.py /app/bridge.py

CMD ["python", "-u", "/app/bridge.py"]
