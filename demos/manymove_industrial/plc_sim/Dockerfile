FROM python:3.12-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/plc_sim

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY plc_server.py ./

ENV PYTHONUNBUFFERED=1 \
    PLC_OPCUA_ENDPOINT=opc.tcp://0.0.0.0:4840/manymove_plc/server \
    PLC_ADMIN_PORT=8500

EXPOSE 4840 8500

HEALTHCHECK --interval=10s --timeout=3s --start-period=10s --retries=5 \
    CMD curl -fsS http://localhost:8500/health >/dev/null || exit 1

CMD ["python", "-u", "plc_server.py"]
