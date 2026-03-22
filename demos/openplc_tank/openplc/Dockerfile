# OpenPLC Runtime v4 (Autonomy-Logic) with tank demo
# Strategy: Build runtime, then at startup use the web API to upload,
# compile and start the ST program through OpenPLC's full pipeline.

FROM debian:bookworm-slim AS matiec-builder

RUN apt-get update && apt-get install -y \
    git build-essential flex bison autoconf automake \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/beremiz/matiec.git /matiec && \
    cd /matiec && autoreconf -i && ./configure && make -j$(nproc) && make install


FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /openplc

RUN apt-get update && apt-get install -y git ca-certificates curl jq && \
    git clone --depth 1 https://github.com/Autonomy-Logic/openplc-runtime.git /openplc && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/runtime && \
    chmod +x install.sh scripts/* start_openplc.sh && \
    rm -rf build/ venvs/ .venv/ 2>/dev/null || true && \
    ./install.sh && \
    rm -rf /var/lib/apt/lists/*

# Enable OPC-UA plugin and install config
RUN sed -i 's/^opcua,\(.*\),0,0,/opcua,\1,1,0,/' plugins.conf
COPY opcua_config.json /openplc/core/src/drivers/plugins/python/opcua/opcua.json

# Copy matiec for ST -> C compilation
COPY --from=matiec-builder /usr/local/bin/iec2c /usr/local/bin/iec2c
COPY --from=matiec-builder /matiec/lib /matiec/lib

# Copy tank demo ST program
COPY tank_demo.st /openplc/programs/tank_demo.st

# Pre-compile ST -> C and place in core/generated with proper IEC lib
RUN mkdir -p /openplc/core/generated && \
    iec2c -I /matiec/lib -T /openplc/core/generated /openplc/programs/tank_demo.st && \
    cp -r /matiec/lib/C/* /openplc/core/generated/lib/ 2>/dev/null; \
    mkdir -p /openplc/core/generated/lib && \
    cp -r /matiec/lib/C/* /openplc/core/generated/lib/

# Copy hand-written glueVars that implements all required runtime symbols
COPY glueVars_tank.c /openplc/core/generated/glueVars.c

# debug.c stub
RUN cat > /openplc/core/generated/debug.c << 'EOF'
#include "iec_std_lib.h"
int __debug_tick;
void __cleanup(void) {}
void __retrieve(void) {}
void __publish(void) {}
EOF

# Empty c_blocks_code
RUN echo '// no custom C blocks' > /openplc/core/generated/c_blocks_code.cpp

# Create entrypoint
RUN cat > /entrypoint.sh << 'BASH'
#!/bin/bash
set -e
cd /openplc

# Try to compile the PLC program
echo "Compiling PLC program..."
if bash scripts/compile.sh 2>&1; then
    # Rename to pattern expected by runtime
    if [ -f build/new_libplc.so ]; then
        mv build/new_libplc.so build/libplc_tank.so
        echo "PLC program compiled successfully"
    fi
else
    echo "WARNING: Compilation failed, runtime will start without PLC program"
fi

echo "Starting OpenPLC runtime..."
exec bash start_openplc.sh
BASH
RUN chmod +x /entrypoint.sh

EXPOSE 8443 4840

CMD ["/entrypoint.sh"]
