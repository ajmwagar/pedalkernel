# ============================================================================
# PedalKernel SPICE Validation Harness
# Deterministic ngspice environment for WDF compiler validation
# ============================================================================
FROM ubuntu:24.04 AS base

LABEL maintainer="Future Present Labs"
LABEL description="Deterministic SPICE simulation environment for PedalKernel WDF compiler validation"

ENV DEBIAN_FRONTEND=noninteractive
ENV NGSPICE_VERSION=43

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    autoconf \
    automake \
    libtool \
    bison \
    flex \
    libreadline-dev \
    libfftw3-dev \
    libx11-dev \
    libxaw7-dev \
    wget \
    ca-certificates \
    python3 \
    python3-pip \
    python3-venv \
    git \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Build ngspice from source for version pinning + deterministic behavior
# ============================================================================
FROM base AS ngspice-build

WORKDIR /build

RUN wget -q "https://sourceforge.net/projects/ngspice/files/ng-spice-rework/${NGSPICE_VERSION}/ngspice-${NGSPICE_VERSION}.tar.gz/download" \
    -O ngspice-${NGSPICE_VERSION}.tar.gz \
    && tar xzf ngspice-${NGSPICE_VERSION}.tar.gz

WORKDIR /build/ngspice-${NGSPICE_VERSION}

# Configure with shared lib (for Python bindings) + no X11 (headless)
RUN ./configure \
    --prefix=/usr/local \
    --with-ngshared \
    --enable-xspice \
    --enable-cider \
    --enable-predictor \
    --disable-debug \
    && make -j$(nproc) \
    && make install

# Also build the CLI binary
RUN ./configure \
    --prefix=/usr/local \
    --enable-xspice \
    --enable-cider \
    --enable-predictor \
    --disable-debug \
    && make -j$(nproc) \
    && make install

# ============================================================================
# Final image — slim runtime
# ============================================================================
FROM base AS runtime

# Copy ngspice binaries and libs from build stage
COPY --from=ngspice-build /usr/local/bin/ngspice /usr/local/bin/ngspice
COPY --from=ngspice-build /usr/local/lib/libngspice* /usr/local/lib/
COPY --from=ngspice-build /usr/local/share/ngspice /usr/local/share/ngspice
COPY --from=ngspice-build /usr/local/lib/ngspice /usr/local/lib/ngspice

RUN ldconfig

# Python environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# Harness code
WORKDIR /harness
COPY scripts/ /harness/scripts/
COPY config/ /harness/config/
COPY circuits/ /harness/circuits/

# Golden references mount point
VOLUME /harness/golden
# WDF output mount point (from PedalKernel under test)
VOLUME /harness/wdf_output
# Results output
VOLUME /harness/results

ENV PYTHONUNBUFFERED=1
ENV NGSPICE_FIXED_TIMESTEP=1

ENTRYPOINT ["python3", "/harness/scripts/run_validation.py"]
CMD ["--suite", "all"]
