#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="16"
fi

export STAGING_ROOT="/data/safe_staging"

# Prefer the repo venv on PC/WSL so UI deps like pyray are found.
if [ -n "$DIR" ] && [ -d "$DIR/.venv/bin" ]; then
  export PATH="$DIR/.venv/bin:$PATH"
fi

# Force big UI on PC/WSL unless explicitly overridden.
if [ ! -f /TICI ] && [ -z "$BIG" ]; then
  export BIG=1
fi
