#!/usr/bin/env bash
# check_host.sh — verify a host (laptop OR server) is ready to run the
# sensing-node Docker stack. Read-only: pulls one tiny image to test GPU
# passthrough, changes nothing else. Run before `deploy.sh build`.
set -uo pipefail

pass=0; warn=0; fail=0
ok()   { printf '  \033[32mPASS\033[0m  %s\n' "$1"; pass=$((pass+1)); }
note() { printf '  \033[33mWARN\033[0m  %s\n' "$1"; warn=$((warn+1)); }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; fail=$((fail+1)); }

echo "── sensing-node host check ($(hostname)) ──────────────────────────────"

# 1. Docker engine + compose v2
if command -v docker >/dev/null 2>&1; then
  ok "docker present: $(docker --version | awk '{print $3}' | tr -d ,)"
else
  bad "docker not found — install Docker Engine";
fi
if docker compose version >/dev/null 2>&1; then
  ok "docker compose v2: $(docker compose version --short 2>/dev/null)"
else
  bad "'docker compose' (v2 plugin) not found"
fi

# 2. NVIDIA Container Toolkit + nvidia runtime
if command -v nvidia-ctk >/dev/null 2>&1; then
  ok "nvidia-ctk: $(nvidia-ctk --version 2>/dev/null | head -1 | awk '{print $NF}')"
else
  note "nvidia-ctk not found — needed for GPU in Docker (NVIDIA Container Toolkit)"
fi
if docker info 2>/dev/null | grep -qiE 'Runtimes:.*nvidia|nvidia\.com/gpu'; then
  ok "nvidia runtime / CDI configured in Docker"
else
  note "Docker doesn't list an nvidia runtime/CDI — GPU passthrough may fail"
fi

# 3. The real test: can a container see the GPU?
if command -v nvidia-smi >/dev/null 2>&1; then
  ok "host GPU: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -1)"
  echo "        (testing GPU passthrough into a container — pulls ubuntu:24.04 if absent)"
  if docker run --rm --gpus all ubuntu:24.04 nvidia-smi -L >/tmp/_gpucheck 2>&1; then
    ok "GPU visible inside container: $(grep -m1 GPU /tmp/_gpucheck || echo ok)"
  else
    bad "GPU NOT visible in container — check --gpus/runtime ($(tail -1 /tmp/_gpucheck))"
  fi
  rm -f /tmp/_gpucheck
else
  bad "nvidia-smi not found on host — install the NVIDIA driver"
fi

# 4. Disk headroom (Docker images ~15 GB + 60 GB dataset cap, PLAN §9/§10)
avail=$(df -BG --output=avail . 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -n "${avail:-}" ] && [ "$avail" -ge 80 ]; then
  ok "disk free: ${avail} GB (images + dataset cap fit)"
elif [ -n "${avail:-}" ]; then
  note "disk free: ${avail} GB — tight; images ~15 GB + datasets up to 60 GB"
else
  note "could not determine free disk"
fi

# 5. GUI role only: is there an X display to render into locally?
if [ "${1:-}" = "gui" ] || [ "${ROLE:-}" = "gui" ] || [ "${ROLE:-}" = "all" ]; then
  if [ -n "${DISPLAY:-}" ] && [ -S "/tmp/.X11-unix/X${DISPLAY##*:}" ] 2>/dev/null; then
    ok "X display ${DISPLAY} present (GUI can render locally)"
  else
    note "no usable X display (\$DISPLAY='${DISPLAY:-}') — GUI services need one;"
    note "  on a headless box, run GUI on your laptop instead (ROLE=gui there)"
  fi
fi

echo "──────────────────────────────────────────────────────────────────────"
printf 'result: \033[32m%d pass\033[0m, \033[33m%d warn\033[0m, \033[31m%d fail\033[0m\n' "$pass" "$warn" "$fail"
[ "$fail" -eq 0 ]
