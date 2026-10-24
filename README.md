# xiaozhi-table-pet (firmware rewrite, P1 sketch)

ESP-IDF v5.5.x, target esp32s3. Spec: `FIRMWARE-REWRITE-SPEC.md`.

## Toolchain (CachyOS/Arch note)

`sudo pacman -S` has no `python312` package; ESP-IDF 5.5 breaks on system
Python 3.14. Use the uv-managed CPython 3.12 **first on PATH** for every
`install.sh` / `export.sh` / `idf.py` invocation:

```sh
export PATH="$HOME/.local/share/uv/python/cpython-3.12-linux-x86_64-gnu/bin:$PATH"
python3 --version  # must say 3.12.x, NEVER 3.14
```

IDF lives outside the repo at `~/esp-idf/v5.5` (`release/v5.5` shallow clone,
`./install.sh esp32s3`).

## Build / flash / monitor

```sh
export PATH="$HOME/.local/share/uv/python/cpython-3.12-linux-x86_64-gnu/bin:$PATH"
. ~/esp-idf/v5.5/export.sh
idf.py set-target esp32s3
idf.py build
idf.py flash -p /dev/ttyACM0
idf.py monitor -p /dev/ttyACM0  # 60 s: expect hello -> session_id + 1 Hz heartbeat
```

Provision WiFi/OTA (stored NVS `wifi`: `ssid`, `password`, `ota_url`):
join AP `xiaozhi-setup`, open `http://192.168.4.1/`, POST the form.
OTA URL is never hardcoded; fallback `http://192.168.1.106:8000/xiaozhi/ota/`,
WS fallback `ws://192.168.1.106:8000/xiaozhi/v1/`.

## Pre-commit hint (ban enforced, hooks NOT installed)

`main/` must contain no `portMAX_DELAY` and no `ESP_ERROR_CHECK`
(SPEC §5/§6; lines with `ALLOWLIST` are exempt). Run manually:

```sh
sh scripts/ci-grep-ban.sh
```

Optional local hook (not installed by default):

```sh
cat > .git/hooks/pre-commit <<'EOF'
#!/bin/sh
sh scripts/ci-grep-ban.sh
EOF
chmod +x .git/hooks/pre-commit
```
