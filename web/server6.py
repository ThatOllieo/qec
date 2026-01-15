#!/usr/bin/env python3
import asyncio, struct, binascii, json
from aiohttp import web, WSMsgType

# ---- note ----
# *most* of this was ai, its not fully functional, but needed something quick to demo

# ---------- Config ----------
TCP_HOST = "0.0.0.0"
TCP_PORT = 5000           # satellite connects here
WS_HOST  = "0.0.0.0"
WS_PORT  = 8000           # browser connects to ws://<host>:8000/ws

GS_NODE_ID = 0x11         # ground-station node id to use in replies

# ---------- Auto Telemetry (only runs while satellite is connected) ----------
AUTO_TLM_INTERVAL = 5.0  # seconds between polls
AUTO_TLM_SENSORS  = [0x0003, 0x2020, 0x2030, 0x2040, 0x2050, 0x2010]  # add/remove as needed

# Message types (1 byte)
I_BRD    = 0x00
I_CMD_RQ = 0x11
I_CMD_OK = 0x01
I_CMD_ER = 0x03
I_TLM_RQ = 0x12
I_TLM_PT = 0x02
I_TLM_ER = 0x13

# ---------- Frame helpers ----------
# [LEN(2)][TYPE(1)][CID(2)][SRC(1)][DST(1)][PAY...]
# LEN = 5 + len(payload)
def make_frame(msg_type: int, correlation_id: int, src: int, dst: int, payload: bytes) -> bytes:
    body_len = 5 + len(payload)
    return struct.pack("<HBHBB", body_len, msg_type & 0xFF,
                       correlation_id & 0xFFFF, src & 0xFF, dst & 0xFF) + payload

def parse_frame(buf: bytes):
    if len(buf) < 7:
        return None
    (body_len,) = struct.unpack_from("<H", buf, 0)
    if body_len + 2 != len(buf):
        return None
    msg_type, cid, src, dst = struct.unpack_from("<BHBB", buf, 2)
    payload = buf[7:]
    return {"type": msg_type, "cid": cid, "src": src, "dst": dst, "payload": payload}

def hexdump(b: bytes, maxn=64) -> str:
    s = binascii.hexlify(b[:maxn]).decode()
    return s if len(b) <= maxn else s + f"...(+{len(b)-maxn}B)"

# ---------- Decoders (by sensor id) ----------
# NOTE: Euler/Quat/Gravity/LinearAccel are doubles (float64) per your latest setup.
def decode_sensor_payload_by_id(sensor_id: int, payload: bytes) -> tuple[str, dict]:
    """
    Returns (human_readable_text, structured_dict)
    structured_dict becomes JSON for the UI.
    """
    try:
        if sensor_id == 0x0003:
            # CPU temp: 1 x uint8
            if len(payload) < 1:
                return ("cpu_temp: <short payload>", {"error": "short"})
            (temp,) = struct.unpack_from("<B", payload, 0)
            return (
                f"cpu_temp {temp} °C",
                {"cpu_temp": temp}
            )

        # Calibration: 4 x uint8
        elif sensor_id == 0x2010:
            if len(payload) < 4:
                return ("calib: <short payload>", {"error": "short"})
            sys_, gyro, accel, mag = struct.unpack_from("<BBBB", payload, 0)
            return (
                f"calib sys={sys_} gyro={gyro} accel={accel} mag={mag}",
                {"calibration": {"sys": sys_, "gyro": gyro, "accel": accel, "mag": mag}}
            )

        # Euler as doubles (support 0x2020 too)
        elif sensor_id in (0x2011, 0x2020):
            if len(payload) < 24:
                return (f"euler(d): <short payload len={len(payload)}>", {"error": "short"})
            h, r, p = struct.unpack_from("<ddd", payload, 0)
            return (
                f"euler(deg) [double] heading={h:.4f} roll={r:.4f} pitch={p:.4f}",
                {"euler_deg": {"heading": h, "roll": r, "pitch": p}}
            )

        # Quaternion as doubles
        elif sensor_id in (0x2012, 0x2030):
            if len(payload) < 32:
                return (f"quat(d): <short payload len={len(payload)}>", {"error": "short"})
            w, x, y, z = struct.unpack_from("<dddd", payload, 0)
            norm = (w*w + x*x + y*y + z*z) ** 0.5
            return (
                f"quat [double] w={w:.6f} x={x:.6f} y={y:.6f} z={z:.6f} | |q|={norm:.6f}",
                {"quat": {"w": w, "x": x, "y": y, "z": z, "norm": norm}}
            )

        # Gravity as doubles (m/s^2)
        elif sensor_id in (0x2013, 0x2040):
            if len(payload) < 24:
                return (f"gravity(d): <short payload len={len(payload)}>", {"error": "short"})
            gx, gy, gz = struct.unpack_from("<ddd", payload, 0)
            return (
                f"gravity m/s^2 [double] x={gx:.5f} y={gy:.5f} z={gz:.5f}",
                {"gravity_mps2": {"x": gx, "y": gy, "z": gz}}
            )

        # Linear Accel as doubles (m/s^2)
        elif sensor_id in (0x2014, 0x2050):
            if len(payload) < 24:
                return (f"linacc(d): <short payload len={len(payload)}>", {"error": "short"})
            ax, ay, az = struct.unpack_from("<ddd", payload, 0)
            return (
                f"linacc m/s^2 [double] x={ax:.5f} y={ay:.5f} z={az:.5f}",
                {"linacc_mps2": {"x": ax, "y": ay, "z": az}}
            )

        # Fallback: parse any N doubles
        else:
            n = len(payload)
            if n % 8 == 0 and n > 0:
                cnt = n // 8
                vals = struct.unpack_from("<" + "d"*cnt, payload, 0)
                return (
                    f"sensor 0x{sensor_id:04X} doubles[{cnt}]=" + ",".join(f"{v:.6g}" for v in vals),
                    {"unknown_doubles": list(vals)}
                )
            return (f"sensor 0x{sensor_id:04X} payload(len={n})={payload.hex()}",
                    {"raw_hex": payload.hex(), "len": n})

    except struct.error as e:
        return (f"decode error 0x{sensor_id:04X}: {e}", {"error": str(e)})

# ---------- Shared state ----------
sat_writer: asyncio.StreamWriter | None = None
ws_clients: set[web.WebSocketResponse] = set()
corr_id = 1

# correlation id -> sensor id (to decode replies)
pending_tlm: dict[int, int] = {}

def next_cid() -> int:
    global corr_id
    c = corr_id
    corr_id = (corr_id + 1) & 0xFFFF
    if corr_id == 0: corr_id = 1
    return c

async def ws_broadcast(line: str):
    dead = []
    for ws in list(ws_clients):
        try:
            await ws.send_str(line)
        except Exception:
            dead.append(ws)
    for ws in dead:
        ws_clients.discard(ws)

async def ws_broadcast_json(obj: dict):
    dead = []
    for ws in list(ws_clients):
        try:
            await ws.send_json(obj)
        except Exception:
            dead.append(ws)
    for ws in dead:
        ws_clients.discard(ws)

# ---------- Auto TLM sender (spawns when a satellite connects, stops on disconnect) ----------
async def auto_tlm_task(current_writer: asyncio.StreamWriter):
    """
    Periodically request telemetry for selected sensors while the given satellite connection is alive.
    Stops automatically when the connection closes or a different satellite replaces the writer.
    """
    try:
        while True:
            # If the connection is gone or replaced, stop.
            if sat_writer is None or sat_writer is not current_writer:
                break

            await asyncio.sleep(AUTO_TLM_INTERVAL)

            # Double-check connection is still valid after sleeping
            if sat_writer is None or sat_writer is not current_writer:
                break

            for sensor_id in AUTO_TLM_SENSORS:
                try:
                    cid = next_cid()
                    payload = struct.pack("<H", sensor_id)
                    frame = make_frame(I_TLM_RQ, cid, 0x01, GS_NODE_ID, payload)
                    current_writer.write(frame)
                    await current_writer.drain()
                    pending_tlm[cid] = sensor_id
                    await ws_broadcast(f"[GS] Auto-Requested TLM (cid={cid}, sensor=0x{sensor_id:04X})")
                except Exception as e:
                    await ws_broadcast(f"[GS] auto-TLM error for sensor 0x{sensor_id:04X}: {e}")
                    # If we hit an error writing, assume connection is bad and exit loop
                    break
    except asyncio.CancelledError:
        # Task cancelled on disconnect; just exit quietly
        pass

# ---------- TCP: satellite link ----------
async def tcp_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    global sat_writer
    peer = writer.get_extra_info("peername")
    sat_writer = writer
    await ws_broadcast(f"[GS] Satellite connected from {peer}")
    auto_task = asyncio.create_task(auto_tlm_task(writer))
    try:
        buf = b""
        while True:
            chunk = await reader.read(4096)
            if not chunk:
                await ws_broadcast("[GS] Satellite disconnected")
                break
            buf += chunk
            while len(buf) >= 2:
                body_len = struct.unpack_from("<H", buf, 0)[0]
                total = body_len + 2
                if len(buf) < total:
                    break
                frame, buf = buf[:total], buf[total:]
                parsed = parse_frame(frame)
                if not parsed:
                    await ws_broadcast("[GS] RX: length mismatch / parse error")
                    continue

                # --- SPECIAL: image refresh trigger (cmd 0x0013) ---
                if parsed["type"] == I_CMD_RQ and len(parsed["payload"]) >= 2:
                    (cmd_id,) = struct.unpack_from("<H", parsed["payload"], 0)
                    if cmd_id == 0x0013:
                        await ws_broadcast("IMG_REFRESH")
                        if sat_writer is not None:
                            ack = make_frame(I_CMD_OK, parsed["cid"], GS_NODE_ID, parsed["src"], b"")
                            sat_writer.write(ack)
                            try:
                                await sat_writer.drain()
                            except Exception as e:
                                await ws_broadcast(f"[GS] ACK drain error: {e}")
                        await ws_broadcast(f"[GS] Auto-ACK image ready (cid={parsed['cid']})")
                        continue
                # --- /SPECIAL ---

                # --- Telemetry Point reply: decode using cid -> sensor_id ---
                if parsed["type"] == I_TLM_PT:
                    cid = parsed["cid"]
                    sensor_id = pending_tlm.pop(cid, None)
                    if sensor_id is None:
                        await ws_broadcast(
                            f"[GS] TLM_PT (cid={cid}) unknown sensor-id; raw(len={len(parsed['payload'])}): {hexdump(parsed['payload'])}"
                        )
                        continue

                    human, data = decode_sensor_payload_by_id(sensor_id, parsed["payload"])
                    # Human log
                    await ws_broadcast(
                        f"[GS] TLM_PT cid={cid} sensor=0x{sensor_id:04X} {human} raw(len={len(parsed['payload'])}): {hexdump(parsed['payload'])}"
                    )
                    # JSON event for UI
                    await ws_broadcast_json({
                        "type": "TLM_UPDATE",
                        "cid": cid,
                        "sensor_id": sensor_id,
                        "len": len(parsed["payload"]),
                        "raw_hex": parsed["payload"].hex(),
                        "data": data
                    })
                    continue

                # Optional: Telemetry Error
                if parsed["type"] == I_TLM_ER:
                    cid = parsed["cid"]
                    sensor_id = pending_tlm.pop(cid, None)
                    await ws_broadcast(
                        f"[GS] TLM_ER cid={cid} sensor={f'0x{sensor_id:04X}' if sensor_id is not None else '<?>'} "
                        f"payload(len={len(parsed['payload'])}): {hexdump(parsed['payload'])}"
                    )
                    await ws_broadcast_json({
                        "type": "TLM_ERROR",
                        "cid": cid,
                        "sensor_id": sensor_id,
                        "len": len(parsed["payload"]),
                        "raw_hex": parsed["payload"].hex()
                    })
                    continue

                # Default: just log it
                await ws_broadcast(
                    f"[GS] RX: type=0x{parsed['type']:02X} cid={parsed['cid']} "
                    f"src=0x{parsed['src']:02X} dst=0x{parsed['dst']:02X} "
                    f"payload(len={len(parsed['payload'])}): {hexdump(parsed['payload'])} "
                    f"payloadAsInt={int.from_bytes(parsed['payload'], 'little')}"
                )
    except Exception as e:
        await ws_broadcast(f"[GS] TCP error: {e}")
    finally:
        try:
            auto_task.cancel()
            await auto_task
        except Exception:
            pass
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass
        if sat_writer is writer:
            sat_writer = None

async def start_tcp_server(app: web.Application):
    server = await asyncio.start_server(tcp_client, TCP_HOST, TCP_PORT)
    app["tcp_server"] = server
    await ws_broadcast(f"[GS] Listening SAT TCP on {TCP_HOST}:{TCP_PORT}")

async def stop_tcp_server(app: web.Application):
    server: asyncio.AbstractServer = app.get("tcp_server")
    if server:
        server.close()
        await server.wait_closed()

# ---------- WebSocket for browser UI ----------
async def ws_handler(request: web.Request):
    ws = web.WebSocketResponse(max_msg_size=1<<20)
    await ws.prepare(request)
    ws_clients.add(ws)
    await ws.send_str("[UI] Connected. Commands: tlm <id> | cmd <id> | raw <hex> | help")
    await ws.send_str('[UI] JSON events: {"type":"TLM_UPDATE", ...} / {"type":"TLM_ERROR", ...}')
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                break
            line = msg.data.strip()
            if not line:
                continue
            await handle_ui_command(ws, line)
    finally:
        ws_clients.discard(ws)
        await ws.close()
    return ws

async def handle_ui_command(ws: web.WebSocketResponse, line: str):
    from binascii import unhexlify
    if line.lower() in ("help", "?"):
        await ws.send_str("Commands:\n  tlm <sensor_id>\n  cmd <cmd_id>\n  raw <hexbytes>\n  help")
        return
    if sat_writer is None:
        await ws.send_str("[GS] No satellite connected")
        return

    parts = line.split()
    cmd = parts[0].lower()
    try:
        if cmd == "tlm" and len(parts) >= 2:
            sensor_id = int(parts[1], 0)
            cid = next_cid()
            payload = struct.pack("<H", sensor_id)
            frame = make_frame(I_TLM_RQ, cid, 0x01, GS_NODE_ID, payload)
            sat_writer.write(frame); await sat_writer.drain()
            # remember mapping for the incoming reply
            pending_tlm[cid] = sensor_id
            await ws.send_str(f"[GS] Sent TelemetryRequest (cid={cid}, sensor=0x{sensor_id:04X})")

        elif cmd == "cmd" and len(parts) >= 2:
            command_id = int(parts[1], 0)
            cid = next_cid()
            payload = struct.pack("<H", command_id)
            frame = make_frame(I_CMD_RQ, cid, 0x01, GS_NODE_ID, payload)
            sat_writer.write(frame); await sat_writer.drain()
            await ws.send_str(f"[GS] Sent Command (cid={cid}, cmd=0x{command_id:04x})")

        elif cmd == "raw" and len(parts) >= 2:
            payload = unhexlify("".join(parts[1:]))
            cid = next_cid()
            frame = make_frame(I_BRD, cid, 0x01, GS_NODE_ID, payload)
            sat_writer.write(frame); await sat_writer.drain()
            await ws.send_str(f"[GS] Sent RAW (cid={cid}, {payload.hex()})")

        else:
            await ws.send_str("Commands:\n  tlm <sensor_id)\n  cmd <cmd_id>\n  raw <hexbytes>\n  help")
    except Exception as e:
        await ws.send_str(f"[GS] TX error: {e}")

# ---------- App wiring ----------
def main():
    app = web.Application()
    app.router.add_get("/ws", ws_handler)
    app.on_startup.append(start_tcp_server)
    app.on_cleanup.append(stop_tcp_server)
    print(f"[GS] WebSocket on {WS_HOST}:{WS_PORT} | SAT TCP on {TCP_HOST}:{TCP_PORT}")
    web.run_app(app, host=WS_HOST, port=WS_PORT)

if __name__ == "__main__":
    main()
