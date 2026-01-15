const log  = document.getElementById('log');
const inp  = document.getElementById('input');
const limg = document.getElementById('leftImage');
const rimg = document.getElementById('rightImage');

// ---- hard-coded image path ----
const IMAGE_PATH_L = '../media/left.jpg';
const IMAGE_PATH_R = '../media/right.jpg';
// --------------------------------

const WS_PORT = 8000;
const WS_URL  = `ws://${location.hostname}:${WS_PORT}/ws`;

let ws;

function append(line) {
  const atBottom = (log.scrollTop + log.clientHeight + 8) >= log.scrollHeight;
  const div = document.createElement('div');
  div.textContent = line;
  log.appendChild(div);
  if (atBottom) log.scrollTop = log.scrollHeight;
}

// --- Simple event bus so you can bind UI widgets by sensor id ---
const bus = {};
function on(evt, cb){ (bus[evt] ||= []).push(cb); }
function emit(evt, data){ (bus[evt]||[]).forEach(cb => cb(data)); }

function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => append('[UI] connected');
  ws.onclose = () => { append('[UI] disconnected, retrying...'); setTimeout(connect, 1000); };
  ws.onerror = () => append('[UI] websocket error');

  ws.onmessage = ev => {
    const raw = ev.data || '';

    // Special text signal for images
    if (raw === 'IMG_REFRESH') {
      limg.src = IMAGE_PATH_L + '?t=' + Date.now();
      rimg.src = IMAGE_PATH_R + '?t=' + Date.now();
      append('[UI] Refreshed image');
      return;
    }

    // Try JSON first
    let msg = null;
    try { msg = JSON.parse(raw); } catch {}

    if (msg && typeof msg === 'object') {
      // Route telemetry JSON
      if (msg.type === 'TLM_UPDATE') {
        const sid = msg.sensor_id;
        // e.g., emit both exact and wildcard events
        emit(`sensor:${sid.toString(16).padStart(4,'0')}`, msg);
        emit('sensor:any', msg);
        // Optional: also append a compact log
        append(`[TLM] 0x${sid.toString(16).padStart(4,'0')} ${JSON.stringify(msg.data)}`);
        return;
      }
      if (msg.type === 'TLM_ERROR') {
        append(`[TLM_ERROR] cid=${msg.cid} sensor=${msg.sensor_id != null ? '0x'+msg.sensor_id.toString(16) : '<?>'} len=${msg.len}`);
        return;
      }

      // Unrecognized JSON → log it
      append('[JSON] ' + raw);
      return;
    }

    // Fallback: plain string logs from the server
    append(raw);
  };
}
connect();

function sendLine() {
  const line = inp.value.trim();
  if (!line || ws.readyState !== WebSocket.OPEN) return;
  append('> ' + line);
  ws.send(line);
  inp.value = '';
}
inp.addEventListener('keydown', e => { if (e.key === 'Enter') sendLine(); });

/* Wildcard: log any sensor update in console (optional) */
on('sensor:any', (msg) => {
  // console.debug('TLM_UPDATE', msg.sensor_id.toString(16), msg.data);
  updateBoundElements(msg);
});

// Helper: safe nested lookup
function get(obj, path) {
  return path.split('.').reduce((o, k) => (o && k in o) ? o[k] : undefined, obj);
}

// Generic updater: fills any element with matching data-sensor + data-path
function updateBoundElements(msg) {
  const sidHex = '0x' + msg.sensor_id.toString(16).padStart(4,'0');
  const nodes = document.querySelectorAll(`[data-sensor="${sidHex}"]`);
  nodes.forEach(el => {
    const path = el.getAttribute('data-path');
    const fmt  = el.getAttribute('data-fmt'); // optional decimals
    const v = get(msg.data, path);
    if (v == null) return;
    el.textContent = (typeof v === 'number' && fmt) ? v.toFixed(+fmt) : v;
  });
}
