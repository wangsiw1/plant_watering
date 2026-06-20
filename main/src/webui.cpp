#include "WebUI.h"
#include "config.h"
#include "Utility.h"
#include "Sensor.h"
#include "Pump.h"
#include <WebServer.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include "BluetoothMain.h"
// extra system headers for diagnostics
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_system.h>
#include <WiFi.h>
#include <freertos/timers.h>
#include "esp_timer.h"
#include "WateringManager.h"

static WebServer server(80);

static TaskHandle_t sLoopTask = nullptr;
static TaskHandle_t sWebTask = nullptr;
static TaskHandle_t sSensorTask = nullptr;
static TaskHandle_t sWateringTask = nullptr;

void webSetDiagnosticsTaskHandles(TaskHandle_t loopTask,
                                  TaskHandle_t webTask,
                                  TaskHandle_t sensorTask,
                                  TaskHandle_t wateringTask) {
  sLoopTask = loopTask;
  sWebTask = webTask;
  sSensorTask = sensorTask;
  sWateringTask = wateringTask;
}

// One-shot timer used to turn the pump off without allocating a task.
static TimerHandle_t sPumpOffTimer = nullptr;
static void pumpOffTimerCallback(TimerHandle_t xTimer) {
  (void)xTimer;
  pumpOff();
}

static const char *stateToString(State currentState) {
  switch (currentState) {
    case READY: return "READY";
    case SYNCING: return "SYNCING";
    case WATERING: return "WATERING";
    case SLEEPING: return "SLEEPING";
    default: return "UNKNOWN";
  }
}

void handleRoot() {
  const char* html = R"rawliteral(
  <!doctype html>
  <html>
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>Plant Watering</title>
    <style>
      body{font-family:Arial,Helvetica,sans-serif;margin:8px}
      div{margin-top:6px}
      button{padding:6px 10px;margin-top:6px; margin-left:6px}
      table{border-collapse:collapse;width:100%}
    th,td{border:1px solid #ddd;padding:6px;text-align:left}
    .field{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-top:6px}
    .field label{flex:0 0 150px}
    .field input:not([type="checkbox"]){box-sizing:border-box;width:100%;max-width:var(--field-width,240px)}
    .field.short{--field-width:120px}
    .field.medium{--field-width:240px}
    .field.long{--field-width:380px}
    .field.compact label{flex:0 0 auto}
    .field-row{display:flex;flex-wrap:wrap;gap:6px 14px;align-items:center;margin-top:6px}
    .field-row .field{margin-top:0}
    .actions{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin-top:6px}
    .actions button{margin-left:0}
    .meta{color:#666}
    /* pot rows are indented to visually separate from worker headers */
    .pot{margin-left:18px;padding-left:8px;border-left:2px solid #eee;background:#fafafa}
    @media (max-width:600px){
      .field{display:block}
      .field label{display:block;margin-bottom:3px}
      .field input:not([type="checkbox"]){max-width:none}
      .field-row{display:block}
      .field-row .field{margin-top:6px}
      .pot{margin-left:0}
    }
    </style>
  </head>
  <body>
    <h2 id="title">Plant Watering</h2>
    <h3>Status</h3>
    <div>Current time (HH:MM): <span id="time">-</span> (<span id="timezone">-</span>)</div>
    <div>Water tank level: <span id="tank">-</span></div>
    <div>System status: <span id="status">-</span></div>
    <div>Auto mode: <span id="auto">-</span></div>
    <div>Water interval (s): <span id="wint">-</span></div>
    <div>Data sync interval (s): <span id="dsint">-</span></div>
    <div>Last data sync cycle: <span id="lastDataSync">-</span></div>
    <div>Next data sync cycle: <span id="nextDataSync">-</span></div>
    <div>Last auto watering: <span id="last">-</span></div>
    <div>
      <button id="pumpBtn" onclick="togglePump()">Toggle Pump 5s</button>
    </div>

    <h3>Settings</h3>
    <div class="field long">
      <label for="name">Name</label>
      <input id="name" maxlength="255">
    </div>
    <div class="field short">
      <label for="newTime">Current time (HH:MM)</label>
      <input id="newTime" type="time">
    </div>
    <div class="field medium">
      <label for="tzOffset">Timezone offset (±HH:MM)</label>
      <input id="tzOffset" placeholder="+01:00 or -05:30">
    </div>
    <div class="field short">
      <label for="activeStart">Active start (HH:MM)</label>
      <input id="activeStart" type="time">
    </div>
    <div class="field short">
      <label for="activeEnd">Active end (HH:MM)</label>
      <input id="activeEnd" type="time">
    </div>
    <div class="actions">
      <button id="syncTimeBtn" onclick="syncTime()">Sync time (NTP)</button>
      <span id="syncStatus" style="margin-left:8px"></span>
    </div>
    <div class="field medium">
      <label for="newWint">Water interval (s)</label>
      <input id="newWint" type="number" placeholder="60-2419200(28 days)" min="60" max="2419200">
    </div>
    <div class="field medium">
      <label for="newDsint">Data sync interval (s)</label>
      <input id="newDsint" type="number" placeholder="60-2419200(28 days)" min="60" max="2419200">
    </div>
    <div class="field short">
      <label for="newAuto">Auto</label>
      <input id="newAuto" type="checkbox">
    </div>
    <div class="actions">
      <button id="applySettingsBtn" onclick="applySettings()">Apply Settings</button>
      <span id="settingsStatus" style="margin-left:8px"></span>
    </div>
    <div class="actions">
      <button id="clearWifiBtn" onclick="clearWifiCredConfirm()">Clear WiFi Credentials</button>
      <button id="clearAllBtn" onclick="clearAllSettingsConfirm()" style="margin-left:8px">Reset All Settings</button>
      <span id="wifiStatus" style="margin-left:8px"></span>
      <span id="resetStatus" style="margin-left:8px"></span>
    </div>

    <h3>Workers</h3>
    <div>
      <form id="addForm" onsubmit="return false;">
          <div class="field medium"><label for="wmac">MAC (hex)</label><input id="wmac" placeholder="AABBCCDDEEFF"></div>
          <div class="field long"><label for="wname">Name</label><input id="wname" maxlength="64" placeholder="Worker name (64 UTF-8 bytes max)" title="Up to 64 UTF-8 bytes, e.g. about 64 ASCII chars or 16 emoji."></div>
          <!-- Threshold and duration removed from quick add; defaults used -->
        <div class="actions"><button onclick="addWorker()">Add Worker</button></div>
      </form>
    </div>
    <h3>Worker list</h3>
    <div style="margin-top:8px">
      <div class="actions">
        <button id="applyAllBtn" onclick="applyAll()" disabled>Apply All</button>
      </div>
      <div id="workersList">Loading...</div>
    </div>

    <div id="diagnosticsSection" style="margin-top:18px;">
      <h3>Diagnostics</h3>
      <table id="diagTable"><tr><th>Metric</th><th>Value</th></tr></table>
    </div>

    <script>
    const dirty = new Set();
    const pending = new Set();
    const NAME_MAX_BYTES = 64;
    const utf8Encoder = new TextEncoder();
    const urlParams = new URLSearchParams(window.location.search);
    const debugMode = urlParams.get('debug') === 'true';
    let autoMode = false;
    let settingsInputsLoaded = false;

    function clampUtf8Value(value, maxBytes) {
      if (utf8Encoder.encode(value).length <= maxBytes) return value;
      let out = '';
      for (const ch of value) {
        if (utf8Encoder.encode(out + ch).length > maxBytes) break;
        out += ch;
      }
      return out;
    }

    function sanitizeNameInput(input) {
      if (!input) return '';
      const clamped = clampUtf8Value(input.value || '', NAME_MAX_BYTES);
      if (input.value !== clamped) input.value = clamped;
      return clamped;
    }

    function setWaterButtonBlocked(button, blocked) {
      if (!button) return;
      button.dataset.manualBlocked = blocked ? '1' : '0';
      button.disabled = autoMode || blocked;
    }

    function updateManualControls() {
      const pumpBtn = document.getElementById('pumpBtn');
      if (pumpBtn) pumpBtn.disabled = autoMode;
      document.querySelectorAll('.waterBtn').forEach(btn => {
        const blocked = btn.dataset.manualBlocked === '1';
        btn.disabled = autoMode || blocked;
      });
    }

    function markDirty(id) {
      dirty.add(id);
      updateButtons();
    }

    function updateButtons(){
      document.querySelectorAll('.worker').forEach(wel=>{
        const mac = wel.dataset.mac;
        const applyBtn = wel.querySelector('.applyWorkerBtn');
        const anyDirty = Array.from(dirty).some(id => id.startsWith(mac + '-'));
        if (applyBtn) applyBtn.disabled = !anyDirty || pending.size > 0;
      });
      document.getElementById('applyAllBtn').disabled = dirty.size === 0 || pending.size > 0;
    }

    // Global status timers and setter so other functions can display transient status messages.
    const statusTimers = {};
    function setStatus(id, msg, ok) {
      // Try pot/node-level status first
      const nodeEl = document.querySelector(`.node[data-id="${id}"]`);
      if (nodeEl) {
        const span = nodeEl.querySelector('.status');
        if (!span) return;
        span.innerText = msg || '';
        span.style.color = ok ? 'green' : 'crimson';
        if (statusTimers[id]) clearTimeout(statusTimers[id]);
        if (msg) statusTimers[id] = setTimeout(()=>{ span.innerText=''; delete statusTimers[id]; }, 3000);
        return;
      }
      // Fallback: worker-level status (id like MAC-all)
      if (typeof id === 'string' && id.endsWith('-all')) {
        const mac = id.slice(0, -4);
        const workerDiv = document.querySelector(`.worker[data-mac="${mac}"]`);
        if (!workerDiv) return;
        let wstatus = workerDiv.querySelector('.wstatus');
        if (!wstatus) {
          wstatus = document.createElement('span');
          wstatus.className = 'wstatus';
          wstatus.style.marginLeft = '8px';
          const headerDiv = workerDiv.querySelector('div');
          if (headerDiv) headerDiv.appendChild(wstatus);
        }
        wstatus.innerText = msg || '';
        wstatus.style.color = ok ? 'green' : 'crimson';
        if (statusTimers[id]) clearTimeout(statusTimers[id]);
        if (msg) statusTimers[id] = setTimeout(()=>{ wstatus.innerText=''; delete statusTimers[id]; }, 3000);
      }
    }

    function getNodePayloadEl(el){
      return {
        mac: el.dataset.mac,
        potIndex: parseInt(el.dataset.pi) || 0,
        name: sanitizeNameInput(el.querySelector('.iname')),
        threshold: parseInt(el.querySelector('.ith').value) || 2000,
        duration: parseInt(el.querySelector('.idur').value) || 5
      };
    }

    async function applyNode(el){
      const id = el.dataset.id;
      if (pending.has(id)) return;
      pending.add(id); updateButtons();
      const payload = getNodePayloadEl(el);
      try {
        const res = await fetch('/worker/update', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)});
        if (!res.ok) throw new Error(await res.text());
        dirty.delete(id);
        setStatus(id, 'Saved', true);
      } catch (e) {
        console.error('applyNode', id, e);
        setStatus(id, 'Save failed', false);
      } finally {
        pending.delete(id); updateButtons();
      }
    }

    async function applyAll(){
      if (pending.size) return;
      const ids = Array.from(dirty);
      if (ids.length===0) return;
      // apply sequentially to avoid flooding
      for (let i=0;i<ids.length;i++){
        if (ids[i].endsWith('-worker')) {
          const mac = ids[i].slice(0, -7);
          const wel = document.querySelector(`.worker[data-mac="${mac}"]`);
          if (wel) await applyWorker(wel);
        } else {
          const el = document.querySelector(`.node[data-id="${ids[i]}"]`);
          if (el) await applyNode(el);
        }
      }
      // refresh list after (full rebuild to capture input changes)
      setTimeout(refreshWorkersFull,300);
    }

    async function applyWorker(wel){
      const mac = wel.dataset.mac;
      const id = mac + '-worker';
      if (pending.has(id)) return;
      pending.add(id); updateButtons();
      const payload = { mac: mac, workerName: sanitizeNameInput(wel.querySelector('.wname')) };
      try {
        const res = await fetch('/worker/update', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)});
        if (!res.ok) throw new Error(await res.text());
        dirty.delete(id);
        setStatus(mac+'-all', 'Saved', true);
      } catch (e) {
        console.error('applyWorker', mac, e);
        setStatus(mac+'-all', 'Save failed', false);
      } finally {
        pending.delete(id); updateButtons();
      }
    }

    // Full rebuild of worker list including inputs and event bindings.
    async function refreshWorkersFull(){
      let r = await fetch('/nodes');
      let list = await r.json();
      let container = document.getElementById('workersList');
      container.innerHTML = '';

      const workers = list.map(w => ({
        mac: w.mac,
        workerName: w.workerName || '',
        battery: w.battery ?? null,
        batteryMv: w.batteryMv ?? null,
        lastSyncAgo: w.lastSyncAgo ?? null,
        rssi: w.rssi ?? null,
        nodePotCount: w.nodePotCount,
        pots: (w.pots || []).map(p => ({ potIndex: (typeof p.index === 'number' ? p.index : 0), name: p.name, threshold: p.threshold, duration: p.duration, soil: p.soil, soilRaw: p.soilRaw ?? null, lastWaterAgo: p.lastWaterAgo ?? null }))
      }));
      workers.forEach(w => w.pots.sort((a,b)=> (a.potIndex - b.potIndex)));

      if (workers.length === 0) {
        container.innerText = 'No workers configured';
        updateManualControls();
        updateButtons();
        return;
      }

      let idx = 0;
      for (const worker of workers) {
        idx++;
        // worker header
        const workerDiv = document.createElement('div');
        workerDiv.className = 'worker';
        workerDiv.dataset.mac = worker.mac;
        workerDiv.style.padding = '6px 0';
        workerDiv.innerHTML = `
          <hr>
          <div><strong>${idx}. ${worker.mac}</strong></div>
          <div class="field long">
            <label for="workerName-${worker.mac}">Worker name</label>
            <input id="workerName-${worker.mac}" class="wname" value="${escapeHtml(worker.workerName)}" maxlength="64" title="Up to 64 UTF-8 bytes, e.g. about 64 ASCII chars or 16 emoji.">
          </div>
          <div class="wmeta meta">${formatBattery(worker)} Last Sync: ${formatRelativeAge(worker.lastSyncAgo)} RSSI: ${worker.rssi===null?'-':(worker.rssi + 'dBm')}</div>
          <div class="actions">
            <button class="applyWorkerBtn">Apply</button>
            <button class="removeAllBtn">Remove</button>
          </div>
        `;
        container.appendChild(workerDiv);
        // bind apply for this worker (apply all dirty pots for the worker)
        const applyWorkerBtn = workerDiv.querySelector('.applyWorkerBtn');
        const workerNameInput = workerDiv.querySelector('.wname');
        workerNameInput.addEventListener('input', ()=> { sanitizeNameInput(workerNameInput); markDirty(worker.mac + '-worker'); });
        applyWorkerBtn.addEventListener('click', async ()=>{
          if (pending.size) return;
          if (dirty.has(worker.mac + '-worker')) await applyWorker(workerDiv);
          const nodes = workerDiv.querySelectorAll('.node');
          for (let i=0;i<nodes.length;i++){
            const el = nodes[i];
            if (dirty.has(el.dataset.id)) await applyNode(el);
          }
          setTimeout(refreshWorkersFull,300);
        });
        // bind remove all
        const removeAllBtn = workerDiv.querySelector('.removeAllBtn');
        removeAllBtn.addEventListener('click', async ()=> {
          setStatus(worker.mac+'-all', 'Removing...', false);
          const ok = await removeWorker(worker.mac);
          if (ok) { setStatus(worker.mac+'-all', 'Removed', true); setTimeout(refreshWorkersFull,300); } else setStatus(worker.mac+'-all', 'Remove failed', false);
        });

        // render pots (ordered by potIndex)
        if (!worker.pots || worker.pots.length === 0) {
          const p = document.createElement('div');
          p.innerText = 'No pots configured/synced yet';
          workerDiv.appendChild(p);
        } else {
          for (let pi=0; pi<worker.pots.length; ++pi) {
            const pot = worker.pots[pi];
            const div = document.createElement('div');
            div.className = 'node pot';
            div.dataset.mac = worker.mac;
            div.dataset.pi = (pot.potIndex===undefined?0:pot.potIndex);
            const id = worker.mac + '-' + div.dataset.pi;
            div.dataset.id = id;
            div.style.padding = '6px 6px';
            div.innerHTML = `
              <div><strong>Pot ${pot.potIndex+1}</strong></div>
              <div class="field-row">
                <div class="field long compact">
                  <label for="potName-${id}">Name</label>
                  <input id="potName-${id}" class="iname" value="${escapeHtml(pot.name||'')}" maxlength="64" title="Up to 64 UTF-8 bytes, e.g. about 64 ASCII chars or 16 emoji.">
                </div>
                <div class="field short compact">
                  <label for="potThreshold-${id}">Threshold</label>
                  <input id="potThreshold-${id}" class="ith" type="number" value="${pot.threshold}" min="0" max="4095">
                </div>
                <div class="field short compact">
                  <label for="potDuration-${id}">Duration(s)</label>
                  <input id="potDuration-${id}" class="idur" type="number" value="${pot.duration}" min="1" max="60">
                </div>
              </div>
                <div class="meta">
                  <span class="soil">${formatSoil(pot)}</span>
                  <span class="lastwater" style="margin-left:8px">Last Water: ${formatRelativeAge(pot.lastWaterAgo)}</span>
                </div>
                <div class="actions">
                  <button class="waterBtn" disabled>Water</button>
                  <span class="status" style="margin-left:8px"></span>
                </div>
            `;
            const iname = div.querySelector('.iname');
            const ith = div.querySelector('.ith');
            const idur = div.querySelector('.idur');
            iname.addEventListener('input', ()=> { sanitizeNameInput(iname); markDirty(id); });
            ith.addEventListener('input', ()=> markDirty(id));
            idur.addEventListener('input', ()=> markDirty(id));
            const waterBtn = div.querySelector('.waterBtn');
            // display sensor status and disable water button if sensor not connected or worker not synced
            const soilSpan = div.querySelector('.soil');
            const lastSpan = div.querySelector('.lastwater');
            const synced = worker.lastSyncAgo != null;
            if (isSoilDisconnected(pot)) {
              soilSpan.innerText = formatSoil(pot);
              setWaterButtonBlocked(waterBtn, true);
            } else {
              soilSpan.innerText = formatSoil(pot);
              setWaterButtonBlocked(waterBtn, !synced);
            }
            if (lastSpan) lastSpan.innerText = 'Last Water: ' + formatRelativeAge(pot.lastWaterAgo);
            waterBtn.addEventListener('click', async ()=> { setStatus(id, 'Sending...', false); const dur = parseInt(idur.value) || pot.duration; const ok = await waterWorker(div.dataset.mac, parseInt(div.dataset.pi), dur); if (ok) setStatus(id, 'Water sent', true); else setStatus(id, 'Send failed', false); });
            workerDiv.appendChild(div);
          }
        }
      }
      updateManualControls();
      updateButtons();
    }

    // Partial refresh: only update non-input information (battery, last sync, rssi, soil, last water)
    async function refreshWorkersPartial(){
      try {
        let r = await fetch('/nodes');
        let list = await r.json();
        // Group by MAC like full
        const workers = list.map(w => ({
          mac: w.mac,
          battery: w.battery ?? null,
          batteryMv: w.batteryMv ?? null,
          lastSyncAgo: w.lastSyncAgo ?? null,
          rssi: w.rssi ?? null,
          pots: (w.pots || []).map(p => ({ potIndex: (typeof p.index === 'number' ? p.index : 0), soil: p.soil, soilRaw: p.soilRaw ?? null, lastWaterAgo: p.lastWaterAgo ?? null }))
        }));
        workers.forEach(w => w.pots.sort((a,b)=> (a.potIndex - b.potIndex)));

        for (const worker of workers) {
          const workerDiv = document.querySelector(`.worker[data-mac="${worker.mac}"]`);
          if (!workerDiv) continue; // skip workers not present in DOM (added/removed -> full refresh will handle)
          // update header meta
          const winfo = workerDiv.querySelector('.wmeta');
          if (winfo) {
            winfo.innerText = `${formatBattery(worker)} Last Sync: ${formatRelativeAge(worker.lastSyncAgo)} RSSI: ${worker.rssi===null?'-':(worker.rssi + 'dBm')}`;
          }
          // update pots
          for (let pi=0; pi<worker.pots.length; ++pi) {
            const pot = worker.pots[pi];
            const node = workerDiv.querySelector(`.node[data-pi="${pot.potIndex}"]`);
            if (!node) continue;
            const soilSpan = node.querySelector('.soil');
            const lastSpan = node.querySelector('.lastwater');
            const waterBtn = node.querySelector('.waterBtn');
            const synced = worker.lastSyncAgo != null;
            if (soilSpan) {
              soilSpan.innerText = formatSoil(pot);
            }
            if (lastSpan) lastSpan.innerText = 'Last Water: ' + formatRelativeAge(pot.lastWaterAgo);
            setWaterButtonBlocked(waterBtn, (isSoilDisconnected(pot) || !synced));
          }
        }
        updateManualControls();
        updateButtons();
      } catch (e) { console.error('refreshWorkersPartial', e); }
    }

    function escapeHtml(s){ return (s||'').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }

    function finiteNumber(value) {
      return typeof value === 'number' && Number.isFinite(value);
    }

    function formatBattery(worker) {
      if (!worker || !finiteNumber(worker.battery)) return 'Battery: -';
      let text = 'Battery: ' + worker.battery + '%';
      if (debugMode && finiteNumber(worker.batteryMv)) text += ' (' + worker.batteryMv + ' mV)';
      return text;
    }

    function isSoilDisconnected(pot) {
      return pot && finiteNumber(pot.soilRaw) && pot.soilRaw < 50;
    }

    function formatSoil(pot) {
      if (isSoilDisconnected(pot)) {
        let text = 'Soil Moisture: Sensor not connected';
        if (debugMode && finiteNumber(pot.soilRaw)) text += ' (raw ' + pot.soilRaw + ')';
        return text;
      }
      let text = 'Soil Moisture: ' + (pot && finiteNumber(pot.soil) ? pot.soil : '-');
      if (debugMode && pot && finiteNumber(pot.soilRaw)) text += ' (raw ' + pot.soilRaw + ')';
      return text;
    }

    function formatRelativeAge(ageSeconds){
      if (typeof ageSeconds !== 'number' || !Number.isFinite(ageSeconds) || ageSeconds < 0) return '-';
      const seconds = Math.floor(ageSeconds);
      let value;
      let unit;
      if (seconds < 60) {
        value = seconds;
        unit = 'second';
      } else if (seconds < 3600) {
        value = Math.floor(seconds / 60);
        unit = 'minute';
      } else if (seconds < 86400) {
        value = Math.floor(seconds / 3600);
        unit = 'hour';
      } else {
        value = Math.floor(seconds / 86400);
        unit = 'day';
      }
      return value + ' ' + unit + (value === 1 ? '' : 's') + ' ago';
    }

    function formatRelativeFuture(secondsUntil){
      if (typeof secondsUntil !== 'number' || !Number.isFinite(secondsUntil) || secondsUntil < 0) return '-';
      const seconds = Math.floor(secondsUntil);
      let value;
      let unit;
      if (seconds < 60) {
        value = seconds;
        unit = 'second';
      } else if (seconds < 3600) {
        value = Math.floor(seconds / 60);
        unit = 'minute';
      } else if (seconds < 86400) {
        value = Math.floor(seconds / 3600);
        unit = 'hour';
      } else {
        value = Math.floor(seconds / 86400);
        unit = 'day';
      }
      return 'in ' + value + ' ' + unit + (value === 1 ? '' : 's');
    }

    async function addWorker(){
      const mac = document.getElementById('wmac').value.trim();
      const name = sanitizeNameInput(document.getElementById('wname'));
      const body = {mac:mac, workerName:name};
      await fetch('/worker/add', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
      setTimeout(refreshWorkersFull,300);
    }

    async function removeWorker(mac, potIndex){
      try {
        const body = {mac:mac};
        const res = await fetch('/worker/remove',{method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
        const ok = res.ok;
        if (ok) setTimeout(refreshWorkersFull,300);
        return ok;
      } catch (e) { return false; }
    }

    async function waterWorker(mac, potIndex, duration){
      try {
        const body = {mac:mac, duration:duration}; if (typeof potIndex !== 'undefined') body.potIndex = potIndex;
        const res = await fetch('/worker/water',{method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
        return res.ok;
      } catch (e) { return false; }
    }

    async function togglePump(){
      await fetch('/pump/toggle', {method:'POST'});
    }

    async function applySettings(){
      const btn = document.getElementById('applySettingsBtn');
      const status = document.getElementById('settingsStatus');
      const timeVal = (document.getElementById('newTime').value || '').trim();
      const tzVal = (document.getElementById('tzOffset').value || '').trim();
      const body = {
        name:document.getElementById('name').value,
        auto:document.getElementById('newAuto').checked,
        wint:parseInt(document.getElementById('newWint').value),
        dsint:parseInt(document.getElementById('newDsint').value)
      };
      if (timeVal) body.newTime = timeVal;
      if (tzVal) body.tzOffset = tzVal;
      // active window inputs (HH:MM -> seconds since midnight)
      const aStart = (document.getElementById('activeStart') || {}).value || '';
      const aEnd = (document.getElementById('activeEnd') || {}).value || '';
      if (aStart) {
        const parts = aStart.split(':'); if (parts.length === 2) body.activeStart = parseInt(parts[0])*3600 + parseInt(parts[1])*60;
      }
      if (aEnd) {
        const parts = aEnd.split(':'); if (parts.length === 2) body.activeEnd = parseInt(parts[0])*3600 + parseInt(parts[1])*60;
      }
      try {
        btn.disabled = true;
        status.innerText = 'Saving...'; status.style.color = 'gray';
        const res = await fetch('/settings', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
        if (!res.ok) throw new Error(await res.text());
        status.innerText = 'Saved'; status.style.color = 'green';
        settingsInputsLoaded = false;
        setTimeout(fetchStatus,300);
      } catch (e) {
        status.innerText = 'Save failed'; status.style.color = 'crimson';
      } finally {
        btn.disabled = false;
        setTimeout(()=>{ status.innerText = ''; }, 3000);
      }
    }

    async function syncTime(){
      const btn = document.getElementById('syncTimeBtn');
      const status = document.getElementById('syncStatus');
      if (!btn || !status) return;
      btn.disabled = true; status.innerText = 'Syncing...'; status.style.color = 'gray';
      try {
        const res = await fetch('/time/sync', {method:'POST'});
        const j = await res.json();
        if (!res.ok || !j.ok) throw new Error(JSON.stringify(j));
        status.innerText = 'Synced'; status.style.color = 'green';
        setTimeout(fetchStatus, 300);
      } catch (e) {
        status.innerText = 'Sync failed'; status.style.color = 'crimson';
      } finally {
        btn.disabled = false; setTimeout(()=>{ status.innerText=''; }, 3000);
      }
    }

    async function clearWifiCredConfirm() {
      const confirmed = confirm("Are you sure you want to clear saved WiFi credentials? This action cannot be undone.");
      if (!confirmed) return;
      
      const btn = document.getElementById('clearWifiBtn');
      const status = document.getElementById('wifiStatus');
      btn.disabled = true;
      status.innerText = 'Clearing...'; status.style.color = 'gray';
      
      try {
        const res = await fetch('/wifi/clear', {method:'POST'});
        if (!res.ok) throw new Error(await res.text());
        status.innerText = 'Cleared'; status.style.color = 'green';
        setTimeout(fetchStatus, 300);
      } catch (e) {
        status.innerText = 'Clear failed'; status.style.color = 'crimson';
      } finally {
        btn.disabled = false;
        setTimeout(()=>{ status.innerText = ''; }, 3000);
      }
    }
    
    async function clearAllSettingsConfirm() {
      const confirmed = confirm("Are you sure you want to reset all settings and remove all workers? This action cannot be undone.");
      if (!confirmed) return;
      const btn = document.getElementById('clearAllBtn');
      const status = document.getElementById('resetStatus');
      btn.disabled = true;
      status.innerText = 'Resetting...'; status.style.color = 'gray';
      try {
        const res = await fetch('/settings/clear_all', {method:'POST'});
        if (!res.ok) throw new Error(await res.text());
        status.innerText = 'Reset'; status.style.color = 'green';
        setTimeout(fetchStatus, 300);
        setTimeout(refreshWorkersFull, 300);
      } catch (e) {
        status.innerText = 'Reset failed'; status.style.color = 'crimson';
      } finally {
        btn.disabled = false; setTimeout(()=>{ status.innerText=''; }, 3000);
      }
    }
      
    // render diagnostics table (located at page bottom)
    function renderDiagnostics(j){
      const tbl = document.getElementById('diagTable');
      if (!tbl) return;
      function fmt(n){ if (n===null || n===undefined) return '-'; return n; }
      function fmtKb(n){ if (n===null || n===undefined) return '-'; return parseFloat((n/1024).toFixed(2)); }
      tbl.innerHTML = `
        <tr><th>Metric</th><th>Value</th></tr>
        <tr><td>Free Heap (KiB)</td><td>${fmtKb(j.freeHeap)}</td></tr>
        <tr><td>Minimum Free Heap (KiB)</td><td>${fmtKb(j.minFreeHeap)}</td></tr>
        <tr><td>Largest Free Block (KiB)</td><td>${fmtKb(j.largestFreeBlock)}</td></tr>
        <tr><td>Total Heap (KiB)</td><td>${fmtKb(j.heapSize)}</td></tr>
        <tr><td>Loop Stack Minimum Free (bytes)</td><td>${fmt(j.loopStackMinFree)}</td></tr>
        <tr><td>Web Stack Minimum Free (bytes)</td><td>${fmt(j.webStackMinFree)}</td></tr>
        <tr><td>Sensor Stack Minimum Free (bytes)</td><td>${fmt(j.sensorStackMinFree)}</td></tr>
        <tr><td>Watering Stack Minimum Free (bytes)</td><td>${fmt(j.wateringStackMinFree)}</td></tr>
        <tr><td>Bluetooth TX Stack Minimum Free (bytes)</td><td>${fmt(j.btSenderStackMinFree)}</td></tr>
        <tr><td>Free Sketch Space (KiB)</td><td>${fmtKb(j.freeSketchSpace)}</td></tr>
        <tr><td>Sketch Size (KiB)</td><td>${fmtKb(j.sketchSize)}</td></tr>
        <tr><td>Flash Chip Size (KiB)</td><td>${fmtKb(j.flashChipSize)}</td></tr>
        <tr><td>Chip Revision</td><td>${fmt(j.chipRevision)}</td></tr>
        <tr><td>NVS used entries</td><td>${fmt(j.nvs_entries)}</td></tr>
        <tr><td>NVS total entries</td><td>${fmt(j.nvs_total_entries)}</td></tr>
        <tr><td>WiFi RSSI (dBm)</td><td>${fmt(j.rssi)}</td></tr>
      `;
    }

    async function fetchDiagnostics(){
      if (!debugMode) return;
      try {
        const r = await fetch('/diagnostics');
        const j = await r.json();
        renderDiagnostics(j);
      } catch (e) {
        console.error('fetchDiagnostics', e);
      }
    }

    async function fetchStatus(){
      try {
        let r = await fetch('/status');
        let j = await r.json();
        const tankEl = document.getElementById('tank');
        if (tankEl) {
          let ttext = '-';
          if (typeof j.tank === 'number') {
            const v = j.tank;
            if (v < 10) ttext = 'Low';
            else if (v > 2800) ttext = 'High';
            else ttext = 'Not Connected';
          }
          tankEl.innerText = ttext;
        }

        if (j.clockValid && typeof j.timeOfDaySec === 'number') {
          const hh = Math.floor(j.timeOfDaySec/3600)%24;
          const mm = Math.floor((j.timeOfDaySec%3600)/60);
          const s = String(hh).padStart(2,'0')+':' + String(mm).padStart(2,'0');
          const timeSpan = document.getElementById('time');
          if (timeSpan) timeSpan.innerText = s;
        } else {
          const timeSpan = document.getElementById('time');
          if (timeSpan) timeSpan.innerText = 'Invalid';
        }

        const autoSpan = document.getElementById('auto');
        autoMode = !!j.auto;
        if (autoSpan) autoSpan.innerText = autoMode ? "On" : "Off";

        const wintSpan = document.getElementById('wint');
        if (wintSpan) wintSpan.innerText = j.waterInterval || 3600;
        const dsintSpan = document.getElementById('dsint');
        if (dsintSpan) dsintSpan.innerText = j.dataSyncInterval || 3600;
        const lastDataSyncSpan = document.getElementById('lastDataSync');
        if (lastDataSyncSpan) lastDataSyncSpan.innerText = formatRelativeAge(j.lastDataSyncAgo);
        const nextDataSyncSpan = document.getElementById('nextDataSync');
        if (nextDataSyncSpan) nextDataSyncSpan.innerText = formatRelativeFuture(j.nextDataSyncIn);
        const statusSpan = document.getElementById('status');
        if (statusSpan) statusSpan.innerText = j.state || '-';
        const lastSpan = document.getElementById('last');
        if (lastSpan) lastSpan.innerText = formatRelativeAge(j.lastAutoWateringAgo);

        const title = document.getElementById('title');
        if (title) title.innerText = 'Plant Watering ' + (j.name?('(' + j.name + ')'):"" );

        // populate timezone display every refresh, but editable settings inputs only on page load
        try {
          if (typeof j.tzOffsetMinutes === 'number') {
            const tzSpan = document.getElementById('timezone');
            if (tzSpan) {
              const mins = j.tzOffsetMinutes;
              const sign = mins < 0 ? '-' : '+';
              const am = Math.abs(mins);
              const hh = Math.floor(am/60).toString().padStart(2,'0');
              const mm = (am%60).toString().padStart(2,'0');
              tzSpan.innerText = sign + hh + ':' + mm;
            }
          }
          if (!settingsInputsLoaded) populateSettingsInputs(j);
        } catch(e) {}

        updateManualControls();
      } catch (e) {
        console.error('fetchStatus', e);
      }
    }

    function formatTimeInput(secondsOfDay){
      const s = secondsOfDay || 0;
      const hh = Math.floor(s/3600)%24;
      const mm = Math.floor((s%3600)/60);
      return String(hh).padStart(2,'0')+':' + String(mm).padStart(2,'0');
    }

    function populateSettingsInputs(j){
      const nameInput = document.getElementById('name');
      if (nameInput) nameInput.value = j.name || '';
      const autoInput = document.getElementById('newAuto');
      if (autoInput) autoInput.checked = !!j.auto;
      if (typeof j.activeStart === 'number') {
        const el = document.getElementById('activeStart');
        if (el) el.value = formatTimeInput(j.activeStart);
      }
      if (typeof j.activeEnd === 'number') {
        const el = document.getElementById('activeEnd');
        if (el) el.value = formatTimeInput(j.activeEnd);
      }
      if (typeof j.waterInterval === 'number') {
        const el = document.getElementById('newWint');
        if (el) el.value = j.waterInterval;
      }
      if (typeof j.dataSyncInterval === 'number') {
        const el = document.getElementById('newDsint');
        if (el) el.value = j.dataSyncInterval;
      }
      settingsInputsLoaded = true;
    }

    // Poll less frequently to reduce CPU/network and avoid clobbering user inputs
    const diagnosticsSection = document.getElementById('diagnosticsSection');
    if (diagnosticsSection) diagnosticsSection.style.display = debugMode ? '' : 'none';
    const addWorkerNameInput = document.getElementById('wname');
    if (addWorkerNameInput) addWorkerNameInput.addEventListener('input', ()=> sanitizeNameInput(addWorkerNameInput));
    setInterval(fetchStatus,3000);
    setInterval(refreshWorkersPartial,5000);
    fetchStatus();
    if (debugMode) {
      setInterval(fetchDiagnostics,10000);
      fetchDiagnostics();
    }
    // initial full worker list render
    refreshWorkersFull();
    </script>
  </body>
  </html>
  )rawliteral";
  
  // strlen on a string literal: computed once, cached.
  // Even better: declare html as const char html[] = R"(...)" 
  // and use sizeof(html)-1 for a compile-time constant.
  static const size_t htmlLen = strlen(html);

  // Send headers only — setContentLength ensures the correct
  // Content-Length header is written despite the empty body string.
  server.setContentLength(htmlLen);
  server.send(200, "text/html", ""); // headers sent, zero body bytes

  // Write body directly to TCP socket in MTU-sized chunks.
  // WiFiClient::write(const uint8_t*, size_t) has no String conversion.
  WiFiClient client = server.client();
  const uint8_t* data = (const uint8_t*)html;
  size_t remaining = htmlLen;
  while (remaining > 0) {
      size_t chunk = min(remaining, (size_t)1460); // ~1 TCP segment (MTU - headers)
      size_t written = client.write(data, chunk);
      if (written == 0) break; // client disconnected mid-transfer
      data += written;
      remaining -= written;
  }
}

void handleStatus() {
  int tank = getTankLevel();
  unsigned long now = millis()/1000;
  uint32_t tod = getCurrentTimeOfDaySec();
  Settings settings{};
  RuntimeSnapshot runtime{};
  getSettingsSnapshot(settings);
  getRuntimeSnapshot(runtime);
  JsonDocument doc;
  doc["name"] = settings.name;
  doc["auto"] = runtime.autoEnabled;
  doc["tank"] = tank;
  doc["now"] = now;
  doc["timeOfDaySec"] = tod;
  doc["waterInterval"] = settings.waterInterval;
  doc["dataSyncInterval"] = settings.dataSyncInterval;
  doc["tzOffsetMinutes"] = settings.tzOffsetMinutes;
  doc["activeStart"] = settings.activeStart;
  doc["activeEnd"] = settings.activeEnd;
  doc["state"] = stateToString(runtime.state);
  doc["clockValid"] = isClockValid();
  int64_t nowUs = esp_timer_get_time();
  if (runtime.autoEnabled && runtime.lastDataSyncUs != 0 &&
      nowUs >= runtime.lastDataSyncUs) {
    doc["lastDataSyncAgo"] =
        static_cast<uint64_t>((nowUs - runtime.lastDataSyncUs) / 1000000LL);
  } else {
    doc["lastDataSyncAgo"] = nullptr;
  }
  if (runtime.autoEnabled && runtime.nextDataSyncUs != 0 &&
      runtime.nextDataSyncUs >= nowUs) {
    doc["nextDataSyncIn"] =
        static_cast<uint64_t>((runtime.nextDataSyncUs - nowUs) / 1000000LL);
  } else {
    doc["nextDataSyncIn"] = nullptr;
  }
  uint64_t curEpoch = getCurrentEpochSec();
  if (curEpoch) doc["epoch"] = curEpoch;
  if (settings.lastWateringUtcSec != 0 &&
      curEpoch >= settings.lastWateringUtcSec) {
    doc["lastAutoWateringAgo"] = curEpoch - settings.lastWateringUtcSec;
  } else {
    doc["lastAutoWateringAgo"] = nullptr;
  }

  server.setContentLength(measureJson(doc));
  server.send(200, "application/json", "");
  WiFiClient client = server.client();
  serializeJson(doc, client); // Streams directly to the network buffer
}

void handleDiagnostics() {
  JsonDocument doc;
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  uint32_t largestFreeBlock = ESP.getMaxAllocHeap();
  uint32_t heapSize = ESP.getHeapSize();
  uint32_t freeSketch = ESP.getFreeSketchSpace();
  uint32_t sketchSize = ESP.getSketchSize();
  uint32_t flashChipSize = ESP.getFlashChipSize();
  uint32_t chipRev = ESP.getChipRevision();
  doc["freeHeap"] = freeHeap;
  doc["minFreeHeap"] = minFreeHeap;
  doc["largestFreeBlock"] = largestFreeBlock;
  doc["heapSize"] = heapSize;
  doc["freeSketchSpace"] = freeSketch;
  doc["sketchSize"] = sketchSize;
  doc["flashChipSize"] = flashChipSize;
  doc["chipRevision"] = chipRev;

  auto addStackHighWaterMark = [&doc](const char* name, TaskHandle_t task) {
    if (task) doc[name] = uxTaskGetStackHighWaterMark(task);
    else doc[name] = nullptr;
  };
  addStackHighWaterMark("loopStackMinFree", sLoopTask);
  addStackHighWaterMark("webStackMinFree", sWebTask);
  addStackHighWaterMark("sensorStackMinFree", sSensorTask);
  addStackHighWaterMark("wateringStackMinFree", sWateringTask);
  UBaseType_t btSenderStackMinFree = 0;
  if (BT_TLV::btCommonGetSenderStackHighWaterMark(btSenderStackMinFree)) {
    doc["btSenderStackMinFree"] = btSenderStackMinFree;
  } else {
    doc["btSenderStackMinFree"] = nullptr;
  }
  // NVS stats (may fail if NVS not initialized)
  nvs_stats_t nvs_stats; // from nvs_flash.h
  if (nvs_get_stats(NULL, &nvs_stats) == ESP_OK) {
    doc["nvs_entries"] = (uint32_t)nvs_stats.used_entries;
    doc["nvs_total_entries"] = (uint32_t)nvs_stats.total_entries;
  } else {
    doc["nvs_entries"] = nullptr;
    doc["nvs_total_entries"] = nullptr;
  }
  // WiFi RSSI if connected
  if (WiFi.status() == WL_CONNECTED) doc["rssi"] = WiFi.RSSI();
  else doc["rssi"] = nullptr;

  server.setContentLength(measureJson(doc));
  server.send(200, "application/json", "");
  WiFiClient client = server.client();
  serializeJson(doc, client); // Streams directly to the network buffer
}

void handleTimeSync() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  bool ok = trySyncNTP(10000);
  JsonDocument doc;
  doc["ok"] = ok;
  if (ok) {
    uint64_t epoch = getCurrentEpochSec();
    Settings settings{};
    getSettingsSnapshot(settings);
    doc["epoch"] = epoch;
    doc["tzOffsetMinutes"] = settings.tzOffsetMinutes;
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  } else {
    doc["error"] = "NTP sync failed";
    String out; serializeJson(doc, out);
    server.send(500, "application/json", out);
  }
}

void handlePumpToggle() {
  if (getAutoEnabled()) {
    server.send(409, "text/plain", "AUTO_MODE");
    return;
  }
  pumpOn();
  // reset/start one-shot timer to turn pump off after delay
  if (sPumpOffTimer) {
    xTimerReset(sPumpOffTimer, 0);
  } else {
    // fallback: create and start timer if not yet created
    sPumpOffTimer = xTimerCreate("pumpOff", pdMS_TO_TICKS(5000), pdFALSE, NULL, pumpOffTimerCallback);
    if (sPumpOffTimer) xTimerStart(sPumpOffTimer, 0);
  }
  server.send(200, "text/plain", "OK");
}

void handleSettings() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  if (body.isEmpty()) { server.send(400, "text/plain", "EMPTY"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) {
    server.send(400, "text/plain", "BAD_JSON");
    return;
  }

  Settings next{};
  getSettingsSnapshot(next);
  bool oldAuto = getAutoEnabled();
  bool newAuto = oldAuto;
  bool hasManualTime = false;
  uint32_t manualTimeSec = 0;
  if (doc["name"].is<const char*>())
    copyUtf8Truncated(doc["name"].as<const char*>(), next.name, sizeof(next.name));
  if (doc["auto"].is<bool>()) newAuto = doc["auto"].as<bool>();
  if (doc["wint"].is<uint32_t>()) next.waterInterval = doc["wint"].as<uint32_t>();
  if (doc["dsint"].is<uint32_t>()) next.dataSyncInterval = doc["dsint"].as<uint32_t>();
  if (doc["activeStart"].is<uint32_t>()) next.activeStart = doc["activeStart"].as<uint32_t>();
  if (doc["activeEnd"].is<uint32_t>()) next.activeEnd = doc["activeEnd"].as<uint32_t>();
  if (doc["newTime"].is<const char*>()) {
    const char* value = doc["newTime"];
    int hours = 0;
    int minutes = 0;
    char trailing = '\0';
    if (!value || sscanf(value, "%d:%d%c", &hours, &minutes, &trailing) != 2 ||
        hours < 0 || hours > 23 || minutes < 0 || minutes > 59 ||
        (getCurrentEpochSec() == 0 && next.savedUtcSec == 0)) {
      server.send(400, "text/plain", "BAD_TIME");
      return;
    }
    hasManualTime = true;
    manualTimeSec = hours * 3600u + minutes * 60u;
  }

  if (doc["tzOffsetMinutes"].is<int>()) {
    int value = doc["tzOffsetMinutes"].as<int>();
    if (value < -840 || value > 840) {
      server.send(400, "text/plain", "BAD_TIMEZONE");
      return;
    }
    next.tzOffsetMinutes = value;
  } else if (doc["tzOffset"].is<const char*>()) {
    const char* value = doc["tzOffset"];
    int hours = 0;
    int minutes = 0;
    char trailing = '\0';
    if (!value || (value[0] != '+' && value[0] != '-') ||
        sscanf(value + 1, "%d:%d%c", &hours, &minutes, &trailing) != 2 ||
        hours > 14 || minutes > 59 || (hours == 14 && minutes != 0)) {
      server.send(400, "text/plain", "BAD_TIMEZONE");
      return;
    }
    int sign = value[0] == '-' ? -1 : 1;
    next.tzOffsetMinutes = sign * (hours * 60 + minutes);
  }

  if (!applySettingsSnapshot(next)) {
    server.send(400, "text/plain", "BAD_SETTINGS");
    return;
  }
  setAutoEnabled(newAuto);
  if (newAuto && !oldAuto) {
    if (sPumpOffTimer) xTimerStop(sPumpOffTimer, 0);
    pumpOff();
  }

  if (hasManualTime && !setUserTimeOfDaySec(manualTimeSec)) {
      server.send(400, "text/plain", "BAD_TIME");
      return;
  }
  server.send(200, "text/plain", "OK");
}

void handleClearWifiCred() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  bool ok = clearWifiCredentials();
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleClearAllSettings() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  bool ok = clearAllSettings();
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleNodes() {
  // Return configured worker devices with latest discovered readings when available
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  int64_t nowUs = esp_timer_get_time();
  int workerCount = getWorkerConfigCount();
  for (int i = 0; i < workerCount; ++i) {
    WorkerConfig wc{};
    if (!getWorkerConfigAt(i, wc)) continue;
    char macs[32];
    sprintf(macs, "%02X%02X%02X%02X%02X%02X", wc.mac[0],wc.mac[1],wc.mac[2],wc.mac[3],wc.mac[4],wc.mac[5]);
    JsonObject o = arr.add<JsonObject>();
    o["mac"] = macs;
    o["workerName"] = wc.workerName;
    o["potCount"] = wc.potCount;
    JsonArray pots = o["pots"].to<JsonArray>();
    WorkerNode node{};
    bool hasNode = btMainGetNodeByMac(wc.mac, node);
    if (hasNode) {
      o["battery"] = calculateBatteryPercent(node.batteryMv);
      o["batteryMv"] = node.batteryMv;
      if (node.lastRssi != 0x7FFF) o["rssi"] = (int)node.lastRssi;
      else o["rssi"] = nullptr;
      if (node.lastSyncUs != 0 && nowUs >= node.lastSyncUs) {
        o["lastSyncAgo"] =
            static_cast<uint64_t>((nowUs - node.lastSyncUs) / 1000000LL);
      } else {
        o["lastSyncAgo"] = nullptr;
      }
      o["nodePotCount"] = node.potCount;
    } else {
      o["battery"] = nullptr;
      o["batteryMv"] = nullptr;
      o["rssi"] = nullptr;
      o["lastSyncAgo"] = nullptr;
      o["nodePotCount"] = nullptr;
    }
    int potCount = wc.potCount;
    if (potCount > MAX_POTS_PER_DEVICE) potCount = MAX_POTS_PER_DEVICE;
    for (int p = 0; p < potCount; ++p) {
      JsonObject po = pots.add<JsonObject>();
      po["index"] = p;
      po["name"] = wc.potName[p];
      po["threshold"] = wc.thresholds[p];
      po["duration"] = wc.durations[p];
      if (hasNode && p < node.potCount) {
        po["soil"] =
            (int)getCorrectedSoilMoisture(node.batteryMv, node.soils[p]);
        po["soilRaw"] = (int)node.soils[p];
      } else {
        po["soil"] = nullptr;
        po["soilRaw"] = nullptr;
      }
      int64_t lastWaterUs = hasNode ? node.lastWaterUs[p] : 0;
      if (lastWaterUs != 0 && nowUs >= lastWaterUs) {
        po["lastWaterAgo"] =
            static_cast<uint64_t>((nowUs - lastWaterUs) / 1000000LL);
      } else {
        po["lastWaterAgo"] = nullptr;
      }
    }
  }
  server.setContentLength(measureJson(doc));
  server.send(200, "application/json", "");
  WiFiClient client = server.client();
  serializeJson(doc, client); // Streams directly to the network buffer
}

void handleWorkerAdd() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac=""; uint16_t th=2000; uint16_t dur=5; String name="";
  JsonDocument doc;
  if (body.isEmpty() || deserializeJson(doc, body)) {
    server.send(400, "text/plain", "BAD_JSON");
    return;
  }
  mac = String((const char*)(doc["mac"] | ""));
  uint32_t thresholdValue = doc["threshold"] | 2000;
  uint32_t durationValue = doc["duration"] | 5;
  if (thresholdValue > 4095 || durationValue == 0 || durationValue > 60) {
    server.send(400, "text/plain", "BAD_VALUES");
    return;
  }
  th = thresholdValue;
  dur = durationValue;
  if (doc["workerName"].is<const char*>()) name = String((const char*)doc["workerName"]);
  else if (doc["name"].is<const char*>()) name = String((const char*)doc["name"]);
  bool ok = addWorkerByHex(mac.c_str(), th, dur, name.length()?name.c_str():nullptr);
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerRemove() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac="";
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (!err) mac = String((const char*)(doc["mac"] | ""));
  }
  bool ok = removeWorkerByHex(mac.c_str());
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerUpdate() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac=""; uint16_t th=2000; uint16_t dur=5; String name="";
  int potIndex = -1;
  bool hasNameField = false;
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (err) {
      server.send(400, "text/plain", "BAD_JSON");
      return;
    }
    mac = String((const char*)(doc["mac"] | ""));
    uint32_t thresholdValue = doc["threshold"] | 2000;
    uint32_t durationValue = doc["duration"] | 5;
    if (thresholdValue > 4095 || durationValue == 0 || durationValue > 60) {
      server.send(400, "text/plain", "BAD_VALUES");
      return;
    }
    th = thresholdValue;
    dur = durationValue;
    if (doc["name"].is<const char*>()) { name = String((const char*)doc["name"]); hasNameField = true; }
    if (doc["workerName"].is<const char*>()) { name = String((const char*)doc["workerName"]); hasNameField = true; }
    if (doc["potIndex"].is<int>()) potIndex = (int)doc["potIndex"];
  }
  bool ok = updateWorkerByHex(mac.c_str(), th, dur, hasNameField ? name.c_str() : nullptr, potIndex);
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerWater() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  if (getAutoEnabled()) { server.send(409, "text/plain", "AUTO_MODE"); return; }
  String body = server.arg("plain");
  String mac=""; int dur = -1;
  int potIndex = -1;
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (!err) {
      mac = String((const char*)(doc["mac"] | ""));
      if (doc["duration"].is<int>()) dur = (int)doc["duration"];
      if (doc["potIndex"].is<int>()) potIndex = (int)doc["potIndex"];
    }
  }
  uint8_t macb[6];
  if (!macFromHexString(mac.c_str(), macb)) { server.send(400, "text/plain", "BAD_MAC"); return; }
  WorkerConfig worker{};
  if (!findWorkerConfigByMac(macb, worker)) {
    server.send(400, "text/plain", "NO_WORKER");
    return;
  }
  if (dur > 60) { server.send(400, "text/plain", "BAD_DURATION"); return; }

  uint16_t potMask = 0;
  uint16_t durations[MAX_POTS_PER_DEVICE] = {};
  int durCount = 0;
  if (potIndex >= 0) {
    if (potIndex >= worker.potCount) {
      server.send(400, "text/plain", "BAD_POT_INDEX");
      return;
    }
    if (dur <= 0) dur = worker.durations[potIndex];
    potMask = (uint16_t)(1u << potIndex);
    durations[durCount++] = (uint16_t)dur;
  } else {
    int potCount = min(static_cast<int>(worker.potCount), MAX_POTS_PER_DEVICE);
    for (int p = 0; p < potCount; ++p) {
      potMask |= (uint16_t)(1u << p);
      durations[durCount++] =
          dur > 0 ? static_cast<uint16_t>(dur) : worker.durations[p];
    }
  }
  bool queued = wateringQueueManual(macb, potMask, durations, durCount);
  server.send(queued ? 202 : 503, "text/plain",
              queued ? "QUEUED" : "BUSY");
}

void webBegin() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/diagnostics", HTTP_GET, handleDiagnostics);
  server.on("/pump/toggle", HTTP_POST, handlePumpToggle);
  server.on("/settings", HTTP_ANY, handleSettings);
  server.on("/time/sync", HTTP_POST, handleTimeSync);
  server.on("/nodes", HTTP_GET, handleNodes);
  server.on("/worker/add", HTTP_POST, handleWorkerAdd);
  server.on("/worker/remove", HTTP_POST, handleWorkerRemove);
  server.on("/worker/update", HTTP_POST, handleWorkerUpdate);
  server.on("/worker/water", HTTP_POST, handleWorkerWater);
  server.on("/wifi/clear", HTTP_POST, handleClearWifiCred);
  server.on("/settings/clear_all", HTTP_POST, handleClearAllSettings);
  server.begin();
  // create one-shot pump-off timer (no heap allocation per request)
  if (!sPumpOffTimer) sPumpOffTimer = xTimerCreate("pumpOff", pdMS_TO_TICKS(5000), pdFALSE, NULL, pumpOffTimerCallback);
  // OTA handled via ArduinoOTA in main setup
}

void webHandleClientLoop() {
  server.handleClient();
}
