#include "WebUI.h"
#include "Config.h"
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

static WebServer server(80);

// background task to switch the pump off after a delay (non-blocking web handler)
static void pumpOffTask(void *pv) {
  uint32_t delay_ms = 5000;
  if (pv) {
    uint32_t *p = (uint32_t*)pv;
    delay_ms = *p;
    delete p;
  }
  vTaskDelay(pdMS_TO_TICKS(delay_ms));
  pumpOff();
  vTaskDelete(NULL);
}

static String makeStatusJson() {
  int tank = getTankLevel();
  unsigned long now = millis()/1000;
  uint32_t tod = getCurrentTimeOfDaySec();
  JsonDocument doc;
  doc["name"] = settings.name;
  doc["auto"] = autoEnabled;
  doc["tank"] = tank;
  doc["now"] = now;
  doc["timeOfDaySec"] = tod;
  doc["waterInterval"] = settings.waterInterval;
  doc["dataSyncInterval"] = settings.dataSyncInterval;
  doc["tzOffsetMinutes"] = settings.tzOffsetMinutes;
  doc["activeStart"] = settings.activeStart;
  doc["activeEnd"] = settings.activeEnd;
  uint64_t curEpoch = getCurrentEpochSec();
  if (curEpoch) doc["epoch"] = curEpoch;
  // diagnostics
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t heapSize = ESP.getHeapSize();
  uint32_t freeSketch = ESP.getFreeSketchSpace();
  uint32_t sketchSize = ESP.getSketchSize();
  uint32_t flashChipSize = ESP.getFlashChipSize();
  uint32_t chipRev = ESP.getChipRevision();
  doc["freeHeap"] = freeHeap;
  doc["heapSize"] = heapSize;
  doc["freeSketchSpace"] = freeSketch;
  doc["sketchSize"] = sketchSize;
  doc["flashChipSize"] = flashChipSize;
  doc["chipRevision"] = chipRev;
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
  String json;
  serializeJson(doc, json);
  return json;
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
      label{margin-top:6px;display:block;}
      button{padding:6px 10px;margin-top:6px; margin-left:6px}
      table{border-collapse:collapse;width:100%}
    th,td{border:1px solid #ddd;padding:6px;text-align:left}
    /* pot rows are indented to visually separate from worker headers */
    .pot{margin-left:18px;padding-left:8px;border-left:2px solid #eee;background:#fafafa}
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
    <div>Last auto watering (epoch s): <span id="last">-</span> ago</div>
    <div>
      <button onclick="togglePump()">Toggle Pump 5s</button>
    </div>

    <h3>Settings</h3>
    <div>
      <label>Name: <input id="name" maxlength="255"></label>
    </div>
    <div>
      <label>Current time (HH:MM): <input id="newTime" type="time"></label>
    </div>
    <div>
      <label>Timezone offset (±HH:MM): <input id="tzOffset" placeholder="+01:00 or -05:30"></label>
    </div>
    <div>
      <label>Active start (HH:MM): <input id="activeStart" type="time"></label>
    </div>
    <div>
      <label>Active end (HH:MM): <input id="activeEnd" type="time"></label>
    </div>
    <div>
      <button id="syncTimeBtn" onclick="syncTime()">Sync time (NTP)</button>
      <span id="syncStatus" style="margin-left:8px"></span>
    </div>
    <div>
      <label>Water interval (s): <input id="newWint" type="number" placeholder="60-2419200(28 days)" min="60" max="2419200"></label>
    </div>
    <div>
      <label>Auto: <input id="newAuto" type="checkbox"></label>
    </div>
    <div>
      <button id="applySettingsBtn" onclick="applySettings()">Apply Settings</button>
      <span id="settingsStatus" style="margin-left:8px"></span>
    </div>
    <div>
      <button id="clearWifiBtn" onclick="clearWifiCredConfirm()">Clear WiFi Credentials</button>
      <button id="clearAllBtn" onclick="clearAllSettingsConfirm()" style="margin-left:8px">Reset All Settings</button>
      <span id="wifiStatus" style="margin-left:8px"></span>
      <span id="resetStatus" style="margin-left:8px"></span>
    </div>

    <h3 style="margin-top:14px">Workers</h3>
    <div>
      <form id="addForm" onsubmit="return false;">
          <label>MAC (hex): <input id="wmac" placeholder="AABBCCDDEEFF"></label>
          <label>Name: <input id="wname" maxlength="31" placeholder="Worker name(31 characters max)" style="width:140px"></label>
          <!-- Threshold and duration removed from quick add; defaults used -->
        <button onclick="addWorker()">Add Worker</button>
      </form>
    </div>
    <div style="margin-top:8px">
      <div style="margin-bottom:6px">
        <button id="applyAllBtn" onclick="applyAll()" disabled>Apply All</button>
      </div>
      <div id="workersList"></div>
    </div>

    <div style="margin-top:18px;">
      <h3>Diagnostics</h3>
      <table id="diagTable"><tr><th>Metric</th><th>Value</th></tr></table>
    </div>

    <script>
    const dirty = new Set();
    const pending = new Set();

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

    function getNodePayloadEl(el){
      return {
        mac: el.dataset.mac,
        potIndex: parseInt(el.dataset.pi) || 0,
        name: el.querySelector('.iname').value.trim(),
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
        const el = document.querySelector(`.node[data-id="${ids[i]}"]`);
        if (el) await applyNode(el);
      }
      // refresh list after
      setTimeout(refreshWorkers,300);
    }

    async function refreshWorkers(){
      let r = await fetch('/nodes');
      let list = await r.json();
      let container = document.getElementById('workersList');
      container.innerHTML = '';
      const statusTimers = {};

      function setStatus(id, msg, ok) {
        const el = document.querySelector(`.node[data-id="${id}"]`);
        if (!el) return;
        const span = el.querySelector('.status');
        span.innerText = msg || '';
        span.style.color = ok ? 'green' : 'crimson';
        if (statusTimers[id]) clearTimeout(statusTimers[id]);
        if (msg) statusTimers[id] = setTimeout(()=>{ span.innerText=''; delete statusTimers[id]; }, 3000);
      }

      function fmtTime(v){
        if (v===null || v===undefined) return '-';
        return formatEpoch(v);
      }

      // Group returned flat config list by MAC
      const map = {};
      for (let i=0;i<list.length;i++){
        const w = list[i];
        if (!map[w.mac]) map[w.mac] = { mac: w.mac, battery: null, lastSync: null, lastWater: null, nodePotCount: w.nodePotCount, pots: [] };
        // update worker-level info if available
        if (w.battery !== null && map[w.mac].battery === null) map[w.mac].battery = w.battery;
        if (w.lastSync !== null && map[w.mac].lastSync === null) map[w.mac].lastSync = w.lastSync;
        if (w.lastWater !== null && map[w.mac].lastWater === null) map[w.mac].lastWater = w.lastWater;
        if (w.nodePotCount !== undefined) map[w.mac].nodePotCount = w.nodePotCount;
        // collect per-pot config
        map[w.mac].pots.push({ potIndex: (typeof w.potIndex === 'number' ? w.potIndex : 0), name: w.name, threshold: w.threshold, duration: w.duration, soil: w.soil });
      }

      // sort pot lists by potIndex for deterministic ordering
      for (const m in map) {
        map[m].pots.sort((a,b)=> (a.potIndex - b.potIndex));
      }

      let idx = 0;
      for (const mac in map) {
        idx++;
        const worker = map[mac];
        // worker header
        const workerDiv = document.createElement('div');
        workerDiv.className = 'worker';
        workerDiv.dataset.mac = worker.mac;
        workerDiv.style.padding = '6px 0';
        workerDiv.innerHTML = `
          <hr>
          <div><strong>${idx}. ${mac}</strong></div>
          <div>
            <label>Worker name: <input class="wname" value="${escapeHtml(worker.pots[0] ? worker.pots[0].name : '')}" maxlength="31" style="width:160px"></label>
            <span style="margin-left:8px;color:#666">Battery: ${worker.battery===null?'-':worker.battery} LastSync: ${fmtTime(worker.lastSync)}</span>
            <button class="applyWorkerBtn" style="margin-left:8px">Apply</button>
            <button class="removeAllBtn" style="margin-left:8px">Remove</button>
          </div>
        `;
        container.appendChild(workerDiv);
        // bind apply for this worker (apply all dirty pots for the worker)
        const applyWorkerBtn = workerDiv.querySelector('.applyWorkerBtn');
        applyWorkerBtn.addEventListener('click', async ()=>{
          if (pending.size) return;
          const nodes = workerDiv.querySelectorAll('.node');
          for (let i=0;i<nodes.length;i++){
            const el = nodes[i];
            if (dirty.has(el.dataset.id)) await applyNode(el);
          }
          setTimeout(refreshWorkers,300);
        });
        // bind remove all
        const removeAllBtn = workerDiv.querySelector('.removeAllBtn');
        removeAllBtn.addEventListener('click', async ()=> {
          setStatus(mac+'-all', 'Removing...', false);
          const ok = await removeWorker(mac);
          if (ok) { setStatus(mac+'-all', 'Removed', true); setTimeout(refreshWorkers,300); } else setStatus(mac+'-all', 'Remove failed', false);
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
              <div><strong>Pot ${pot.potIndex+1}</strong> <label style="margin-left:8px">Name: <input class="iname" value="${escapeHtml(pot.name||'')}" maxlength="31" style="width:160px"></label>
                <label style="margin-left:8px">Threshold: <input class="ith" type="number" value="${pot.threshold}" min="0" max="4095" style="width:90px"></label>
                <label style="margin-left:8px">Duration(s): <input class="idur" type="number" value="${pot.duration}" min="1" max="60" style="width:80px"></label>
              </div>
              <div>
                <button class="waterBtn">Water</button>
                <span class="status" style="margin-left:8px"></span>
                <span class="soil" style="margin-left:8px;color:#666">Soil: ${pot.soil===null?'-':pot.soil}</span>
                <span class="lastwater" style="margin-left:8px;color:#666">LastWater: ${fmtTime(pot.lastWater)}</span>
              </div>
            `;
            const iname = div.querySelector('.iname');
            const ith = div.querySelector('.ith');
            const idur = div.querySelector('.idur');
            iname.addEventListener('input', ()=> markDirty(id));
            ith.addEventListener('input', ()=> markDirty(id));
            idur.addEventListener('input', ()=> markDirty(id));
            const waterBtn = div.querySelector('.waterBtn');
            // display sensor status and disable water button if sensor not connected (soil < 50)
            const soilSpan = div.querySelector('.soil');
            const lastSpan = div.querySelector('.lastwater');
            if (pot.soil !== null && pot.soil < 50) {
              soilSpan.innerText = 'Sensor not connected';
              if (waterBtn) waterBtn.disabled = true;
            } else {
              soilSpan.innerText = (pot.soil===null?'-':pot.soil);
              if (waterBtn) waterBtn.disabled = false;
            }
            if (lastSpan) lastSpan.innerText = 'LastWater: ' + fmtTime(pot.lastWater);
            waterBtn.addEventListener('click', async ()=> { setStatus(id, 'Sending...', false); const dur = parseInt(idur.value) || pot.duration; const ok = await waterWorker(div.dataset.mac, parseInt(div.dataset.pi), dur); if (ok) setStatus(id, 'Water sent', true); else setStatus(id, 'Send failed', false); });
            workerDiv.appendChild(div);
          }
        }
      }
      updateButtons();
    }

    function escapeHtml(s){ return (s||'').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }

    function formatEpoch(epoch){
      if (!epoch || epoch===null || epoch===undefined) return '-';
      try {
        // epoch is seconds; show relative if within a day, otherwise ISO date
        const now = Math.floor(Date.now()/1000);
        const d = now - epoch;
        if (d < 0) return 'in future';
        if (d < 60) return d + 's ago';
        if (d < 3600) return Math.floor(d/60) + 'm ago';
        if (d < 86400) return Math.floor(d/3600) + 'h ago';
        const dt = new Date(epoch*1000);
        return dt.getFullYear()+'-'+String(dt.getMonth()+1).padStart(2,'0')+'-'+String(dt.getDate()).padStart(2,'0')+' '+String(dt.getHours()).padStart(2,'0')+':'+String(dt.getMinutes()).padStart(2,'0');
      } catch(e){ return String(epoch); }
    }

    async function addWorker(){
      const mac = document.getElementById('wmac').value.trim();
      const name = document.getElementById('wname').value.trim();
      const body = {mac:mac, name:name};
      await fetch('/worker/add', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
      setTimeout(refreshWorkers,300);
    }

    async function removeWorker(mac, potIndex){
      try {
        const body = {mac:mac};
        const res = await fetch('/worker/remove',{method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
        const ok = res.ok;
        if (ok) setTimeout(refreshWorkers,300);
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
      const body = {name:document.getElementById('name').value, auto:document.getElementById('newAuto').checked, wint:parseInt(document.getElementById('newWint').value)};
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
        setTimeout(refreshWorkers, 300);
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
        <tr><td>Total Heap (KiB)</td><td>${fmtKb(j.heapSize)}</td></tr>
        <tr><td>Free Sketch Space (KiB)</td><td>${fmtKb(j.freeSketchSpace)}</td></tr>
        <tr><td>Sketch Size (KiB)</td><td>${fmtKb(j.sketchSize)}</td></tr>
        <tr><td>Flash Chip Size (KiB)</td><td>${fmtKb(j.flashChipSize)}</td></tr>
        <tr><td>Chip Revision</td><td>${fmt(j.chipRevision)}</td></tr>
        <tr><td>NVS used entries</td><td>${fmt(j.nvs_entries)}</td></tr>
        <tr><td>NVS total entries</td><td>${fmt(j.nvs_total_entries)}</td></tr>
        <tr><td>WiFi RSSI (dBm)</td><td>${fmt(j.rssi)}</td></tr>
      `;
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

        if (typeof j.timeOfDaySec === 'number') {
          const hh = Math.floor(j.timeOfDaySec/3600)%24;
          const mm = Math.floor((j.timeOfDaySec%3600)/60);
          const s = String(hh).padStart(2,'0')+':' + String(mm).padStart(2,'0');
          const timeSpan = document.getElementById('time');
          if (timeSpan) timeSpan.innerText = s;
        }

        const autoSpan = document.getElementById('auto');
        if (autoSpan) autoSpan.innerText = j.auto ? "On" : "Off";

        const wintSpan = document.getElementById('wint');
        if (wintSpan) wintSpan.innerText = j.waterInterval || 3600;

        const title = document.getElementById('title');
        if (title) title.innerText = 'Plant Watering ' + (j.name?('(' + j.name + ')'):"" );

        // populate timezone display (do NOT overwrite tzOffset input) and active window inputs
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
          // populate active window inputs if available and not focused
          try {
            if (typeof j.activeStart === 'number') {
              const s = j.activeStart;
              const hh = Math.floor(s/3600)%24;
              const mm = Math.floor((s%3600)/60);
              const el = document.getElementById('activeStart');
              if (el && document.activeElement !== el) el.value = String(hh).padStart(2,'0')+':' + String(mm).padStart(2,'0');
            }
            if (typeof j.activeEnd === 'number') {
              const s = j.activeEnd;
              const hh = Math.floor(s/3600)%24;
              const mm = Math.floor((s%3600)/60);
              const el = document.getElementById('activeEnd');
              if (el && document.activeElement !== el) el.value = String(hh).padStart(2,'0')+':' + String(mm).padStart(2,'0');
            }
          } catch(e) {}
        } catch(e) {}

        renderDiagnostics(j);
      } catch (e) {
        console.error('fetchStatus', e);
      }
    }

    // Poll less frequently to reduce CPU/network and avoid clobbering user inputs
    setInterval(fetchStatus,3000);
    setInterval(refreshWorkers,5000);
    fetchStatus();
    refreshWorkers();
    </script>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleStatus() {
  server.send(200, "application/json", makeStatusJson());
}

void handleTimeSync() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  bool ok = trySyncNTP(10000);
  JsonDocument doc;
  doc["ok"] = ok;
  if (ok) {
    uint64_t epoch = getCurrentEpochSec();
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
  pumpOn();
  // schedule pump off in background task to avoid blocking web server
  uint32_t *delay_ms = new uint32_t(5000);
  xTaskCreate(pumpOffTask, "pumpOff", 2048, delay_ms, tskIDLE_PRIORITY+1, NULL);
  server.send(200, "text/plain", "OK");
}

void handleSettings() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (err) {
      server.send(400, "text/plain", "BAD_JSON");
      return;
    }
    // capture old tz for epoch adjustment
    int16_t oldTz = settings.tzOffsetMinutes;

    if (doc["name"].is<const char*>()) settings.name = String((const char*)doc["name"]);
    if (doc["auto"].is<bool>()) autoEnabled = (bool)doc["auto"];
    if (doc["wint"].is<uint32_t>()) settings.waterInterval = constrain((uint32_t)doc["wint"], 60, 2419200);
    // optional time input in "HH:MM" (24h). If provided, update saved time.
    if (doc["newTime"].is<const char*>()) {
      const char* tstr = doc["newTime"];
      int hh=0, mm=0;
      if (sscanf(tstr, "%d:%d", &hh, &mm) == 2) {
        if (hh >=0 && hh <24 && mm >=0 && mm <60) {
          uint32_t sec = (uint32_t)hh*3600u + (uint32_t)mm*60u;
          setUserTimeOfDaySec(sec);
        }
      }
    }
    // optional timezone input: accept tzOffsetMinutes or tzOffset string like +01:00
    if (doc["tzOffsetMinutes"].is<int>()) {
      int16_t newTz = (int16_t)(int)doc["tzOffsetMinutes"];
      if (settings.savedEpochSec != 0) {
        int32_t deltaMin = (int32_t)newTz - (int32_t)oldTz;
        int64_t newEpoch = (int64_t)settings.savedEpochSec + (int64_t)deltaMin * 60LL;
        if (newEpoch < 0) newEpoch = 0;
        settings.savedEpochSec = (uint64_t)newEpoch;
      }
      settings.tzOffsetMinutes = newTz;
    } else if (doc["tzOffset"].is<const char*>()) {
      const char* tzs = doc["tzOffset"];
      int hh=0, mm=0;
      if (sscanf(tzs, "%d:%d", &hh, &mm) == 2) {
        int sign = 1;
        if (tzs[0] == '-') sign = -1; else if (tzs[0] == '+') sign = 1;
        int ah = hh < 0 ? -hh : hh;
        int16_t newTz = (int16_t)(sign*(ah*60 + mm));
        if (settings.savedEpochSec != 0) {
          int32_t deltaMin = (int32_t)newTz - (int32_t)oldTz;
          int64_t newEpoch = (int64_t)settings.savedEpochSec + (int64_t)deltaMin * 60LL;
          if (newEpoch < 0) newEpoch = 0;
          settings.savedEpochSec = (uint64_t)newEpoch;
        }
        settings.tzOffsetMinutes = newTz;
      }
    }
      // optional active window (seconds since midnight)
      if (doc["activeStart"].is<uint32_t>()) settings.activeStart = (uint32_t)doc["activeStart"];
      if (doc["activeEnd"].is<uint32_t>()) settings.activeEnd = (uint32_t)doc["activeEnd"];
  }
  markSettingsDirty();
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
  // Return configured workers list with latest discovered readings when available
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  uint64_t nowEpoch = getCurrentEpochSec();
  unsigned long curBootSec = millis() / 1000;
  for (int i=0;i<workerListCount;i++) {
    const WorkerConfig &wc = workerList[i];
    char macs[32];
    sprintf(macs, "%02X%02X%02X%02X%02X%02X", wc.mac[0],wc.mac[1],wc.mac[2],wc.mac[3],wc.mac[4],wc.mac[5]);
    JsonObject o = arr.add<JsonObject>();
    o["mac"] = String(macs);
    o["potIndex"] = (int)wc.potIndex;
    o["name"] = String(wc.name);
    o["threshold"] = wc.threshold;
    o["duration"] = wc.duration;
    const WorkerNode* n = btMainFindNodeByMac(wc.mac);
    if (n) {
      if (wc.potIndex < n->potCount) o["soil"] = (int)n->soils[wc.potIndex]; else o["soil"] = nullptr;
      o["battery"] = n->battery;
      // convert node's boot-relative lastSync to epoch if available
      if (n->lastSync != 0 && nowEpoch != 0) o["lastSync"] = (uint64_t)(nowEpoch - (uint64_t)(curBootSec - n->lastSync)); else o["lastSync"] = nullptr;
      // prefer per-pot lastWater if set, otherwise compute a node-level fallback
      unsigned long perPot = 0;
      if (wc.potIndex < MAX_POTS_PER_DEVICE) perPot = n->lastWater[wc.potIndex];
      if (perPot != 0 && nowEpoch != 0) {
        o["lastWater"] = (uint64_t)(nowEpoch - (uint64_t)(curBootSec - perPot));
      } else {
        // find the most recent per-pot lastWater for this node
        unsigned long nodeLast = 0;
        int maxP = (n->potCount > 0) ? n->potCount : MAX_POTS_PER_DEVICE;
        if (maxP > MAX_POTS_PER_DEVICE) maxP = MAX_POTS_PER_DEVICE;
        for (int p = 0; p < maxP; ++p) {
          if (n->lastWater[p] > nodeLast) nodeLast = n->lastWater[p];
        }
        if (nodeLast != 0) {
          if (nodeLast >= 1600000000UL) o["lastWater"] = (uint64_t)nodeLast;
          else if (nowEpoch != 0) o["lastWater"] = (uint64_t)(nowEpoch - (uint64_t)(curBootSec - nodeLast));
          else o["lastWater"] = nullptr;
        } else {
          o["lastWater"] = nullptr;
        }
      }
      o["nodePotCount"] = n->potCount;
    } else {
      o["soil"] = nullptr;
      o["battery"] = nullptr;
      o["lastSync"] = nullptr;
      o["lastWater"] = nullptr;
      o["nodePotCount"] = nullptr;
    }
  }
  String out;
  serializeJson(arr, out);
  server.send(200, "application/json", out);
}

void handleWorkerAdd() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  // expect JSON: {"mac":"AABBCCDDEEFF","threshold":2000,"duration":5,"name":"plant name"}
  String mac=""; uint16_t th=2000; uint16_t dur=5; String name="";
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (!err) {
      mac = String((const char*)(doc["mac"] | ""));
      th = (uint16_t)constrain((uint32_t)(doc["threshold"] | 2000), 0, 4095);
      dur = (uint16_t)constrain((uint32_t)(doc["duration"] | 5), 1, 60);
      if (doc["name"].is<const char*>()) name = String((const char*)doc["name"]);
    }
  }
  bool ok = addWorkerByHex(mac, th, dur, name);
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
  bool ok = removeWorkerByHex(mac);
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerUpdate() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac=""; uint16_t th=2000; uint16_t dur=5; String name="";
  int potIndex = -1;
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (err) {
      server.send(400, "text/plain", "BAD_JSON");
      return;
    }
    mac = String((const char*)(doc["mac"] | ""));
    th = (uint16_t)constrain((uint32_t)(doc["threshold"] | 2000), 0, 4095);
    dur = (uint16_t)constrain((uint32_t)(doc["duration"] | 5), 1, 60);
    if (doc["name"].is<const char*>()) name = String((const char*)doc["name"]);
    if (doc["potIndex"].is<int>()) potIndex = (int)doc["potIndex"];
  }
  bool ok = updateWorkerByHex(mac, th, dur, name, potIndex);
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerWater() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
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
  if (!macFromHexString(mac, macb)) { server.send(400, "text/plain", "BAD_MAC"); return; }
  // find configured duration if not provided
  if (dur <= 0) {
    bool found = false;
    for (int i=0;i<workerListCount;i++) {
      bool same = true; for (int k=0;k<6;k++) if (workerList[i].mac[k]!=macb[k]) { same=false; break; }
      if (same) { dur = workerList[i].duration; found = true; break; }
    }
    if (!found) { server.send(400, "text/plain", "NO_DURATION"); return; }
  }
  // Build compact CMD_WATER payload: [TYPE_CMD_WATER][potMask(2)][duration(2)]
  uint16_t potMask = 0;
  if (potIndex >= 0) {
    if (potIndex >= MAX_POTS_PER_DEVICE) { server.send(400, "text/plain", "BAD_POT_INDEX"); return; }
    potMask = (uint16_t)(1u << potIndex);
  } else {
    // if no potIndex specified, pick first matching configured pot for this MAC
    for (int i=0;i<workerListCount;i++) {
      bool same = true; for (int k=0;k<6;k++) if (workerList[i].mac[k]!=macb[k]) { same=false; break; }
      if (same) { potMask |= (uint16_t)(1u << workerList[i].potIndex); }
    }
    if (potMask == 0) potMask = 1; // default to pot 0
  }
  uint8_t payload[5];
  payload[0] = BT_TLV::TYPE_CMD_WATER;
  payload[1] = (uint8_t)((potMask >> 8) & 0xFF);
  payload[2] = (uint8_t)(potMask & 0xFF);
  payload[3] = (uint8_t)((dur >> 8) & 0xFF);
  payload[4] = (uint8_t)(dur & 0xFF);
  bool queued = btMainQueueCommand(macb, payload, sizeof(payload), 2, 700);
  if (queued) {
    // record per-pot last water (boot seconds); UI will convert to epoch for display
    btMainSetNodeLastWater(macb, potMask, millis() / 1000);
    server.send(202, "text/plain", "QUEUED");
  } else server.send(500, "text/plain", "ERR");
}

void webBegin() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
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
  // OTA handled via ArduinoOTA in main setup
}

void webHandleClientLoop() {
  server.handleClient();
}
