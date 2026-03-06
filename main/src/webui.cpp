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

static String makeStatusJson() {
  int tank = readTankLevel();
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
      button{padding:6px 10px;margin-left:6px}
      table{border-collapse:collapse;width:100%}
      th,td{border:1px solid #ddd;padding:6px;text-align:left}
    </style>
  </head>
  <body>
    <h2 id="title">Plant Watering</h2>
    <div>Current time (HH:MM): <span id="time">-</span></div>
    <div>Tank level: <span id="tank">-</span></div>
    <div>Last watering end (epoch s): <span id="last">-</span></div>
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
      <label>Water interval (s): <input id="wint" type="number" min="60" max="2419200"></label>
    </div>
    <div>
      <label>Auto: <input id="auto" type="checkbox"></label>
    </div>
    <div>
      <button id="applySettingsBtn" onclick="applySettings()">Apply Settings</button>
      <span id="settingsStatus" style="margin-left:8px"></span>
    </div>

    <h3 style="margin-top:14px">Workers</h3>
    <div>
      <form id="addForm" onsubmit="return false;">
          <label>MAC (hex): <input id="wmac" placeholder="AABBCCDDEEFF"></label>
          <label>Name: <input id="wname" maxlength="31" placeholder="Worker name" style="width:140px"></label>
          <label>Threshold: <input id="wth" type="number" value="2000" min="0" max="4095" style="width:90px"></label>
          <label>Duration(s): <input id="wdur" type="number" value="5" min="1" max="600" style="width:80px"></label>
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
      document.querySelectorAll('.node').forEach(el=>{
        const id = el.dataset.mac;
        const applyBtn = el.querySelector('.applyBtn');
        applyBtn.disabled = !dirty.has(id) || pending.has(id);
      });
      document.getElementById('applyAllBtn').disabled = dirty.size === 0 || pending.size > 0;
    }

    function getNodePayload(mac){
      const el = document.querySelector(`.node[data-mac="${mac}"]`);
      return {
        mac: mac,
        name: el.querySelector('.iname').value.trim(),
        threshold: parseInt(el.querySelector('.ith').value) || 2000,
        duration: parseInt(el.querySelector('.idur').value) || 5
      };
    }

    async function applyNode(mac){
      if (pending.has(mac)) return;
      pending.add(mac); updateButtons();
      const payload = getNodePayload(mac);
      try {
        const res = await fetch('/worker/update', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)});
        if (!res.ok) throw new Error(await res.text());
        dirty.delete(mac);
        setStatus(mac, 'Saved', true);
      } catch (e) {
        console.error('applyNode', mac, e);
        setStatus(mac, 'Save failed', false);
      } finally {
        pending.delete(mac); updateButtons();
      }
    }

    async function applyAll(){
      if (pending.size) return;
      const ids = Array.from(dirty);
      if (ids.length===0) return;
      // apply sequentially to avoid flooding
      for (let i=0;i<ids.length;i++){
        await applyNode(ids[i]);
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

      function setStatus(mac, msg, ok) {
        const el = document.querySelector(`.node[data-mac="${mac}"]`);
        if (!el) return;
        const span = el.querySelector('.status');
        span.innerText = msg || '';
        span.style.color = ok ? 'green' : 'crimson';
        if (statusTimers[mac]) clearTimeout(statusTimers[mac]);
        if (msg) statusTimers[mac] = setTimeout(()=>{ span.innerText=''; delete statusTimers[mac]; }, 3000);
      }

      function fmtTime(v){
        if (v===null || v===undefined) return '-';
        return formatEpoch(v);
      }

      for (let i=0;i<list.length;i++){
        let w = list[i];
        let div = document.createElement('div');
        div.className = 'node';
        div.dataset.mac = w.mac;
        div.style.padding = '6px 0';
        div.innerHTML = `
          <hr>
          <div><strong>${w.mac}</strong></div>
          <div>
            <label>Name: <input class="iname" value="${escapeHtml(w.name||'')}" maxlength="31" style="width:160px"></label>
            <label style="margin-left:8px">Threshold: <input class="ith" type="number" value="${w.threshold}" min="0" max="4095" style="width:90px"></label>
            <label style="margin-left:8px">Duration(s): <input class="idur" type="number" value="${w.duration}" min="1" max="600" style="width:80px"></label>
          </div>
          <div>
            <button class="applyBtn" disabled>Apply</button>
            <button class="removeBtn">Remove</button>
            <button class="waterBtn">Water</button>
            <span class="status" style="margin-left:8px"></span>
              <span style="margin-left:8px;color:#666">Soil: ${w.soil===null?'-':w.soil} Battery: ${w.battery===null?'-':w.battery} LastSeen: ${fmtTime(w.lastSeen)} LastWater: ${fmtTime(w.lastWater)}</span>
          </div>
        `;
        // attach events
        div.querySelector('.iname').addEventListener('input', ()=> markDirty(w.mac));
        div.querySelector('.ith').addEventListener('input', ()=> markDirty(w.mac));
        div.querySelector('.idur').addEventListener('input', ()=> markDirty(w.mac));
        div.querySelector('.applyBtn').addEventListener('click', ()=> applyNode(w.mac));
        div.querySelector('.removeBtn').addEventListener('click', async ()=> { setStatus(w.mac, 'Removing...', false); const ok = await removeWorker(w.mac); if (ok) { setStatus(w.mac, 'Removed', true); setTimeout(refreshWorkers,300); } else setStatus(w.mac, 'Remove failed', false); });
        div.querySelector('.waterBtn').addEventListener('click', async ()=> { setStatus(w.mac, 'Sending...', false); const ok = await waterWorker(w.mac, w.duration); if (ok) setStatus(w.mac, 'Water sent', true); else setStatus(w.mac, 'Send failed', false); });
        container.appendChild(div);
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
      const th = parseInt(document.getElementById('wth').value)||2000;
      const dur = parseInt(document.getElementById('wdur').value)||5;
      const body = {mac:mac, name:name, threshold:th, duration:dur};
      await fetch('/worker/add', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
      setTimeout(refreshWorkers,300);
    }

    async function removeWorker(mac){
      try {
        const res = await fetch('/worker/remove',{method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({mac:mac})});
        const ok = res.ok;
        if (ok) setTimeout(refreshWorkers,300);
        return ok;
      } catch (e) { return false; }
    }

    async function waterWorker(mac, duration){
      try {
        const res = await fetch('/worker/water',{method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({mac:mac, duration:duration})});
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
      const body = {name:document.getElementById('name').value, auto:document.getElementById('auto').checked, wint:parseInt(document.getElementById('wint').value)};
      if (timeVal) body.newTime = timeVal;
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
      let r = await fetch('/status');
      let j = await r.json();
      document.getElementById('tank').innerText = j.tank;
      document.getElementById('name').value = j.name || '';
      if (typeof j.timeOfDaySec === 'number') {
        const hh = Math.floor(j.timeOfDaySec/3600)%24;
        const mm = Math.floor((j.timeOfDaySec%3600)/60);
        const s = String(hh).padStart(2,'0')+':' + String(mm).padStart(2,'0');
        document.getElementById('time').value = s;
      }
      document.getElementById('auto').checked = j.auto || false;
      document.getElementById('wint').value = j.waterInterval || 3600;
      document.getElementById('title').innerText = 'Plant Watering ' + (j.name?('(' + j.name + ')'):"");
      renderDiagnostics(j);
    }

    setInterval(fetchStatus,1000);
    setInterval(refreshWorkers,1000);
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

void handlePumpToggle() {
  pumpOn();
  delay(5000);
  pumpOff();
  server.send(200, "text/plain", "OK");
}

void handleSettings() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (!err) {
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
    }
  }
  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleNodes() {
  // Return configured workers list with latest discovered readings when available
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (int i=0;i<workerListCount;i++) {
    const WorkerConfig &wc = workerList[i];
    char macs[32];
    sprintf(macs, "%02X%02X%02X%02X%02X%02X", wc.mac[0],wc.mac[1],wc.mac[2],wc.mac[3],wc.mac[4],wc.mac[5]);
    JsonObject o = arr.add<JsonObject>();
    o["mac"] = String(macs);
    o["name"] = String(wc.name);
    o["threshold"] = wc.threshold;
    o["duration"] = wc.duration;
    const WorkerNode* n = btMainFindNodeByMac(wc.mac);
    if (n) {
      o["soil"] = n->soil;
      o["battery"] = n->battery;
      o["lastSeen"] = n->lastSeen;
      o["lastWater"] = n->lastWater;
    } else {
      o["soil"] = nullptr;
      o["battery"] = nullptr;
      o["lastSeen"] = nullptr;
      o["lastWater"] = nullptr;
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
      dur = (uint16_t)constrain((uint32_t)(doc["duration"] | 5), 1, 600);
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
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (!err) {
      mac = String((const char*)(doc["mac"] | ""));
      th = (uint16_t)constrain((uint32_t)(doc["threshold"] | 2000), 0, 4095);
      dur = (uint16_t)constrain((uint32_t)(doc["duration"] | 5), 1, 600);
      if (doc["name"].is<const char*>()) name = String((const char*)doc["name"]);
    }
  }
  bool ok = updateWorkerByHex(mac, th, dur, name);
  server.send(ok?200:400, "text/plain", ok?"OK":"ERR");
}

void handleWorkerWater() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg("plain");
  String mac=""; int dur = -1;
  if (body.length()) {
    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    if (!err) {
      mac = String((const char*)(doc["mac"] | ""));
      if (doc["duration"].is<int>()) dur = (int)doc["duration"];
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
  uint8_t payload[3]; payload[0] = 0x03; payload[1] = (uint8_t)(dur>>8); payload[2] = (uint8_t)(dur & 0xFF);
  bool queued = btMainQueueCommand(macb, payload, sizeof(payload), 2, 700);
  if (queued) server.send(202, "text/plain", "QUEUED");
  else server.send(500, "text/plain", "ERR");
}

void webBegin() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/pump/toggle", HTTP_POST, handlePumpToggle);
  server.on("/settings", HTTP_ANY, handleSettings);
  server.on("/nodes", HTTP_GET, handleNodes);
  server.on("/worker/add", HTTP_POST, handleWorkerAdd);
  server.on("/worker/remove", HTTP_POST, handleWorkerRemove);
  server.on("/worker/update", HTTP_POST, handleWorkerUpdate);
  server.on("/worker/water", HTTP_POST, handleWorkerWater);
  server.begin();
  // OTA handled via ArduinoOTA in main setup
}

void webHandleClientLoop() {
  server.handleClient();
}
