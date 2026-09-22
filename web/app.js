/* Front end for the io-homecontrol rig.
 *
 * No framework and no build step beyond gzipping: this is served from an
 * ESP32-C6 that also has a radio to run, and every kilobyte is flash that the
 * firmware could have used. It is one file because the page is fetched over a
 * single synchronous HTTP server -- three assets would be three round trips
 * served one after another.
 *
 * Polling, not push. A websocket would be tidier, but the device has to hold
 * the connection open across OTA updates, reboots and a radio that
 * monopolises the CPU for seconds at a time while it keys a command; a poll
 * that misses simply happens again three seconds later.
 *
 * Re-rendering rule: the Rooms tab and anything without a text field is redrawn
 * on every poll. The forms on Devices and Settings are drawn when you open the
 * tab and left alone afterwards, because rebuilding them under someone who is
 * typing a room name into them is worse than showing a value three seconds old.
 */
'use strict';

const $  = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

let S = null;            // last /api/state document
let tab = 'rooms';
let pollTimer = null;
let logCursor = null;    // absolute character position in the rig's output ring
let logFollow = true;
let draggingSlider = null;   // node whose slider is under a finger right now
let openEditor = null;       // node whose edit form is open, so it survives a redraw

/* ------------------------------------------------------------------ util */

function esc(s) {
  return String(s == null ? '' : s).replace(/[&<>"']/g,
    c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
}

/** "4 s", "3 min", "2 h" -- or "never" for the registry's zero, which is how
 *  it reports something that has not happened rather than something that
 *  happened at boot. */
function ago(s) {
  if (!s) return 'never';
  if (s < 60) return s + ' s';
  if (s < 3600) return Math.round(s / 60) + ' min';
  if (s < 86400) return Math.round(s / 3600) + ' h';
  return Math.round(s / 86400) + ' d';
}

/** As above, phrased as a moment in the past. Only for things that definitely
 *  happened, where the caller has already established that much -- otherwise
 *  a zero would read as "never ago". */
function since(s) {
  return s < 2 ? 'just now' : ago(s) + ' ago';
}

let toastTimer = null;
function toast(msg, bad) {
  const el = $('#toast');
  el.textContent = msg;
  el.classList.toggle('bad', !!bad);
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), bad ? 4200 : 2200);
}

async function post(path, params) {
  const body = new URLSearchParams(params).toString();
  const res = await fetch(path, {
    method: 'POST',
    // X-Velux is what tells the rig this came from its own page. A form on
    // another site can make a browser post here, but it cannot add a header,
    // and a script that tries needs a CORS preflight the rig never answers.
    headers: {'Content-Type': 'application/x-www-form-urlencoded', 'X-Velux': '1'},
    body
  });
  let data = {};
  try { data = await res.json(); } catch (e) { /* an error page, not JSON */ }
  if (!res.ok || data.ok === false) {
    throw new Error(data.error || ('HTTP ' + res.status));
  }
  return data;
}

/** Post, report what happened, and pull fresh state. */
async function act(path, params, okMsg) {
  try {
    const r = await post(path, params);
    if (okMsg) toast(okMsg);
    await poll();
    return r;
  } catch (e) {
    toast(e.message, true);
    throw e;
  }
}

/* ----------------------------------------------------------------- icons */

const ICONS = {
  // A skylight: a pitched opening with the sash swung out.
  window: '<svg viewBox="0 0 24 24"><path d="M4 20V9l8-5 8 5v11z"/><path d="M4 20l16-6"/></svg>',
  // A blind: a head rail with slats below it.
  blind: '<svg viewBox="0 0 24 24"><path d="M3 4h18"/><path d="M5 8h14M5 12h14M5 16h14"/><path d="M12 16v4"/></svg>',
  other: '<svg viewBox="0 0 24 24"><rect x="4" y="4" width="16" height="16" rx="3"/><path d="M9 12h6"/></svg>'
};

function iconFor(type) {
  if (type === 4) return ICONS.window;                 // window opener
  if (type === 1 || type === 2 || type === 10 || type === 11) return ICONS.blind;
  return ICONS.other;
}

/* ------------------------------------------------------- rooms (tab one) */

/** Devices, grouped by room, rooms in alphabetical order with anything
 *  unassigned last -- so the list does not reshuffle as names are edited. */
function grouped(devices) {
  const map = new Map();
  for (const d of devices) {
    const g = d.group || 'Unassigned';
    if (!map.has(g)) map.set(g, []);
    map.get(g).push(d);
  }
  return [...map.entries()].sort((a, b) => {
    if (a[0] === 'Unassigned') return 1;
    if (b[0] === 'Unassigned') return -1;
    return a[0].localeCompare(b[0]);
  });
}

/** The line under a device's name: what it is and what it is doing. */
function statusLine(d) {
  if (!d.hasKey) return 'No key bound &mdash; it can be read but not moved';
  if (d.err) return '<span style="color:var(--bad)">Refused: ' + errName(d.err) + '</span>';
  if (d.moving && d.target >= 0) return 'Moving to ' + d.target + '% open&hellip;';
  if (d.pos < 0) return d.typeName + ' &middot; position not read yet';
  return d.pos + '% open &middot; read ' + since(d.posAgoS);
}

function errName(code) {
  return ({
    3:  'operated by hand',
    7:  'reached the wrong position',
    8:  'execution error, or unsupported',
    24: 'automatic cycle engaged',
    33: 'wrong position',
    35: 'no intermediate position set',
    88: 'request rejected'
  })[code] || ('code 0x' + code.toString(16).toUpperCase());
}

function deviceCard(d) {
  // A device that has never been read must not look like one that is closed:
  // both would draw an empty bar. An unknown position hatches the whole track
  // instead, and the read-out says so, because these actuators report nothing
  // until they are asked and "shut" is a claim worth not making by accident.
  // While a move is in flight the last reading is stale by definition -- these
  // actuators say nothing until asked -- so the bar shows where it is heading,
  // pulsing, and the line above says so in words.
  const known = d.pos >= 0 || d.target >= 0;
  const pct = (d.moving && d.target >= 0) ? d.target : (d.pos >= 0 ? d.pos : 0);
  const cls = 'bar' + (known ? '' : ' unknown') + (d.moving ? ' moving' : '');
  // Without a key the actuator will challenge us and we cannot answer, so the
  // command would cost four seconds of keying the radio to achieve nothing.
  // Reading the position needs no key, so that button stays live.
  const off = d.hasKey ? '' : ' disabled';
  return `
  <div class="dev" data-node="${d.node}">
    <div class="dev-top">
      <div class="dev-icon">${iconFor(d.type)}</div>
      <div class="dev-name">
        <b>${esc(d.label)}</b>
        <small>${statusLine(d)}</small>
      </div>
      <button class="btn small" data-act="status" title="Read the position">&#8635;</button>
    </div>
    <div class="${cls}"><i style="width:${known ? pct : 100}%"></i></div>
    <div class="row">
      <button class="btn grow" data-act="close"${off}>Close</button>
      <button class="btn grow" data-act="stop"${off}>Stop</button>
      <button class="btn grow primary" data-act="open"${off}>Open</button>
    </div>
    <div class="row">
      <input type="range" min="0" max="100" step="5" value="${pct}" data-act="slider"
             aria-label="Percent open"${off}>
      <span class="pct">${known ? pct + '%' : '&mdash;'}</span>
    </div>
  </div>`;
}

/* Rebuilding a block's innerHTML throws away whatever the user was doing
 * inside it -- a half-typed room name, a button mid-press, a slider under a
 * finger -- so every block redrawn by the poll is written through here and
 * only actually replaced when the markup differs.
 *
 * The comparison is on the rendered markup rather than on the state it came
 * from, because the state never stops changing: seenAgoS and posAgoS count up
 * every second, so a signature taken from them would differ on every poll and
 * the check would never once fire. The markup only changes when something a
 * reader would notice has changed. */
const sigs = {};
function setHtml(key, host, html) {
  if (sigs[key] === html) return false;
  sigs[key] = html;
  host.innerHTML = html;
  return true;
}

function renderRooms() {
  const groups = grouped(S.devices);
  $('#rooms-empty').hidden = groups.length > 0;

  setHtml('rooms', $('#rooms'), groups.map(([name, devs]) => `
    <div class="card">
      <div class="group-head">
        <h2>${esc(name)}</h2>
        <button class="btn small" data-group="${esc(name)}" data-act="close">Close all</button>
        <button class="btn small" data-group="${esc(name)}" data-act="open">Open all</button>
      </div>
      ${devs.map(deviceCard).join('')}
    </div>`).join(''));
}

/** Update the Rooms tab, leaving a slider alone while it is held: redrawing
 *  would snatch the control out from under the finger. */
function refreshRooms() {
  if (!draggingSlider) renderRooms();
}

/* ------------------------------------------------------ devices (tab two) */

function renderPairLive() {
  const host = $('#pair-live');
  // The capture panel holds the room-name field. Someone typing into it while
  // the poll ticks must not have it emptied under them, so a focused field
  // holds the redraw off until they are done with it.
  //
  // A focused *button* deliberately does not: clicking Save leaves the focus
  // on the button, and if that counted the panel would never refresh to show
  // that the key had been filed away.
  const busy = document.activeElement;
  if (host.contains(busy) && /^(INPUT|SELECT|TEXTAREA)$/.test(busy.tagName)) return;

  const p = S.pair;
  const log = p.log.map(e =>
    `<div class="kv"><span>${esc(e.text)}</span><b>${since(e.agoS)}</b></div>`).join('');

  let capture = '';
  if (p.haveCapture) {
    const known = p.existingSlot >= 0
      ? `<p class="hint">This key is already in slot ${p.existingSlot}. Saving it
         again will not use up another slot.</p>` : '';
    capture = `
      <div class="editor">
        <h2>Key captured</h2>
        <p class="hint">
          key#${esc(p.fp)} from remote ${esc(p.remote)}, ${since(p.capturedAgoS)}.
          Name the room it covers and save it &mdash; it is held in memory only
          until you do.
        </p>
        ${known}
        <label class="field"><span>Room</span>
          <input type="text" id="pair-room" placeholder="bathroom" maxlength="19"></label>
        <div class="row">
          <button class="btn grow danger" id="pair-discard">Discard</button>
          <button class="btn grow primary" id="pair-save">Save key</button>
        </div>
      </div>`;
  }

  // Handlers are bound below, so binding only makes sense when the markup was
  // actually replaced; otherwise the existing elements still carry them.
  if (!setHtml('pair', host, `
    <div class="row">
      <div class="pill ${p.armed ? 'busy' : ''}">${p.armed ? 'Armed &mdash; waiting for the remote' : 'Not armed'}</div>
      <div style="flex:1"></div>
      <button class="btn ${p.armed ? 'danger' : 'primary'}" id="pair-toggle">
        ${p.armed ? 'Disarm' : 'Arm pairing'}</button>
    </div>
    ${capture}
    ${log ? '<div class="editor">' + log + '</div>' : ''}`)) return;

  $('#pair-toggle').onclick = () =>
    act('/api/pair', {on: p.armed ? '0' : '1'}, p.armed ? 'Disarmed' : 'Armed');

  if (p.haveCapture) {
    $('#pair-save').onclick = () =>
      act('/api/pair/commit', {name: $('#pair-room').value.trim()}, 'Key saved');
    $('#pair-discard').onclick = () => {
      if (confirm('Throw away the captured key? Getting another one means running the procedure on the remote again.'))
        act('/api/pair/discard', {}, 'Discarded');
    };
  }
}

function renderObserved() {
  const host = $('#observed');
  // Anything already in the devices table is filtered out firmware-side, so
  // everything here is genuinely unadopted.
  const html = !S.observed.length
    ? '<p class="hint" style="margin:0">Nothing heard yet. Operate a remote &mdash; ' +
      'it addresses each of its devices in turn, and they appear here.</p>'
    : S.observed
    .slice()
    .sort((a, b) => a.agoS - b.agoS)
    .map(o => `
      <div class="dev">
        <div class="dev-top">
          <div class="dev-name">
            <b class="mono">${o.node}</b>
            <small>heard ${since(o.agoS)} &middot; ${o.frames} frame${o.frames === 1 ? '' : 's'}
              &middot; ${o.ed} dBm &middot; last cmd 0x${o.cmd}</small>
          </div>
          <button class="btn small primary" data-add="${o.node}">Add</button>
        </div>
      </div>`).join('');

  if (!setHtml('observed', host, html)) return;

  $$('[data-add]', host).forEach(b => {
    b.onclick = () => {
      openEditor = b.dataset.add;
      act('/api/device', {node: b.dataset.add}, 'Added').then(renderDevices);
    };
  });
}

function slotOptions(selected) {
  let out = `<option value="-1"${selected < 0 ? ' selected' : ''}>&mdash; no key &mdash;</option>`;
  for (const s of S.slots) {
    if (!s.used) continue;
    const label = (s.name || 'slot ' + s.slot) + ' (key#' + s.fp + ')';
    out += `<option value="${s.slot}"${s.slot === selected ? ' selected' : ''}>${esc(label)}</option>`;
  }
  return out;
}

const TYPES = [[0,'Unknown'],[4,'Skylight'],[10,'Blind'],[2,'Roller shutter'],
               [1,'Venetian blind'],[3,'Awning'],[11,'Screen'],[6,'Light']];

function deviceEditor(d) {
  return `
    <div class="editor">
      <label class="field"><span>Name</span>
        <input type="text" data-f="name" value="${esc(d.name)}"
               placeholder="${esc(d.label)}" maxlength="23"></label>
      <label class="field"><span>Room &mdash; leave empty to follow the key's room</span>
        <input type="text" data-f="group" value="${esc(d.groupOwn)}"
               placeholder="${esc(d.group)}" maxlength="19"></label>
      <div class="two">
        <label class="field"><span>Type</span>
          <select data-f="type">${TYPES.map(([v, n]) =>
            `<option value="${v}"${v === d.type ? ' selected' : ''}>${n}</option>`).join('')}</select></label>
        <label class="field"><span>Key</span>
          <select data-f="slot">${slotOptions(d.slot)}</select></label>
      </div>
      <div class="row wrap">
        <button class="btn small" data-act="info">Ask what it is</button>
        <button class="btn small danger" data-act="forget">Forget</button>
        <div style="flex:1"></div>
        <button class="btn small primary" data-act="save">Save</button>
      </div>
    </div>`;
}

function renderDevices() {
  renderPairLive();
  renderObserved();

  const host = $('#devlist');
  if (!S.devices.length) {
    host.innerHTML = '<p class="hint" style="margin:0">None yet.</p>';
    return;
  }
  host.innerHTML = S.devices.map(d => `
    <div class="dev" data-node="${d.node}">
      <div class="dev-top">
        <div class="dev-icon">${iconFor(d.type)}</div>
        <div class="dev-name">
          <b>${esc(d.label)}</b>
          <small><span class="mono">${d.node}</span> &middot; ${esc(d.group)}${
            d.hasKey ? '' : ' &middot; <span style="color:var(--warn)">no key</span>'}</small>
        </div>
        <button class="btn small" data-act="edit">Edit</button>
      </div>
      ${openEditor === d.node ? deviceEditor(d) : ''}
    </div>`).join('');

  $$('.dev', host).forEach(row => {
    const node = row.dataset.node;
    const d = S.devices.find(x => x.node === node);
    row.onclick = async ev => {
      const btn = ev.target.closest('button');
      if (!btn) return;
      const a = btn.dataset.act;
      if (a === 'edit') {
        openEditor = openEditor === node ? null : node;
        renderDevices();
      } else if (a === 'info') {
        act('/api/command', {node, action: 'info'}, 'Asked — watch the console');
      } else if (a === 'forget') {
        if (confirm('Forget ' + d.label + '? The key itself stays in its slot.'))
          act('/api/device/forget', {node}, 'Forgotten').then(() => {
            openEditor = null; renderDevices();
          });
      } else if (a === 'save') {
        const f = sel => $(`[data-f="${sel}"]`, row);
        await act('/api/device', {
          node,
          name:  f('name').value.trim(),
          group: f('group').value.trim(),
          type:  f('type').value,
          slot:  f('slot').value
        }, 'Saved');
        openEditor = null;
        renderDevices();
      }
    };
  });
}

/* ---------------------------------------------------- settings (tab three) */

/* What the bridge is doing, as against what it is configured to do. Worth its
 * own block because "enabled" and "connected" are different questions, and the
 * gap between them -- a wrong password, a broker that is not there -- is the
 * only thing anyone opens this card to find out. */
function mqttStatusHtml() {
  const m = S.mqtt;
  if (!m) return '';
  if (m.state === 'disabled')
    return `<p class="hint">Not publishing. Fill in a broker below and tick
      <b>Publish to a broker</b>.</p>`;

  const cls = m.up ? 'ok' : (m.state === 'failed' ? 'bad' : 'warn');
  let html = `<div class="kv"><span>Bridge</span>
      <b class="${cls}">${esc(m.state)}</b></div>
    <div class="kv"><span>Broker</span><b class="mono">${esc(m.broker)}</b></div>`;
  if (m.up) {
    html += `<div class="kv"><span>Connected</span><b>${ago(m.sinceS)}</b></div>
      <div class="kv"><span>Published</span><b>${m.published}</b></div>
      <div class="kv"><span>Commands in</span><b>${m.received}${
        m.rejected ? ` <span class="bad">(${m.rejected} unreadable)</span>` : ''
      }</b></div>`;
  }
  // A connection count above one means it has been dropping and re-establishing,
  // which looks identical to "working" on every other line here.
  if (m.connects > 1) html += `<div class="kv"><span>Connections</span><b>${m.connects}</b></div>`;
  if (m.error) html += `<p class="hint bad">${esc(m.error)}</p>`;
  return html;
}

/* Refreshed on the poll while the rest of the card is not: the broker fields
 * are a form, and rebuilding it under someone mid-way through typing a password
 * is the bug the Devices tab already has a comment about. */
function renderMqttStatus() {
  const host = $('#mqtt-status');
  if (host) setHtml('mqtt', host, mqttStatusHtml());
}

function renderSettings() {
  const s = S.settings, n = S.net;
  $('#settings').innerHTML = `
    <div class="card">
      <h2>Network</h2>
      <div class="kv"><span>Wi-Fi</span><b>${esc(n.ssid) || '&mdash;'}</b></div>
      <div class="kv"><span>Address</span><b class="mono">${esc(n.ip)}</b></div>
      <div class="kv"><span>Signal</span><b>${n.rssi} dBm</b></div>
      <div class="kv"><span>Uptime</span><b>${ago(n.uptimeS)}</b></div>
      <div class="kv"><span>Free memory</span><b>${(n.heap / 1024).toFixed(0)} kB</b></div>
      <p class="hint" style="margin:12px 0 8px">
        Credentials are held by the setup portal, not here. Forgetting them
        reboots the rig into its <b>velux-rig-setup</b> access point.</p>
      <button class="btn danger" id="wifi-forget">Forget Wi-Fi</button>
    </div>

    <div class="card">
      <h2>MQTT</h2>
      <div id="mqtt-status">${mqttStatusHtml()}</div>
      <label class="check"><input type="checkbox" data-s="mqttEnabled"
        ${s.mqttEnabled ? 'checked' : ''}> Publish to a broker</label>
      <div class="two">
        <label class="field"><span>Broker</span>
          <input type="text" data-s="mqttHost" value="${esc(s.mqttHost)}" placeholder="192.168.1.40"></label>
        <label class="field" style="max-width:110px"><span>Port</span>
          <input type="number" data-s="mqttPort" value="${s.mqttPort}"></label>
      </div>
      <div class="two">
        <label class="field"><span>User</span>
          <input type="text" data-s="mqttUser" value="${esc(s.mqttUser)}"></label>
        <label class="field"><span>Password</span>
          <input type="password" data-s="mqttPass"
                 placeholder="${s.mqttPassSet ? 'unchanged' : 'none'}"></label>
      </div>
      <label class="field"><span>Topic prefix</span>
        <input type="text" data-s="mqttBase" value="${esc(s.mqttBase)}" placeholder="velux"></label>
      <p class="hint">
        Every device publishes to
        <span class="mono">${esc(s.mqttBase || 'velux')}/device/&lt;address&gt;/state</span>
        and takes commands on <span class="mono">&hellip;/set</span>. Rooms work
        the same way, and
        <span class="mono">${esc(s.mqttBase || 'velux')}/discovery</span> lists
        the whole installation. Outcomes &mdash; including a blind that refused
        to move &mdash; go to
        <span class="mono">${esc(s.mqttBase || 'velux')}/event</span>.</p>
      <details class="fold">
        <summary>Advanced</summary>
        <label class="field"><span>Client ID &mdash; blank derives one from the MAC</span>
          <input type="text" data-s="mqttClientId" value="${esc(s.mqttClientId)}"
                 placeholder="${esc(S.mqtt.clientId) || 'velux-xxxxxx'}"></label>
        <div class="two">
          <label class="field"><span>QoS</span>
            <select data-s="mqttQos">
              <option value="0"${s.mqttQos === 0 ? ' selected' : ''}>0 &mdash; at most once</option>
              <option value="1"${s.mqttQos === 1 ? ' selected' : ''}>1 &mdash; at least once</option>
              <option value="2"${s.mqttQos === 2 ? ' selected' : ''}>2 &mdash; exactly once</option>
            </select></label>
          <label class="field"><span>Re-announce every (s)</span>
            <input type="number" data-s="mqttDiscoveryS" value="${s.mqttDiscoveryS}"
                   min="0" max="65535"></label>
        </div>
        <label class="check"><input type="checkbox" data-s="mqttRetain"
          ${s.mqttRetain ? 'checked' : ''}> Retain state topics</label>
        <p class="hint">
          Retaining is what lets something that subscribes later see the current
          position instead of waiting for the next change. Events are never
          retained either way: a window opening is news once.</p>
      </details>
      <button class="btn primary" data-save="settings">Save MQTT settings</button>
    </div>

    <div class="card">
      <h2>Keys</h2>
      <p class="hint">
        One slot per remote. Each key was copied from a remote during pairing
        and cannot be read back &mdash; the fingerprint is just a label.</p>
      <div id="slots"></div>
    </div>

    <div class="card">
      <h2>Radio</h2>
      <label class="field"><span>Our address &mdash; what this rig transmits as</span>
        <input type="text" data-s="ourAddress" value="${esc(s.ourAddress)}"
               maxlength="6" class="mono"></label>
      <p class="hint">Changing it means the installation no longer recognises
        the rig, and every remote has to be paired again.</p>
      <div class="two">
        <label class="field"><span>Command channel</span>
          <select data-s="commandChannel">
            <option value="0"${s.commandChannel === 0 ? ' selected' : ''}>All three</option>
            <option value="15"${s.commandChannel === 15 ? ' selected' : ''}>15 &mdash; 2425 MHz</option>
            <option value="20"${s.commandChannel === 20 ? ' selected' : ''}>20 &mdash; 2450 MHz</option>
            <option value="25"${s.commandChannel === 25 ? ' selected' : ''}>25 &mdash; 2475 MHz</option>
          </select></label>
        <label class="field"><span>Wake-up train (ms)</span>
          <input type="number" data-s="trainMs" value="${s.trainMs}" min="0" max="2000"></label>
      </div>
      <label class="field"><span>Read the position back after a move (s, 0 = never)</span>
        <input type="number" data-s="statusAfterMoveS" value="${s.statusAfterMoveS}" min="0" max="600"></label>
      <label class="field"><span>Refresh every device's position every (s, 0 = never)</span>
        <input type="number" data-s="pollIntervalS" value="${s.pollIntervalS}" min="0" max="65535"></label>
      <p class="hint">
        These devices only report a position when asked, and the rig forgets
        what it knew when it restarts. Without this it shows
        <b>unknown</b> for everything until each one is operated. The reads are
        spread out, one device at a time.</p>
      <label class="check"><input type="checkbox" data-s="silentDefault"
        ${s.silentDefault ? 'checked' : ''}> Move quietly by default</label>
      <p class="hint">
        The slow travel profile, for windows that get driven at night. Applies
        to anything that does not ask for one specifically &mdash; the console
        takes <span class="mono">silent</span> or <span class="mono">loud</span>
        per command, and MQTT takes <span class="mono">"silent"</span>.</p>
      <button class="btn primary" data-save="settings">Save radio settings</button>
    </div>

    <div class="card">
      <h2>Access</h2>
      <p class="hint">
        A password makes every page and every command ask for one, with the user
        name <b>velux</b>. Empty means open to anyone on this network, which is
        the default.</p>
      <label class="field"><span>Password</span>
        <input type="password" data-s="uiPass"
               placeholder="${s.uiPassSet ? 'unchanged' : 'no password'}"></label>
      <div class="row">
        ${s.uiPassSet ? '<button class="btn danger" id="ui-pass-clear">Remove password</button>' : ''}
        <div style="flex:1"></div>
        <button class="btn primary" data-save="settings">Save</button>
      </div>
    </div>

    <div class="card">
      <h2>Rig</h2>
      <button class="btn danger" id="reboot">Reboot</button>
    </div>`;

  renderSlots();

  $$('[data-save="settings"]').forEach(b => b.onclick = saveSettings);
  $('#reboot').onclick = () => {
    if (confirm('Reboot the rig? It will be back in about ten seconds.'))
      post('/api/reboot', {}).then(() => toast('Rebooting'));
  };
  $('#wifi-forget').onclick = () => {
    if (confirm('Forget the Wi-Fi network? You will have to join the rig\'s own access point to set it up again.'))
      post('/api/wifi/forget', {confirm: 'yes'}).then(() => toast('Restarting into setup'));
  };
  const clr = $('#ui-pass-clear');
  if (clr) clr.onclick = () => act('/api/settings', {uiPassClear: '1'}, 'Password removed')
    .then(renderSettings);
}

function saveSettings() {
  const body = {};
  $$('[data-s]').forEach(el => {
    const k = el.dataset.s;
    if (el.type === 'checkbox') body[k] = el.checked ? '1' : '0';
    else body[k] = el.value.trim();
  });
  // An untouched password box is empty, which must not be read as "clear it";
  // the firmware ignores an empty one, and clearing has its own button.
  if (!body.mqttPass) delete body.mqttPass;
  if (!body.uiPass) delete body.uiPass;
  act('/api/settings', body, 'Saved').then(() => {
    if (tab === 'settings') renderSettings();
  });
}

function renderSlots() {
  const host = $('#slots');
  const used = S.slots.filter(s => s.used);
  if (!used.length) {
    host.innerHTML = '<p class="hint" style="margin:0">No keys stored. Pair a remote from the Devices tab.</p>';
    return;
  }
  host.innerHTML = used.map(s => `
    <div class="dev" data-slot="${s.slot}">
      <div class="dev-top">
        <div class="dev-name">
          <b>${esc(s.name || 'Slot ' + s.slot)}</b>
          <small><span class="mono">key#${s.fp}</span> &middot; from remote
            <span class="mono">${s.remote}</span> &middot;
            ${s.devices.length} device${s.devices.length === 1 ? '' : 's'}</small>
        </div>
        <button class="btn small" data-act="rename">Rename</button>
        <button class="btn small danger" data-act="erase">Erase</button>
      </div>
    </div>`).join('');

  $$('.dev', host).forEach(row => {
    const slot = row.dataset.slot;
    const s = S.slots.find(x => String(x.slot) === slot);
    row.onclick = ev => {
      const btn = ev.target.closest('button');
      if (!btn) return;
      if (btn.dataset.act === 'rename') {
        const name = prompt('Room this key covers', s.name || '');
        if (name !== null) act('/api/slot', {slot, name: name.trim()}, 'Renamed').then(renderSettings);
      } else {
        if (confirm('Erase ' + (s.name || 'slot ' + slot) + '?\n\nThis destroys the key. ' +
                    'Getting it back means running the pairing procedure on that remote again.'))
          act('/api/slot/forget', {slot, confirm: 'yes'}, 'Erased').then(renderSettings);
      }
    };
  });
}

/* ----------------------------------------------------- console (tab four) */

async function pollLog() {
  if (tab !== 'console') return;
  try {
    const url = logCursor === null ? '/api/console' : '/api/console?from=' + logCursor;
    const res = await fetch(url);
    if (!res.ok) return;
    const next = res.headers.get('X-Log-Next');
    const lost = parseInt(res.headers.get('X-Log-Lost') || '0', 10);
    const text = await res.text();
    if (next !== null) logCursor = parseInt(next, 10);

    const el = $('#log');
    if (lost) el.textContent += `\n[${lost} characters lost -- the rig produced output faster than this page read it]\n`;
    if (text) el.textContent += text;

    // Trim, or a night of frame tracing turns the page into a memory leak.
    if (el.textContent.length > 120000) el.textContent = el.textContent.slice(-90000);
    if (logFollow) el.scrollTop = el.scrollHeight;
  } catch (e) { /* the rig is busy or rebooting; the next tick will catch up */ }
}

function setupConsole() {
  const quick = ['help', 'net', 'rfstat', 'keys list', 'bootstat', 'sky status'];
  $('#quick').innerHTML = quick.map(c =>
    `<button class="btn" data-cmd="${esc(c)}">${esc(c)}</button>`).join('');
  $$('#quick [data-cmd]').forEach(b => b.onclick = () => run(b.dataset.cmd));

  const submit = ev => {
    if (ev) ev.preventDefault();
    const input = $('#console-input');
    const line = input.value.trim();
    if (!line) return;
    run(line);
    input.value = '';
  };
  $('#console-form').onsubmit = submit;
  // Explicit as well as implicit: a form with one field submits on Enter in a
  // browser, but a phone keyboard's "go" key and an on-screen keyboard that
  // sends a keydown without submitting both turn up in practice.
  $('#console-input').addEventListener('keydown', ev => {
    if (ev.key === 'Enter') submit(ev);
  });

  // Scrolling up pauses the follow, so reading back through a trace is not
  // yanked to the bottom every second.
  $('#log').addEventListener('scroll', () => {
    const el = $('#log');
    logFollow = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
  });
}

function run(line) {
  post('/api/console', {line})
    .then(() => { logFollow = true; return pollLog(); })
    .catch(e => toast(e.message, true));
}

/* -------------------------------------------------------------- polling */

async function poll() {
  try {
    const res = await fetch('/api/state');
    if (!res.ok) throw new Error('HTTP ' + res.status);
    S = await res.json();
    $('#conn').textContent = S.radio.busy
      ? 'sending'
      : (S.radio.pending ? S.radio.pending + ' queued' : 'connected');
    $('#conn').className = 'pill ' + (S.radio.busy || S.radio.pending ? 'busy' : 'ok');

    if (tab === 'rooms') refreshRooms();
    // The Devices tab's live half -- pairing and what is on air -- refreshes
    // with the poll; its forms do not, so typing in one survives.
    if (tab === 'devices') { renderPairLive(); renderObserved(); }
    // Same split on Settings: the bridge's connection state is live, the broker
    // fields are not.
    if (tab === 'settings') renderMqttStatus();
  } catch (e) {
    $('#conn').textContent = 'offline';
    $('#conn').className = 'pill bad';
  }
}

function schedule() {
  clearInterval(pollTimer);
  // The console tab polls its own log every second and does not need the state
  // document at all, so the two cadences are kept apart.
  pollTimer = setInterval(tab === 'console' ? pollLog : poll, tab === 'console' ? 1000 : 3000);
}

/* ------------------------------------------------------------------ tabs */

function show(name) {
  tab = name;
  $$('#tabs button').forEach(b => b.classList.toggle('on', b.dataset.tab === name));
  $$('main .tab').forEach(s => s.hidden = s.id !== 'tab-' + name);
  // replaceState, not location.hash: several containers on the page have ids
  // that match the tab names, so assigning the hash made the browser scroll to
  // one of them and every tab opened part-way down. This also keeps the back
  // button meaning "leave", rather than walking back through tab switches.
  history.replaceState(null, '', '#' + name);

  if (name === 'rooms' && S) renderRooms();
  if (name === 'devices' && S) renderDevices();
  if (name === 'settings' && S) renderSettings();
  if (name === 'console') { logFollow = true; pollLog(); }
  schedule();
  window.scrollTo(0, 0);
}

/* --------------------------------------------------------------- actions */

// One listener for every control on the Rooms tab: the cards are rebuilt on
// each poll, and per-button handlers would have to be rebound with them.
$('#rooms').addEventListener('click', ev => {
  const btn = ev.target.closest('button');
  if (!btn) return;

  if (btn.dataset.group) {
    const devs = S.devices.filter(d => (d.group || 'Unassigned') === btn.dataset.group && d.hasKey);
    if (!devs.length) return toast('Nothing in that room has a key bound', true);
    Promise.all(devs.map(d => post('/api/command', {node: d.node, action: btn.dataset.act})))
      .then(() => toast(devs.length + ' queued'))
      .then(poll)
      .catch(e => toast(e.message, true));
    return;
  }

  const node = btn.closest('.dev').dataset.node;
  const a = btn.dataset.act;
  if (a === 'status') act('/api/command', {node, action: 'status'}, 'Reading position');
  else if (a) act('/api/command', {node, action: a}, a[0].toUpperCase() + a.slice(1) + ' queued');
});

$('#rooms').addEventListener('input', ev => {
  const el = ev.target;
  if (el.dataset.act !== 'slider') return;
  draggingSlider = el.closest('.dev').dataset.node;
  el.parentElement.querySelector('.pct').textContent = el.value + '%';
});

// 'change' rather than 'input': a range control fires it once, on release, so
// dragging from closed to half open sends one command rather than twenty.
$('#rooms').addEventListener('change', ev => {
  const el = ev.target;
  if (el.dataset.act !== 'slider') return;
  const node = el.closest('.dev').dataset.node;
  draggingSlider = null;
  act('/api/command', {node, action: 'pos', position: el.value}, el.value + '% queued');
});

$$('#tabs button').forEach(b => b.onclick = () => show(b.dataset.tab));

/* ------------------------------------------------------------------ boot */

setupConsole();
poll().then(() => show(location.hash.slice(1) || 'rooms'));
