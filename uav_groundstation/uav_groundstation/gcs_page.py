"""gcs_page — the operator's page, as one string.

Seven tabs: Nodes, Telemetry, Camera + Map, Map, Camera, Logs, System. No build
step, no framework, no
CDN — the Jetson serves this to a laptop over field WiFi, and a page that needs
to fetch anything else is a page that does not load at a flight line.

THE PAGE EXPLAINS RULES; IT DOES NOT HOLD THEM. Every control here is re-checked
server-side on arrival (see gcs_server), because anyone can edit JavaScript in a
browser or curl the endpoint. A greyed-out button is a courtesy to the operator,
never a security boundary.

THE MAP DRAWS THE SAME `geofence` PARAM THE UPLOADER SENDS. Not a copy, not a
re-derivation — the snapshot carries the polygon telemetry_bridge would upload,
so what the operator sees is what the autopilot was told. Two sources here would
drift silently and the drift would only show as an unexplained fence breach.

Logs are read INCREMENTALLY: the page sends the newest sequence number it holds
and gets only what is new. Resending the whole ring at the poll rate would cost
more than every other tab combined.

LIGHT AND DARK ARE THE BROWSER'S CHOICE, kept in its own localStorage. The theme
is not aircraft state: two laptops can differ and nothing on the Jetson knows.
Every colour — the map canvas included — comes from the CSS variables, so the
toggle reaches every tab. A colour hard-coded anywhere else is one it cannot.

gcs_server has no ROS imports, so this page can be served on a laptop against
invented state (render() plus a GcsServer handed a fake snapshot function).
bench_gcs does exactly that.
"""

PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>rx26_uav ground station</title>
<style>
/* Dark is the original palette. Light is built for a laptop in direct sun:
   near-black text on white, darker "dim" text and heavier lines than a typical
   light theme, because washed-out grey is exactly what disappears in glare. */
:root{--bg:#11151a;--panel:#1a1f27;--line:#2b3240;--fg:#dfe6ef;--dim:#8b97a8;
      --ok:#4ec27b;--warn:#e0a33e;--bad:#e2564a;--accent:#57a6ff;
      --on-accent:#08121f;--btn:#232a34;--sunk:#0c1015;--dot-off:#454e5c;
      --grid:rgba(223,230,239,.07);--grid-major:rgba(223,230,239,.17);
      --ring:rgba(223,230,239,.35);--fence-fill:rgba(87,166,255,.07);
      --trail:rgba(78,194,123,.55);--veh-ring:rgba(255,255,255,.18);
      --buoy-off:#000;--buoy-solid:#fff;
      --foot:rgba(224,163,62,.9);--foot-fill:rgba(224,163,62,.08);
      --b-red:#e2564a;--b-green:#4ec27b;--b-blue:#57a6ff;
      --ok-bg:rgba(78,194,123,.13);--warn-bg:rgba(224,163,62,.14);
      --bad-bg:rgba(226,86,74,.16);--accent-bg:rgba(87,166,255,.13);
      color-scheme:dark}
:root[data-theme=light]{--bg:#fff;--panel:#eef1f5;--line:#a3adbb;--fg:#0a0e13;
      --dim:#3b4655;--ok:#17743a;--warn:#8a5700;--bad:#b3241a;--accent:#0a56bd;
      --on-accent:#fff;--btn:#dde2e9;--sunk:#f7f9fb;--dot-off:#98a2b0;
      --grid:rgba(10,14,19,.10);--grid-major:rgba(10,14,19,.28);
      --ring:rgba(10,14,19,.45);--fence-fill:rgba(10,86,189,.08);
      --trail:rgba(23,116,58,.8);--veh-ring:rgba(10,14,19,.3);
      --buoy-off:#000;--buoy-solid:#0a0e13;
      --foot:rgba(138,87,0,.9);--foot-fill:rgba(138,87,0,.07);
      --b-red:#c42a1d;--b-green:#157d38;--b-blue:#0a5ccf;
      --ok-bg:rgba(23,116,58,.10);--warn-bg:rgba(138,87,0,.11);
      --bad-bg:rgba(179,36,26,.10);--accent-bg:rgba(10,86,189,.09);
      color-scheme:light}
*{box-sizing:border-box}
/* A normal reading face for words, monospace only where columns must line up
   (logs, coordinates, code). Numbers everywhere use tabular figures so a value
   ticking at the poll rate does not jitter sideways. */
body{margin:0;background:var(--bg);color:var(--fg);
     font:15px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
     font-variant-numeric:tabular-nums}
code,.mono,#logs{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
header{position:sticky;top:0;z-index:5;background:var(--panel);
       border-bottom:1px solid var(--line);box-shadow:0 2px 8px rgba(0,0,0,.12)}
.topbar{display:flex;align-items:center;gap:18px;padding:6px 16px;flex-wrap:wrap}
.brand{display:flex;align-items:baseline;gap:8px;margin-right:6px}
.brand b{font-size:19px;letter-spacing:.2px}
.brand span{font-size:12px;color:var(--dim)}
#tabs{display:flex;gap:2px;flex-wrap:wrap}
.tab{padding:9px 14px 7px;border:0;border-bottom:3px solid transparent;
     border-radius:0;cursor:pointer;background:transparent;color:var(--dim);
     font:inherit;font-weight:600}
.tab:hover{color:var(--fg);border-bottom-color:var(--line)}
.tab.on{color:var(--accent);border-bottom-color:var(--accent);background:transparent}
.spacer{flex:1}
#banner{font-size:13px;font-weight:700;padding:3px 12px;border-radius:14px;
        color:var(--ok);background:var(--ok-bg)}
#banner.bad{color:var(--bad);background:var(--bad-bg)}
.iconbtn{background:transparent;border:1px solid var(--line);border-radius:16px;
         padding:4px 12px;color:var(--dim);font-size:13px;font-weight:600}

/* The four numbers wanted at a glance, on every tab. The coloured edge is the
   verdict; the big number is the reading. */
#vitals{display:grid;grid-template-columns:repeat(auto-fit,minmax(215px,1fr));
        gap:10px;padding:2px 16px 10px}
.tile{background:var(--bg);border:1px solid var(--line);border-left:6px solid var(--line);
      border-radius:8px;padding:4px 12px 6px}
.tile .k{font-size:11px;font-weight:700;letter-spacing:.9px;text-transform:uppercase;
         color:var(--dim)}
.tile .big{font-size:24px;font-weight:700;line-height:1.15;display:flex;
           align-items:center;gap:9px}
.tile .sub{font-size:13px;color:var(--dim)}
.tile.ok{border-left-color:var(--ok)}
.tile.warn{border-left-color:var(--warn);background:var(--warn-bg)}
.tile.bad{border-left-color:var(--bad);background:var(--bad-bg)}
.tile.armed{border-left-color:var(--accent);background:var(--accent-bg)}
.tile.ok .big{color:var(--ok)} .tile.warn .big{color:var(--warn)}
.tile.bad .big{color:var(--bad)} .tile.armed .big{color:var(--accent)}
.gauge{height:6px;border-radius:3px;background:var(--line);margin-top:5px;overflow:hidden}
.gauge i{display:block;height:100%;background:var(--dim)}
.tile.ok .gauge i{background:var(--ok)} .tile.warn .gauge i{background:var(--warn)}
.tile.bad .gauge i{background:var(--bad)}

/* The pre-flight checklist: a dot per check, words only for what is wrong. */
#preflight{display:flex;gap:6px;flex-wrap:wrap;align-items:center;padding:0 16px 10px}
#preflight:empty{display:none}
.pfl{font-size:11px;font-weight:700;letter-spacing:.9px;text-transform:uppercase;
     color:var(--dim);margin-right:4px}
.chip{display:inline-flex;align-items:center;gap:7px;font-size:13px;padding:3px 11px;
      border-radius:14px;border:1px solid var(--line);background:var(--bg);
      max-width:640px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;cursor:help}
.chip.warn{border-color:var(--warn);background:var(--warn-bg);font-weight:600}
.chip.bad{border-color:var(--bad);background:var(--bad-bg);font-weight:700}
.dot{width:10px;height:10px;border-radius:50%;flex:none;display:inline-block;
     background:var(--dot-off)}
.big .dot{width:16px;height:16px}
.dot.ok{background:var(--ok)} .dot.warn{background:var(--warn)} .dot.bad{background:var(--bad)}

main{padding:16px;max-width:1240px;margin:0 auto}
section{display:none} section.on{display:block}
h3.sec{font-size:12px;font-weight:700;letter-spacing:.9px;text-transform:uppercase;
       color:var(--dim);margin:20px 0 8px}
h3.sec:first-child{margin-top:2px}
.cards{display:grid;gap:10px;grid-template-columns:repeat(auto-fill,minmax(180px,1fr))}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:9px 12px}
.card.ok{border-left:5px solid var(--ok)} .card.warn{border-left:5px solid var(--warn)}
.card.bad{border-left:5px solid var(--bad)}
.k{color:var(--dim);font-size:12px;font-weight:600}
.v{font-size:21px;font-weight:600;margin-top:1px;display:flex;align-items:center;gap:8px}
.v.bad{color:var(--bad)} .v.ok{color:var(--ok)} .v.warn{color:var(--warn)}
.grp{margin:0 0 22px}
.grp h2{font-size:12px;color:var(--dim);text-transform:uppercase;letter-spacing:.9px;
        margin:0 0 2px;font-weight:700}
.grp p.why{margin:0 0 8px;color:var(--dim);font-size:13px}
.node{display:flex;align-items:center;gap:14px;background:var(--panel);
      border:1px solid var(--line);border-radius:8px;padding:10px 14px;
      margin-bottom:8px;flex-wrap:wrap}
.nmwrap{flex:1;min-width:240px}
.nm{font-weight:700;font-size:16px}
.nmwrap .note{display:block}
.pill{font-size:12px;font-weight:700;padding:2px 10px;border-radius:10px;min-width:82px;
      text-align:center;background:var(--line);color:var(--dim)}
.pill.up{background:var(--ok-bg);color:var(--ok)}
.pill.restarting{background:var(--warn-bg);color:var(--warn)}
.note{color:var(--dim);font-size:13px}
button{font:inherit;font-size:14px;font-weight:600;padding:6px 14px;border-radius:6px;
       cursor:pointer;border:1px solid var(--line);background:var(--btn);color:var(--fg)}
button:hover{border-color:var(--accent)}
button[disabled]{opacity:.4;cursor:not-allowed}
button.danger{border-color:var(--bad);color:var(--bad)}
button.go{border-color:var(--ok);color:var(--ok)}
button.warnb{border-color:var(--warn);color:var(--warn)}
button.toggle.on{background:var(--accent);border-color:var(--accent);color:var(--on-accent)}
.locked{color:var(--dim);font-size:13px;font-style:italic;max-width:560px}
.badge{font-size:12px;font-weight:700;padding:3px 10px;border-radius:10px;
       border:1px solid var(--line);color:var(--dim);cursor:help}
#toast{position:fixed;right:16px;bottom:16px;background:var(--panel);
       border:1px solid var(--ok);border-left:6px solid var(--ok);color:var(--fg);
       padding:10px 14px;border-radius:8px;max-width:min(560px,86vw);display:none;
       z-index:9;white-space:pre-wrap;box-shadow:0 4px 14px rgba(0,0,0,.25)}
#toast.bad{border-color:var(--bad)}
#mapwrap{position:relative;background:var(--panel);border:1px solid var(--line);
         border-radius:8px;overflow:hidden}
/* Views FILL the screen under the header instead of a fixed size, so nothing
   needs scrolling to be seen. --hdr, --mapbar and --camrec are measured by
   layoutVars() every poll, because the header grows when the checklist has
   something to say and the map bar wraps on a narrow window. */
#map{width:100%;height:max(300px,calc(100vh - var(--hdr,150px) - var(--mapbar,48px) - 66px));
     display:block;cursor:grab}
#camimg{display:block;margin:0 auto;max-width:100%;border:1px solid var(--line);
        border-radius:6px;max-height:max(220px,calc(100vh - var(--hdr,150px) - var(--camrec,48px) - 44px))}
/* Camera + Map: both sections at once, side by side, each fitted to the height. */
main.split{max-width:none;display:grid;gap:14px;align-items:start;
           grid-template-columns:minmax(0,1.2fr) minmax(0,1fr)}
main.split #s-cam{grid-column:1;grid-row:1}
main.split #s-map{grid-column:2;grid-row:1}
/* Housekeeping controls stay on the full Map tab; in flight the split view
   keeps one toolbar row and the height it would have cost. */
main.split details.about,main.split #mapexp,main.split .wide-only{display:none}
@media (max-width:900px){main.split{grid-template-columns:1fr}
  main.split #s-map{grid-column:1;grid-row:2}}
#mapbar{display:flex;gap:6px;align-items:center;padding:8px 10px;
        border-bottom:1px solid var(--line);flex-wrap:wrap}
#mapbar .sep{width:1px;height:26px;background:var(--line);margin:0 4px}
#mapinfo{padding:6px 12px;font-size:13px;color:var(--dim);border-top:1px solid var(--line)}
#map.measuring{cursor:crosshair}
#logs{background:var(--sunk);border:1px solid var(--line);border-radius:8px;
      padding:8px;height:min(60vh,560px);overflow:auto;font-size:12.5px}
.lg{display:flex;gap:8px;padding:1px 0;white-space:pre-wrap;word-break:break-word}
.lg .t{color:var(--dim);flex:none} .lg .n{color:var(--accent);flex:none}
.lg.WARN .m{color:var(--warn)} .lg.ERROR .m,.lg.FATAL .m{color:var(--bad)}
.lg.DEBUG{opacity:.62}
.bar{display:flex;gap:8px;align-items:center;margin-bottom:10px;flex-wrap:wrap}
select,input{font:inherit;font-size:14px;background:var(--btn);color:var(--fg);
             border:1px solid var(--line);border-radius:6px;padding:5px 8px}
.hint{color:var(--dim);font-size:13px;margin:8px 0 0}
.callout{margin:12px 0 0;padding:8px 12px;border-radius:8px;border-left:5px solid var(--warn);
         background:var(--warn-bg);font-size:14px}
.callout:empty{display:none}
details.about{margin-top:16px;color:var(--dim);font-size:13px}
details.about summary{cursor:pointer;font-weight:600}
details.about p{margin:6px 0 0;max-width:920px}
#buoylist table{margin-top:10px;border-collapse:collapse;font-size:14px}
#buoylist td{padding:4px 18px 4px 0;white-space:nowrap}
</style>
<script>
/* Applied before the body paints, so a reload in sunlight does not flash dark.
   No stored choice follows the operating system's light/dark setting. */
(function(){var t;try{t=localStorage.getItem('rx26-theme')}catch(e){}
  if(t!=='light'&&t!=='dark')t=(window.matchMedia&&
    matchMedia('(prefers-color-scheme: light)').matches)?'light':'dark';
  document.documentElement.setAttribute('data-theme',t)})();
</script></head><body>
<header>
  <div class="topbar">
    <div class="brand"><b>Ekko</b><span>ground station</span></div>
    <div id="tabs"></div>
    <div class="spacer"></div>
    <div id="banner">connecting…</div>
    <button id="themeb" class="iconbtn" onclick="toggleTheme()">theme</button>
  </div>
  <div id="vitals">
    <div id="battery" class="tile"></div>
    <div id="flighttime" class="tile"></div>
    <div id="gpstile" class="tile"></div>
    <div id="alttile" class="tile"></div>
  </div>
  <div id="preflight"></div>
</header>
<main>
  <section id="s-nodes"><div id="nodes"></div>
    <details class="about"><summary>About this tab</summary>
    <p>Presence comes from the ROS graph <b>and</b> /proc, so a node started by
    systemd or by hand in another terminal shows here too. Nodes with a systemd
    unit (camera_node, ocs_client) come straight back when killed, so they get
    <b>restart</b> rather than stop.</p></details></section>
  <section id="s-tel"><div id="tel"></div>
    <div class="callout" id="telhint"></div></section>
  <section id="s-map">
    <div id="mapwrap">
      <div id="mapbar">
        <button onclick="zoom(1.4)" title="zoom in">+</button>
        <button onclick="zoom(0.71)" title="zoom out">−</button>
        <button id="followb" class="toggle on" onclick="toggleFollow()"
          title="keep the aircraft in the middle">Follow</button>
        <span class="sep"></span>
        <button id="measureb" class="toggle" onclick="toggleMeasure()"
          title="click two points, or two buoys, for the distance">Measure</button>
        <button id="soundb" class="toggle" onclick="toggleSound()"
          title="a short tone on this laptop when a buoy's state is decided">Lock beep</button>
        <span class="sep wide-only"></span>
        <button class="wide-only" onclick="clearTrail()">Clear trail</button>
        <button class="wide-only" onclick="clearBuoys()">Clear buoys</button>
        <span class="spacer"></span>
        <span id="mapexp"></span>
      </div>
      <canvas id="map"></canvas>
      <div id="mapinfo"></div>
    </div>
    <div id="buoylist"></div>
    <details class="about"><summary>About this tab</summary>
    <p>The polygon is the <b>same <code>geofence</code> parameter
    telemetry_bridge uploads</b> — what you see is what the autopilot was told.
    Drag to pan; scroll or +/− to zoom. The grid is fixed to the ground and
    re-spaces itself as you zoom — the corner says the spacing. The autopilot
    enforces the fence; this is a readout.</p>
    <p><b>Measure</b>: click two points for the distance between
    them. A click on a buoy snaps to its mapped centre, so buoy-to-buoy spacing
    reads straight off the map. <b>The dashed amber rectangle</b> is what the
    camera sees right now (its thick edge is the top of the image), drawn only
    while the gimbal is at nadir. <b>Lock beep</b> plays a short tone on this
    laptop each time a buoy's state is decided; browsers only allow sound after
    a click on the page, so after a reload click anywhere once.</p>
    <p>Buoys come from <code>buoy_mapper</code>. A buoy reads
    <b>UNKNOWN</b> until it has been watched in full view for 4 s — one frame
    cannot tell flashing from solid or off. The dashed ring is its
    <b>spread</b>: a small ring means its sightings agree; a large one means the
    position is not to be trusted. Downloads are the map as it is right now;
    the same files are also written on the Jetson on every disarm.</p>
    </details>
  </section>
  <section id="s-logs">
    <div class="bar">
      <select id="lvl" onchange="repaintLogs()">
        <option value="10">DEBUG+</option><option value="20" selected>INFO+</option>
        <option value="30">WARN+</option><option value="40">ERROR+</option>
      </select>
      <select id="lnode" onchange="repaintLogs()"><option value="">all nodes</option></select>
      <button onclick="clearLogs()">clear</button>
      <span class="note" id="loginfo"></span>
    </div>
    <div id="logs"></div>
    <details class="about"><summary>About this tab</summary>
    <p>From <code>/rosout</code>, not journalctl — we are inside a container and
    the host journal is on the other side of that boundary. It misses output
    written straight to stdout, and anything printed before a node finished
    constructing, which is exactly when a bad parameter kills one.</p></details>
  </section>
  <section id="s-cam"><div id="camrec"></div><div id="cam"></div>
    <details class="about"><summary>About this tab</summary>
    <p>The video is served by <code>camera_node</code> on its own port, not
    proxied through this one — megabytes of MJPEG through the ground station's
    snapshot path would make a stalled camera look like a stalled ground
    station. The tab waits for that port to actually accept a connection before
    pointing at it, because a process appears in the table seconds before its
    server binds.</p></details></section>
  <section id="s-sys"><div class="cards" id="sys"></div><div id="power"></div></section>
</main>
<div id="toast"></div>
<script>
var POLL=__POLL_MS__, S={}, tab='nodes', logs=[], logSeq=0, dropped=0;
var TABS=[['nodes','Nodes'],['tel','Telemetry'],['fly','Camera + Map'],['map','Map'],['cam','Camera'],['logs','Logs'],['sys','System']];
var SECTIONS=['nodes','tel','map','cam','logs','sys'];
/* "fly" has no section of its own: it shows the camera and map sections side by
   side, which is what two browser windows were doing at the park. */
function mapVisible(){return tab==='map'||tab==='fly'}
function camVisible(){return tab==='cam'||tab==='fly'}
function el(i){return document.getElementById(i)}
function esc(s){return String(s==null?'':s).replace(/[&<>"]/g,function(c){
  return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function fmt(v,n){return (v===null||v===undefined||(typeof v==='number'&&isNaN(v)))
  ?'—':Number(v).toFixed(n)}
function toast(m,bad){var t=el('toast');t.textContent=m;t.className=bad?'bad':'';
  t.style.display='block';clearTimeout(t._h);t._h=setTimeout(function(){
  t.style.display='none'},bad?7000:3200)}
/* quiet: for background polls. The logs poll used to go through the toasting
   path once a second, which pinned an "ok" box over the bottom-right corner of
   every tab and turned any WiFi blip into a "request failed" pop-up -- the
   banner already reports an unreachable ground station. A poll that is REFUSED
   still toasts, because that is news. */
function post(p,b,quiet){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify(b||{})}).then(function(r){return r.json()}).then(function(j){
  if(!quiet||!j.ok)toast(j.message||(j.ok?'ok':'failed'),!j.ok);return j}).catch(function(e){
  if(!quiet)toast('request failed: '+e,true)})}
function show(t){tab=t;
  document.querySelector('main').classList.toggle('split',t==='fly');
  TABS.forEach(function(p){el('tb-'+p[0]).className='tab'+(p[0]===t?' on':'')});
  SECTIONS.forEach(function(k){
    el('s-'+k).className=(k===t||(t==='fly'&&(k==='map'||k==='cam')))?'on':''});
  try{localStorage.setItem('rx26-tab',t)}catch(e){}
  layoutVars();
  if(mapVisible()){resize();draw()} if(t==='logs')repaintLogs(); render()}
/* Heights the fitted views subtract. Measured, not guessed: the header changes
   height with the checklist, and toolbars wrap on narrow windows. A change
   re-sizes the map canvas, whose pixel size is set from its CSS box. */
var lastLayout='';
function layoutVars(){
  var h=document.querySelector('header').offsetHeight,
      mb=(el('mapbar')||{}).offsetHeight||0,cr=(el('camrec')||{}).offsetHeight||0,
      key=h+'/'+mb+'/'+cr;
  if(key===lastLayout)return;
  lastLayout=key;
  var st=document.documentElement.style;
  st.setProperty('--hdr',h+'px');
  if(mb)st.setProperty('--mapbar',mb+'px');
  if(cr)st.setProperty('--camrec',cr+'px');
  if(mapVisible()){resize();draw()}}
(function(){el('tabs').innerHTML=TABS.map(function(p){
  return '<button class="tab" id="tb-'+p[0]+'" onclick="show(\''+p[0]+'\')">'+p[1]+'</button>'
  }).join('')})();

/* ---- theme ----
   The canvas cannot read CSS variables by itself, so PAL is a copy of them,
   re-read whenever the theme changes. Use PAL for every canvas colour. */
var PAL={},BPAL={RED:'bRed',GREEN:'bGreen',BLUE:'bBlue'};
var BVAR={RED:'var(--b-red)',GREEN:'var(--b-green)',BLUE:'var(--b-blue)'};
function readPalette(){
  var cs=getComputedStyle(document.documentElement);
  ['fg','dim','ok','bad','accent','panel','grid','grid-major','ring',
   'fence-fill','trail','veh-ring','buoy-off','buoy-solid','b-red','b-green',
   'b-blue','foot','foot-fill'].forEach(function(k){
    PAL[k.replace(/-(\w)/g,function(_,c){return c.toUpperCase()})]=
      cs.getPropertyValue('--'+k).trim()})}
function setTheme(t){
  document.documentElement.setAttribute('data-theme',t);
  try{localStorage.setItem('rx26-theme',t)}catch(e){}
  var b=el('themeb');b.textContent=t==='light'?'\u263e dark':'\u2600 light';
  b.title='switch to the '+(t==='light'?'dark':'light')+' theme';
  readPalette();if(mapVisible())draw()}
function toggleTheme(){
  setTheme(document.documentElement.getAttribute('data-theme')==='light'?'dark':'light')}
setTheme(document.documentElement.getAttribute('data-theme')||'dark');

/* ---- nodes ---- */
/* Every render below runs at the poll rate. Assigning innerHTML destroys and
   rebuilds the whole subtree, and a control rebuilt between mousedown and
   mouseup never fires its click. That is not theoretical: it is how the capture
   button and the shutdown hostname field both became unusable, five rebuilds a
   second each. paint() writes only when the markup actually changed, so a
   control the operator is touching is left completely alone. Use it for
   ANY block containing a button or an input. */
function paint(id,html){
  var e=el(id); if(!e)return;
  if(e.__html===html)return;
  e.__html=html; e.innerHTML=html;
}
function renderNodes(){
  var g=S.groups||[],out=[];
  g.forEach(function(grp){
    out.push('<div class="grp"><h2>'+esc(grp.label)+'</h2><p class="why">'+esc(grp.why)+'</p>');
    (grp.nodes||[]).forEach(function(n){
      /* A node its systemd unit brings back gets RESTART, never stop, and no
         start button while it is on its way back -- a start in that gap runs a
         second copy. The server holds both rules; see node_registry. */
      var b='',st=n.running?'up':(n.restarting?'restarting':'down');
      if(!n.running&&n.restarting) b='<span class="locked">'+esc(n.unit)+' is bringing it back\u2026</span>';
      else if(!n.running) b='<button class="go" onclick="nodeAct(\'start\',\''+n.name+'\')">Start</button>';
      else if(n.may_stop&&n.verb==='restart') b=restartButton(n.name,'Restart');
      else if(n.may_stop) b='<button class="danger" onclick="nodeAct(\'stop\',\''+n.name+'\')">Stop</button>';
      else b='<span class="badge" title="'+esc(n.stop_reason)+'">protected \u2014 stop from a terminal</span>';
      out.push('<div class="node"><span class="pill '+st+'">'+
        {up:'running',restarting:'restarting',down:'stopped'}[st]+'</span>'+
        '<div class="nmwrap"><span class="nm">'+esc(n.label)+'</span>'+
        '<span class="note">'+esc(n.note||'')+'</span></div>'+
        '<span class="note mono">'+esc(n.running?(n.detail||''):'')+'</span>'+b+'</div>');
    });
    out.push('</div>');
  });
  paint('nodes',out.join('')||'<p class="hint">no registry</p>');
}
/* Restart or stop while ARMED asks first. Restarting camera_node mid-sortie can
   be exactly right (a frozen feed), but a slipped click costs a gap in the
   recording, so it is never one click while flying. */
function nodeAct(v,n){
  if(v!=='start'&&(S.tel||{}).armed&&!confirm((v==='restart'?'Restart ':'Stop ')
     +n+' while the aircraft is ARMED?'))return;
  post('/node/'+v,{name:n}).then(poll)}
function restartButton(name,label,title){
  return '<button class="warnb" title="'+esc(title||'')+'" onclick="nodeAct(\'restart\',\''+name+'\')">\u21bb '+label+'</button>'}
function nodeItem(name){var r=null;(S.groups||[]).forEach(function(g){
  (g.nodes||[]).forEach(function(x){if(x.name===name)r=x})});return r}

/* ---- telemetry ---- */
/* dot: a status colour shown beside the value (ok / warn / bad). */
function card(k,v,cls,dot){return '<div class="card '+(dot||'')+'"><div class="k">'+esc(k)+
  '</div><div class="v '+(cls||'')+'">'+(dot?'<span class="dot '+dot+'"></span>':'')
  +v+'</div></div>'}
function section(title,cards){
  return '<h3 class="sec">'+esc(title)+'</h3><div class="cards">'+cards.join('')+'</div>'}
var QUALITY={ok:'Good',warn:'Degraded',bad:'Poor',unknown:'No data'};
function renderTel(){
  var t=S.tel||{},out=[];
  var stale=function(ok){return ok?'':'bad'};
  var hdgBad=!t.pose_ok||t.heading===null||t.heading===undefined;
  out.push(section('Flight',[
    card('Mode',esc(t.mode||'\u2014'),stale(t.fcu_ok)),
    card('Armed',t.fcu_ok?(t.armed?'ARMED':'disarmed'):'\u2014',
         t.fcu_ok?(t.armed?'bad':'ok'):'bad'),
    card('Flight phase',esc((t.landed||'\u2014').replace('_',' ').toLowerCase()),
         t.flight_ok?(t.landed==='IN_AIR'||t.landed==='TAKEOFF'?'warn':'ok'):'bad'),
    card('Altitude (m)',fmt(t.alt_rel,1),stale(t.pose_ok)),
    card('Climb (m/s)',fmt(t.climb,1),stale(t.pose_ok)),
    card('Ground speed (m/s)',fmt(t.speed,1),stale(t.pose_ok)),
    card('Heading (\u00b0)',hdgBad?'unresolved':fmt(t.heading,1),hdgBad?'bad':'')]));
  out.push(section('Position',[
    card('Latitude','<span class="mono">'+fmt(t.lat,7)+'</span>',stale(t.pose_ok)),
    card('Longitude','<span class="mono">'+fmt(t.lon,7)+'</span>',stale(t.pose_ok)),
    card('Altitude AMSL (m)',fmt(t.alt_amsl,1),stale(t.pose_ok)),
    card('Altitude HAE (m)',fmt(t.alt_hae,1),stale(t.pose_ok)),
    card('Inside fence',t.pose_ok?(t.inside?'yes':'NO'):'\u2014',
         t.pose_ok?(t.inside?'ok':'bad'):'bad')]));
  out.push(section('Attitude',[
    card('Roll (\u00b0)',fmt(t.roll,1),stale(t.att_ok)),
    card('Pitch (\u00b0)',fmt(t.pitch,1),stale(t.att_ok)),
    card('Yaw (\u00b0)',fmt(t.yaw,1),stale(t.att_ok))]));
  /* GPS and battery take their colour from the pre-flight checks, so the
     thresholds live in one place (preflight_core) and a card can never be
     green while the checklist above it is red. */
  var G=S.gps,B=S.batt,gs=G?(chipState('gps')||'unknown'):'unknown',
      bs=B?chipState('batt'):'bad';
  out.push(section('GPS',[
    card('GPS quality',QUALITY[gs],gs==='unknown'?'':gs,gs==='unknown'?'':gs),
    card('Fix',G?esc(G.fix_name):'\u2014',''),
    card('Satellites',G&&G.satellites!==255?G.satellites:'\u2014',''),
    card('HDOP',G?fmt(G.hdop,2):'\u2014',''),
    card('Accuracy (m)',G?fmt(G.h_acc_m,2):'\u2014','')]));
  out.push(section('Battery',[
    card('Voltage',B?fmt(B.voltage,2)+' V':'\u2014',bs,bs),
    card('Per cell',B&&B.per_cell!=null?fmt(B.per_cell,2)+' V ('+B.cells+'S)':'\u2014',''),
    card('Above failsafe',B&&B.margin_v!=null?fmt(B.margin_v,2)+' V':'\u2014',bs),
    card('Time to failsafe',B&&B.minutes!=null?'~'+fmt(B.minutes,0)+' min':'\u2014',''),
    card('Current',B?fmt(B.current,1)+' A':'\u2014',''),
    card('Used',B?fmt(B.consumed_mah,0)+' mAh':'\u2014','')]));
  var L=S.ocs||{};
  out.push(section('Operator Control Station link',[
    card('Link',L.present?(L.connected?'up':'down'):'not running',
         L.present?(L.connected?'ok':'bad'):''),
    card('Sent',L.present?L.sent:'\u2014',''),
    card('Skipped',L.present?L.skipped:'\u2014',L.skipped?'warn':'')]));
  el('tel').innerHTML=out.join('');
  var h=[];
  if(hdgBad&&t.pose_ok) h.push('Heading is unresolved: GPS yaw is not available yet. The OCS refuses a heartbeat without it, deliberately.');
  if(L.present&&L.quiet_reason) h.push('The OCS is being sent nothing: '+L.quiet_reason);
  if(L.present&&L.phase_source==='fallback') h.push('Flight phase is coming from armed + altitude, NOT the autopilot. telemetry_bridge requests EXTENDED_SYS_STATE itself; check its log for "no EXTENDED_SYS_STATE yet".');
  if(B) h.push('Battery: current and mAh come from a sensor that is not yet calibrated \u2014 trust the volts.'+(B.basis?' Time to failsafe: '+B.basis+'.':''));
  el('telhint').innerHTML=h.map(function(x){return '<div>'+esc(x)+'</div>'}).join('');
}

/* ---- map ---- */
/* scale is pixels per metre. The default shows a buoy field (2 m grid); the
   zoom is remembered by this browser across reloads. */
var view={x:0,y:0},scale=16,follow=true,trail=[],drag=null,W=0,H=0;
try{var z0=+localStorage.getItem('rx26-map-scale');if(z0>=0.05&&z0<=200)scale=z0}catch(e){}
function sx(x){return W/2+(x-view.x)*scale}
function sy(y){return H/2-(y-view.y)*scale}
function resize(){var c=el('map');W=c.clientWidth;H=c.clientHeight;
  c.width=W*devicePixelRatio;c.height=H*devicePixelRatio;
  c.getContext('2d').setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0)}
function zoom(k){scale=Math.max(0.05,Math.min(200,scale*k));
  try{localStorage.setItem('rx26-map-scale',scale)}catch(e){}draw()}
function setToggle(id,on){var b=el(id);if(b)b.classList.toggle('on',!!on)}
function toggleFollow(){follow=!follow;setToggle('followb',follow);draw()}
function clearTrail(){trail=[];post('/map/clear_trail');draw()}
/* Confirmed, because a clear mid-flight throws away the watch time every buoy
   has built up. It never loses the MAP: buoy_mapper writes the files first. */
function clearBuoys(){
  if(!confirm('Clear the buoy map? It is saved on the Jetson first, but every '
     +'buoy starts watching again from zero.'))return;
  post('/map/clear_buoys').then(poll)}
function buoyShort(b){return b.label.replace('FLASHING','FLASH')}
/* The grid is fixed to the GROUND, not the screen: it pans with the map, so a
   buoy on a line stays on that line. Its spacing is re-chosen on every draw --
   the smallest GRID_STEPS entry still GRID_MIN_PX apart on screen -- so zooming
   in walks down to 1 m and 0.5 m, zooming out walks up, and the lines never
   crowd into a grey wash. Every fifth line is darker; the corner says both. */
var GRID_STEPS=[0.5,1,2,5,10,20,50,100,200,500,1000],GRID_MIN_PX=28;
/* Canvas text in the page's own face, not a terminal's. */
var FONTS='system-ui,-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif',
    MAPFONT='12px '+FONTS;
function gridStep(){
  for(var i=0;i<GRID_STEPS.length;i++)if(GRID_STEPS[i]*scale>=GRID_MIN_PX)return GRID_STEPS[i];
  return GRID_STEPS[GRID_STEPS.length-1]}
function metres(v){return (v<1?v.toFixed(1):String(v))+' m'}
function gridLines(g,st,centre,half,toScreen,vertical){
  for(var k=Math.ceil((centre-half)/st);k*st<=centre+half;k++){
    var p=Math.round(toScreen(k*st))+.5;
    g.strokeStyle=k%5?PAL.grid:PAL.gridMajor;g.beginPath();
    if(vertical){g.moveTo(p,0);g.lineTo(p,H)}else{g.moveTo(0,p);g.lineTo(W,p)}
    g.stroke()}}
/* Does a label box at (x,y,w,h) stay off every marker circle and every label
   already placed? */
function labelClear(x,y,w,h,marks,taken){
  var i,m,dx,dy,o;
  for(i=0;i<marks.length;i++){m=marks[i];
    dx=m.x-Math.max(x,Math.min(m.x,x+w));dy=m.y-Math.max(y,Math.min(m.y,y+h));
    if(dx*dx+dy*dy<(m.r+2)*(m.r+2))return false}
  for(i=0;i<taken.length;i++){o=taken[i];
    if(x<o[0]+o[2]&&o[0]<x+w&&y<o[1]+o[3]&&o[1]<y+h)return false}
  return true}
function drawGrid(g){
  var st=gridStep();g.lineWidth=1;
  gridLines(g,st,view.x,W/2/scale,sx,true);
  gridLines(g,st,view.y,H/2/scale,sy,false);
  return st}
/* ---- tape measure ----
   Click two points; the distance is drawn between them, and a third click
   starts again. A click within SNAP_PX of a buoy snaps to that buoy and FOLLOWS
   it -- the point is re-read from the live map every draw, so the distance
   tracks the buoy's position as its average settles. Map x/y are local metres,
   so the distance is metres with no further conversion. */
var measuring=false,meas=[],press=null,SNAP_PX=14;
function toggleMeasure(){measuring=!measuring;meas=[];
  setToggle('measureb',measuring);
  el('map').classList.toggle('measuring',measuring);draw()}
function measurePoint(cx,cy){
  var bm=(S.map||{}).buoys,best=null,bd=SNAP_PX;
  if(bm&&bm.buoys)bm.buoys.forEach(function(b){
    var d=Math.hypot(sx(b.x)-cx,sy(b.y)-cy);if(d<bd){bd=d;best=b}});
  return best?{id:best.id}:{x:view.x+(cx-W/2)/scale,y:view.y-(cy-H/2)/scale}}
function measResolve(p){
  if(p.id===undefined)return {x:p.x,y:p.y,name:''};
  var bm=(S.map||{}).buoys,hit=null;
  if(bm&&bm.buoys)bm.buoys.forEach(function(b){if(b.id===p.id)hit=b});
  return hit?{x:hit.x,y:hit.y,name:'B'+hit.id}:null}
function drawMeasure(g){
  var pts=meas.map(measResolve).filter(Boolean);
  if(!pts.length)return;
  g.fillStyle=PAL.fg;g.strokeStyle=PAL.fg;g.lineWidth=1.5;
  pts.forEach(function(p){g.beginPath();g.arc(sx(p.x),sy(p.y),3.5,0,6.284);g.fill()});
  if(pts.length<2)return;
  var a=pts[0],b=pts[1],d=Math.hypot(b.x-a.x,b.y-a.y);
  g.beginPath();g.moveTo(sx(a.x),sy(a.y));g.lineTo(sx(b.x),sy(b.y));
  g.setLineDash([6,4]);g.stroke();g.setLineDash([]);
  var lab=(a.name&&b.name?a.name+'–'+b.name+'  ':'')+(d<10?d.toFixed(2):d.toFixed(1))+' m';
  g.font='600 13px '+FONTS;
  var lw=g.measureText(lab).width,mx=(sx(a.x)+sx(b.x))/2,my=(sy(a.y)+sy(b.y))/2;
  /* raised well above the line: buoy labels sit level with their markers */
  g.fillStyle=PAL.panel;g.fillRect(mx-lw/2-5,my-32,lw+10,17);
  g.fillStyle=PAL.fg;g.fillText(lab,mx-lw/2,my-19)}

/* Pan on drag; a press that does not move is a click. Follow is switched off
   only by an actual drag, so a measuring click does not unpin the aircraft. */
(function(){var c=el('map');
  c.addEventListener('mousedown',function(e){drag={x:e.clientX,y:e.clientY};
    press={x:e.clientX,y:e.clientY}});
  addEventListener('mouseup',function(e){
    if(press&&measuring&&Math.hypot(e.clientX-press.x,e.clientY-press.y)<4){
      var r=c.getBoundingClientRect();
      if(meas.length>=2)meas=[];
      meas.push(measurePoint(e.clientX-r.left,e.clientY-r.top));draw()}
    press=null;drag=null;c.style.cursor=''});
  addEventListener('mousemove',function(e){if(!drag)return;
    if(press&&Math.hypot(e.clientX-press.x,e.clientY-press.y)<4)return;
    if(follow){follow=false;setToggle('followb',false)}
    c.style.cursor='grabbing';press=null;
    view.x-=(e.clientX-drag.x)/scale;view.y+=(e.clientY-drag.y)/scale;
    drag={x:e.clientX,y:e.clientY};draw()});
  c.addEventListener('wheel',function(e){e.preventDefault();
    zoom(e.deltaY<0?1.12:0.89)},{passive:false});
  addEventListener('resize',function(){lastLayout='';layoutVars();
    if(mapVisible()){resize();draw()}})})();
function draw(){
  var c=el('map');if(!W)resize();var g=c.getContext('2d');
  g.clearRect(0,0,W,H);
  var m=S.map||{},f=m.fence||[],v=m.veh;
  if(follow&&v){view.x=v.x;view.y=v.y}
  var st=drawGrid(g);
  /* fence */
  if(f.length>1){
    g.beginPath();
    f.forEach(function(p,i){i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1]))});
    g.closePath();
    g.fillStyle=PAL.fenceFill;g.fill();
    g.strokeStyle=PAL.accent;g.lineWidth=1.6;g.setLineDash([7,5]);g.stroke();
    g.setLineDash([]);
    g.fillStyle=PAL.accent;
    f.forEach(function(p){g.fillRect(sx(p[0])-2.5,sy(p[1])-2.5,5,5)});
  }
  /* trail */
  if(trail.length>1){
    g.beginPath();
    trail.forEach(function(p,i){i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1]))});
    g.strokeStyle=PAL.trail;g.lineWidth=1.4;g.stroke();
  }
  /* camera footprint: the patch the camera sees now, computed by gcs_node with
     the mapper's own heading arithmetic and sent only while the gimbal is at
     nadir. The thick edge is the top of the image. */
  var fp=m.footprint;
  if(fp&&fp.length===4){
    g.beginPath();
    fp.forEach(function(p,i){i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1]))});
    g.closePath();g.fillStyle=PAL.footFill;g.fill();
    g.strokeStyle=PAL.foot;g.lineWidth=1.2;g.setLineDash([5,4]);g.stroke();g.setLineDash([]);
    g.beginPath();g.moveTo(sx(fp[0][0]),sy(fp[0][1]));g.lineTo(sx(fp[1][0]),sy(fp[1][1]));
    g.lineWidth=3;g.stroke();
  }
  /* buoys: filled in their colour once decided, black for OFF, hollow grey
     while UNKNOWN; dashed ring = spread of the sightings */
  var bm=m.buoys;
  if(bm&&bm.buoys){
    var marks=bm.buoys.map(function(b){
      return {x:sx(b.x),y:sy(b.y),r:Math.max(5,0.23*scale)}});
    bm.buoys.forEach(function(b,i){
      var X=marks[i].x,Y=marks[i].y,r=marks[i].r;
      if(b.spread_m>0){g.beginPath();g.arc(X,Y,Math.max(r+2,b.spread_m*scale),0,6.284);
        g.setLineDash([3,3]);g.strokeStyle=PAL.ring;g.lineWidth=1;
        g.stroke();g.setLineDash([])}
      g.beginPath();g.arc(X,Y,r,0,6.284);
      if(b.state==='UNKNOWN'){g.strokeStyle=PAL.dim;g.lineWidth=2;g.stroke()}
      else{g.fillStyle=b.state==='OFF'?PAL.buoyOff:(PAL[BPAL[b.colour]]||PAL.fg);g.fill();
        g.strokeStyle=b.state==='SOLID'?PAL.buoySolid:PAL.dim;
        g.lineWidth=b.state==='SOLID'?2.5:1;g.stroke()}
    });
    /* Labels go on AFTER every marker, and each takes the first of right, left,
       below, above that covers no marker and no earlier label. At the
       competition's 3 m gate two buoys sit ~50 px apart at the default zoom,
       and a label drawn blindly to the right lands on its neighbour. */
    g.font=MAPFONT;g.fillStyle=PAL.fg;
    var taken=[];
    bm.buoys.forEach(function(b,i){
      var t='B'+b.id+' '+buoyShort(b),w=g.measureText(t).width,h=13,k=marks[i];
      var spots=[[k.x+k.r+4,k.y-h/2],[k.x-k.r-4-w,k.y-h/2],
                 [k.x-w/2,k.y+k.r+3],[k.x-w/2,k.y-k.r-3-h]];
      var at=spots.filter(function(p){return labelClear(p[0],p[1],w,h,marks,taken)})[0]||spots[0];
      taken.push([at[0],at[1],w,h]);
      g.fillText(t,at[0],at[1]+h-1);
    });
  }
  /* vehicle */
  if(v){
    var X=sx(v.x),Y=sy(v.y),a=(v.heading||0)*Math.PI/180;
    g.save();g.translate(X,Y);g.rotate(a);
    g.beginPath();g.moveTo(0,-11);g.lineTo(7,8);g.lineTo(0,4);g.lineTo(-7,8);
    g.closePath();
    g.fillStyle=m.inside===false?PAL.bad:PAL.ok;g.fill();
    g.restore();
    g.strokeStyle=PAL.vehRing;g.beginPath();
    g.arc(X,Y,Math.max(6,3*scale),0,6.284);g.stroke();
  }
  drawMeasure(g);
  /* what the grid means, on a backing box so it reads over the lines */
  var legend='grid '+metres(st)+' \u00b7 dark lines every '+metres(5*st);
  g.font=MAPFONT;
  g.fillStyle=PAL.panel;g.fillRect(8,H-25,g.measureText(legend).width+12,18);
  g.fillStyle=PAL.dim;g.fillText(legend,14,H-12);
  g.fillText('N \u2191',W-34,20);
}
function renderMap(){
  var m=S.map||{};
  if(m.veh){var p=[m.veh.x,m.veh.y];
    if(!trail.length||Math.hypot(p[0]-trail[trail.length-1][0],
        p[1]-trail[trail.length-1][1])>=(m.trail_gate||0.5))trail.push(p);
    if(trail.length>(m.trail_max||600))trail.splice(0,trail.length-(m.trail_max||600));
  }
  var G=S.gps;
  el('mapinfo').textContent=(m.veh
    ?((m.inside===false?'OUTSIDE FENCE  ':'inside fence  ')+
      'alt '+fmt((S.tel||{}).alt_rel,1)+' m  ·  '+trail.length+' trail pts')
    :'no pose')+'  ·  '+(G?'GPS '+G.fix_name
      +(G.satellites!==255?' '+G.satellites+' sats':'')
      +(G.hdop!=null?' HDOP '+fmt(G.hdop,2):''):'no GPS');
  checkLocks();
  if(mapVisible()){renderBuoys();draw();}
}

/* ---- lock beep ----
   A short tone when a buoy's state locks, so the pilot hears it without looking
   at the screen. Per browser, off by default. Browsers only allow sound after a
   click on the page -- the toggle is one; after a reload with it left on, the
   first click anywhere re-arms it. Buoys already locked when the page loads do
   not beep: only a lock this page saw happen. */
var soundOn=false,audio=null,lockedSeen=null;
try{soundOn=localStorage.getItem('rx26-lock-beep')==='on'}catch(e){}
function audioCtx(){
  if(!audio){var A=window.AudioContext||window.webkitAudioContext;if(!A)return null;audio=new A()}
  if(audio.state==='suspended')audio.resume();
  return audio}
function beep(){
  var a=audioCtx();if(!a)return;
  var o=a.createOscillator(),gn=a.createGain(),t=a.currentTime;
  o.type='sine';o.frequency.value=880;
  gn.gain.setValueAtTime(0.0001,t);gn.gain.exponentialRampToValueAtTime(0.3,t+0.01);
  gn.gain.exponentialRampToValueAtTime(0.0001,t+0.18);
  o.connect(gn);gn.connect(a.destination);o.start(t);o.stop(t+0.2)}
function toggleSound(){soundOn=!soundOn;
  try{localStorage.setItem('rx26-lock-beep',soundOn?'on':'off')}catch(e){}
  setToggle('soundb',soundOn);if(soundOn)beep()}
addEventListener('click',function(){if(soundOn)audioCtx()});
function checkLocks(){
  var bm=(S.map||{}).buoys;if(!bm)return;
  var now={},fresh=false;
  bm.buoys.forEach(function(b){if(b.locked){var k=bm.stem+':'+b.id;now[k]=1;
    if(lockedSeen&&!lockedSeen[k])fresh=true}});
  if(fresh&&soundOn)beep();
  lockedSeen=now}

/* ---- battery header and pre-flight strip (every tab) ----
   Both take their colour from preflight_core's verdicts, computed on the
   ground station, so the header, the strip and the Telemetry cards can never
   disagree about how worried to be. */
function chipState(key){
  var c=(S.preflight||[]).filter(function(x){return x.key===key})[0];
  return c&&(c.state==='ok'||c.state==='warn'||c.state==='bad')?c.state:''}
function hms(sec){
  sec=Math.max(0,Math.floor(sec));var h=Math.floor(sec/3600),m=Math.floor(sec%3600/60),x=sec%60;
  return h+':'+(m<10?'0':'')+m+':'+(x<10?'0':'')+x}
/* A tile is rebuilt through paint(), so it only touches the DOM when its text
   changes; its colour class is set every poll, which costs nothing. */
function tile(id,cls,k,big,sub,extra,title){
  paint(id,'<div class="k">'+k+'</div><div class="big">'+big+'</div>'
    +'<div class="sub">'+sub+'</div>'+(extra||''));
  var e=el(id);e.className='tile '+(cls||'');e.title=title||''}
function renderVitals(){
  var B=S.batt,t=S.tel||{},A=S.armed_time,G=S.gps;
  if(!B){tile('battery','bad','Battery','\u2014','no battery reading')}
  else{
    var sub=B.margin_v!=null?fmt(B.margin_v,2)+' V above failsafe':'failsafe level not read yet';
    if(t.armed&&B.minutes!=null)sub+=' \u00b7 ~'+(B.minutes>=60?'60+':fmt(B.minutes,0))+' min';
    else if(B.per_cell!=null)sub+=' \u00b7 '+fmt(B.per_cell,2)+' V/cell';
    /* The bar spans failsafe (empty) to a full pack at 4.2 V/cell. Under load it
       reads low -- it is the loaded voltage the failsafe watches, too. */
    var frac=(B.low_volt!=null&&B.cells)?Math.max(0,Math.min(1,
      (B.voltage-B.low_volt)/(4.2*B.cells-B.low_volt))):null;
    tile('battery',chipState('batt'),'Battery',fmt(B.voltage,2)+' V',sub,
      frac==null?'':'<div class="gauge"><i style="width:'+Math.round(frac*100)+'%"></i></div>',
      (B.basis||'')+(B.current!=null?' \u00b7 '+fmt(B.current,1)+' A (sensor uncalibrated)':''))}
  if(!A){tile('flighttime','','Armed this power-on','\u2014','')}
  else{tile('flighttime',A.armed?'armed':'','Armed this power-on',hms(A.seconds),
    (A.armed?'<b>ARMED now</b>':'disarmed')+' \u00b7 '+A.flights+' flight'+(A.flights===1?'':'s'),
    '',A.resumed?'resumed after a ground station restart':'')}
  var gs=G?(chipState('gps')||'unknown'):'unknown';
  tile('gpstile',gs==='unknown'?'':gs,'GPS','<span class="dot '+gs+'"></span>'+QUALITY[gs],
    G?esc(G.fix_name)+(G.satellites!==255?' \u00b7 '+G.satellites+' sats':'')
      +(G.hdop!=null?' \u00b7 HDOP '+fmt(G.hdop,2):''):'no GPS data');
  var altOk=t.pose_ok&&t.alt_rel!=null;
  tile('alttile',altOk?'':'bad','Altitude',altOk?fmt(t.alt_rel,1)+' m':'\u2014',
    altOk?esc(t.mode||'')+(t.landed?' \u00b7 '+esc(t.landed.replace('_',' ').toLowerCase()):''):'position stale')}
/* Every chip while disarmed; once armed only what needs attention, so the strip
   stays out of the way in flight and still shouts when something goes wrong. */
function renderPreflight(){
  var armed=(S.tel||{}).armed,list=(S.preflight||[]).filter(function(c){
    return !armed||(c.state!=='ok'&&c.state!=='off')});
  paint('preflight',list.length?'<span class="pfl">'+(armed?'Needs attention':'Pre-flight')
    +'</span>'+list.map(function(c){
    var quiet=c.state==='ok'||c.state==='off',
        label=c.label.charAt(0).toUpperCase()+c.label.slice(1);
    return '<span class="chip '+esc(c.state)+'" title="'+esc(c.detail)+'">'
      +'<span class="dot '+esc(c.state)+'"></span>'+esc(label+(quiet?'':': '+c.detail))+'</span>'
  }).join(''):'')}
/* Export links and the buoy table. paint() rewrites a div only when its markup
   changes, and these hold no buttons, so a poll-rate rewrite costs nothing. */
function renderBuoys(){
  var m=S.map||{},mp=m.mapper,bm=m.buoys,x;
  if(mp&&mp.serving){
    /* Protocol-relative, like the video: no absolute URL in the page. */
    var base='//'+location.hostname+':'+mp.port+'/buoys.';
    x='download: '+['kml','csv','plan','json'].map(function(f){
      return '<a href="'+base+f+'" download style="color:var(--accent)">'+f+'</a>'
    }).join(' · ');
  }else if(mp){x='buoy_mapper starting…'}
  else{x='buoy_mapper not running'}
  paint('mapexp','<span class="note" style="flex:none">'+x+'</span>');
  if(!bm){
    paint('buoylist',mp?'<p class="hint">no buoy map received in the last 3 s</p>':'');
    return;
  }
  var rows=bm.buoys.slice().sort(function(a,b){return a.id-b.id}).map(function(b){
    return '<tr><td>B'+b.id+'</td><td style="color:'+(BVAR[b.colour]||'var(--fg)')
      +'"><b>'+esc(b.label)+'</b>'+(b.locked?'':' <span class="note">watching</span>')
      +'</td><td>'+b.lat.toFixed(7)+', '+b.lon.toFixed(7)
      +'</td><td>±'+fmt(b.spread_m,2)+' m</td><td>'+b.sightings
      +'</td><td>'+fmt(b.observed_s,1)+' s</td><td>'+Math.round(100*b.lit_fraction)
      +'%</td></tr>';
  }).join('');
  paint('buoylist',
    '<table style="margin-top:8px;border-collapse:collapse;font-size:12.5px">'
    +'<tr style="color:var(--dim)"><td>buoy</td><td>state</td><td>lat, lon</td>'
    +'<td>spread</td><td>sightings</td><td>watched</td><td>lit</td></tr>'
    +(rows||'<tr><td colspan="7" class="note">no buoys yet</td></tr>')
    +'</table><p class="note">map '+esc(bm.stem)+' · used '+bm.used
    +' · edge '+bm.partial+' · low conf '+bm.low_conf+' · frames refused '
    +bm.rejected+(bm.reject_reason?' (last: '+esc(bm.reject_reason)+')':'')+'</p>');
}

/* ---- logs ---- */
function pollLogs(){
  post('/logs',{since:logSeq,limit:400},true).then(function(j){
    if(!j||!j.lines)return;
    if(j.dropped)dropped=j.dropped;
    j.lines.forEach(function(l){logs.push(l);logSeq=Math.max(logSeq,l.seq)});
    if(logs.length>4000)logs.splice(0,logs.length-4000);
    var sel=el('lnode'),have={};
    Array.prototype.forEach.call(sel.options,function(o){have[o.value]=1});
    (j.nodes||[]).forEach(function(n){if(!have[n]){
      var o=document.createElement('option');o.value=o.textContent=n;sel.appendChild(o)}});
    if(tab==='logs')repaintLogs();
  })
}
function repaintLogs(){
  var lv=+el('lvl').value,nd=el('lnode').value,box=el('logs');
  var near=box.scrollTop+box.clientHeight>=box.scrollHeight-40;
  var out=logs.filter(function(l){return l.level>=lv&&(!nd||l.name===nd)})
    .slice(-1200).map(function(l){
      return '<div class="lg '+l.lvl+'"><span class="t">'+esc(l.t)+'</span>'+
        '<span class="n">'+esc(l.name)+'</span><span class="m">'+esc(l.msg)+'</span></div>'});
  box.innerHTML=out.join('');
  if(near)box.scrollTop=box.scrollHeight;
  el('loginfo').textContent=logs.length+' held'+(dropped?('  ·  '+dropped+' dropped'):'');
}
function clearLogs(){logs=[];logSeq=0;dropped=0;post('/logs/clear').then(repaintLogs)}

/* ---- system ---- */
/* #power carries the only text input on the page. It goes through paint() for
   the same reason the buttons do -- see the note there. The rebuild that DOES
   happen when the markup changes is correct and wanted: the shutdown control
   must vanish the moment the vehicle arms, even if someone is mid-type. */
function renderSys(){
  var s=S.sys||{},o=[];
  o.push(card('Hostname',esc(s.hostname||'—')));
  o.push(card('CPU',s.cpu==null?'—':fmt(s.cpu,0)+' %',s.cpu>90?'bad':''));
  o.push(card('Temperature',s.temp==null?'—':fmt(s.temp,1)+' °C',s.temp>80?'bad':''));
  o.push(card('Memory',s.mem_used==null?'—':fmt(s.mem_used,1)+' / '+fmt(s.mem_total,1)+' GB'));
  o.push(card('Disk free',s.disk_free==null?'—':fmt(s.disk_free,1)+' GB',
        s.disk_free!=null&&s.disk_free<2?'bad':''));
  o.push(card('Uptime',esc(s.uptime||'—')));
  el('sys').innerHTML=o.join('');
  var p=S.power||{},w=s.workspace||{},h=[];
  h.push('<div class="grp"><h2>workspace</h2><p class="why">'+
    (w.persists
      ? 'Bind-mounted from <b>'+esc(w.source||'?')+'</b> at <b>'+esc(w.mount||'?')+
        '</b> — <code>git pull</code> on the host is visible in here.'
      : '<span style="color:var(--bad)">NOT a bind mount.</span> A <code>git pull</code> on the host is invisible to this container, and a rebuild will silently change nothing. See the README on recreating the container with <code>-v ~/robotx_ws:/root/robotx_ws</code>.')
    +'</p></div>');
  h.push('<div class="grp"><h2>power</h2>');
  if(!p.allowed){
    h.push('<p class="why locked">'+esc(p.reason||'power is disabled')+'</p>');
  }else{
    h.push('<p class="why">Type the hostname <b>'+esc(s.hostname)+
      '</b> to confirm. Refused while armed, and while the armed state is unknown.</p>'+
      '<div class="bar"><input id="pwconf" placeholder="hostname" size="18">'+
      '<button class="danger" onclick="power(\'shutdown\')">shut down</button>'+
      '<button class="danger" onclick="power(\'reboot\')">reboot</button></div>');
  }
  h.push('</div>');
  paint('power',h.join(''));
}
function power(v){post('/power',{verb:v,confirm:(el('pwconf')||{}).value||''})}

/* ---- poll ---- */
/* ---- camera ----
   The <img> src is set ONCE per source change, never on every poll: assigning
   src restarts the MJPEG connection, so re-setting it at the poll rate would
   tear the stream down and rebuild it five times a second. camSrc remembers
   what is already showing so the common case touches nothing. */
var camSrc=null;
/* Which stream the Camera tab is showing. Survives the 5 Hz poll because
   renderCam only touches the <img> when the URL actually changes -- assigning
   src restarts the MJPEG connection, and doing that at the poll rate would
   tear the stream down five times a second. */
var showDet=false;
function toggleDet(){showDet=!showDet;renderCam();}
/* The REC control lives in its OWN div, deliberately not inside #cam. That box
   is rewritten whenever the video source changes, and a button rebuilt under
   the operator's cursor mid-press is a button that misses the press. */
function setCapture(on){post('/camera/capture',{on:on}).then(poll)}
/* One compact row, so the video keeps the height. The explanations that used to
   sit beside each button are their hover titles now. Altitude used to be a big
   readout here; it is the Altitude tile in the header on every tab now, with the
   same blank-when-stale rule. */
function renderCamRec(){
  var c=S.cam||{},r=c.recording_sd,parts=[];
  if(!c.source){paint('camrec','');return}
  /* Stills and the 4K SD recording are separate: stills follow the ARM switch,
     the button drives the 4K. Reported separately because one control showing
     one state for two things is how "stills: off" ended up on screen while
     stills were being written. */
  var armed=(S.tel||{}).armed,stills=armed||r;
  var g=c.record_gate||'',keep=g.indexOf('keeping')===0;
  if(r===null||r===undefined){
    parts.push('<span class="note">capture state unknown \u2014 /uav/camera/status is stale</span>');
  }else{
    parts.push('<button class="toggle'+(r?' on':'')+'" onclick="setCapture('+(r?'false':'true')
      +')" title="keep this session even if the aircraft never arms; also runs the camera\'s 4K SD recording">'
      +(r?'\u25a0 Stop capture':'\u25cf Keep session')+'</button>');
  }
  /* Offered only when detector_node is actually serving: a button that points
     the <img> at a closed port yields connection-refused and stays on that
     error until someone reloads by hand. */
  var d=c.det;
  if(d&&!d.starting){
    parts.push('<button class="toggle'+(showDet?' on':'')+'" onclick="toggleDet()"'
      +' title="boxes are drawn on this view ONLY \u2014 the recorded stills stay clean, or the next model learns that a buoy is a thing with a rectangle on it">'
      +(showDet?'\u25a0 Hide detections':'\u25c9 Show detections')+'</button>');
  }else if(d&&d.starting){
    parts.push('<span class="note">detector loading the model\u2026</span>');
  }
  /* Restart here as well as on the Nodes tab: this is the tab that is open when
     the camera needs one -- after the camera is unplugged or power-cycled its
     RTSP pipeline does not rebuild itself. A RESTART, never a stop. */
  var cn=nodeItem('camera_node');
  if(cn&&cn.may_stop&&cn.verb==='restart'){
    parts.push(restartButton('camera_node','Restart camera',
      'after unplugging or power-cycling the camera, or if the video freezes. Recording pauses for a few seconds.'));
  }
  if(r!==null&&r!==undefined){
    parts.push('<span class="note">stills <b>'+(stills?'ON':'off')+'</b>'
      +(armed?' (armed)':(r?' (forced)':''))+' \u00b7 4K SD <b>'+(r?'ON':'off')+'</b></span>');
  }
  /* The gate is the important half: recording always runs, and a session
     heading for the bin must never be a silent surprise. */
  if(g)parts.push('<span class="note" style="font-weight:700;color:'
    +(keep?'var(--ok)':'var(--bad)')+'">'+esc(g)+'</span>');
  paint('camrec','<div class="bar">'+parts.join('')+'</div>');
}
function renderCam(){
  renderCamRec();
  var c=S.cam||{},host=location.hostname,box=el('cam');
  if(!c.source){
    camSrc=null;
    var cn=nodeItem('camera_node');
    box.innerHTML=(cn&&cn.restarting)
      ?'<p class="hint">camera_node is restarting \u2014 '+esc(cn.unit)
        +' brings it back in a few seconds.</p>'
      :'<p class="hint">camera_node is not running. Start it from '
        +'the <b>Nodes</b> tab.</p>';
    return;
  }
  if(c.starting){
    camSrc=null;
    box.innerHTML='<p class="hint">camera_node is up; waiting for its video '
      +'port to accept a connection…</p>';
    return;
  }
  /* Protocol-relative on purpose. It inherits the page's own scheme, so the
     video is never blocked as mixed content if this page is ever served over
     TLS — and it keeps the page free of an absolute URL, which bench_gcs
     checks for because a page that can fetch from elsewhere is a page that can
     fail on a field network with no route off the subnet. */
  var d=c.det, url;
  if(showDet&&d&&!d.starting){
    url='//'+host+':'+d.port+d.path;
  }else{
    if(showDet&&(!d||d.starting))showDet=false;  /* nothing to show yet */
    url='//'+host+':'+c.port+c.path;
  }
  if(camSrc!==url){
    camSrc=url;
    box.innerHTML='<img id="camimg" alt="camera" src="'+esc(url)+'">';
  }
}
function render(){
  renderVitals();renderPreflight();
  if(tab==='nodes')renderNodes(); else if(tab==='tel')renderTel();
  else if(tab==='sys')renderSys();
  if(camVisible())renderCam();
  renderMap();
  layoutVars();
}
function poll(){
  fetch('/state',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){
    S=j;var b=el('banner');
    if(j.error){b.textContent=j.error;b.className='bad'}
    else{
      var t=j.tel||{},bad=[];
      if(!t.pose_ok)bad.push('pose stale');
      if(!t.fcu_ok)bad.push('fcu stale');
      if(!t.flight_ok)bad.push('flight-state stale');
      b.textContent=bad.length?bad.join(' · '):'\u25cf Live';
      b.className=bad.length?'bad':'';
    }
    render();
  }).catch(function(e){var b=el('banner');
    b.textContent='ground station unreachable';b.className='bad'})
}
setToggle('soundb',soundOn);
var startTab='nodes';
try{var t0=localStorage.getItem('rx26-tab');
  if(TABS.some(function(p){return p[0]===t0}))startTab=t0}catch(e){}
show(startTab);poll();setInterval(poll,POLL);pollLogs();setInterval(pollLogs,1000);
</script></body></html>
"""


def render(poll_ms: float) -> bytes:
    """The page, with the browser's poll period baked in.

    Rendered once at node start rather than per request: it is a constant, and
    re-templating it on every GET would put string work on the path a laptop
    hits several times a second.
    """
    return PAGE.replace("__POLL_MS__", str(int(poll_ms))).encode("utf-8")
