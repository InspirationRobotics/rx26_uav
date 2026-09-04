"""gcs_page — the operator's page, as one string.

Five tabs: Nodes, Telemetry, Map, Logs, System. No build step, no framework, no
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
"""

PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>rx26_uav ground station</title>
<style>
:root{--bg:#11151a;--panel:#1a1f27;--line:#2b3240;--fg:#dfe6ef;--dim:#8b97a8;
      --ok:#4ec27b;--warn:#e0a33e;--bad:#e2564a;--accent:#57a6ff;}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.45 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
header{display:flex;align-items:center;gap:14px;padding:10px 14px;
       background:var(--panel);border-bottom:1px solid var(--line);
       position:sticky;top:0;z-index:5;flex-wrap:wrap}
h1{font-size:15px;margin:0;font-weight:600;letter-spacing:.4px}
#tabs{display:flex;gap:4px;flex-wrap:wrap}
.tab{padding:5px 12px;border:1px solid var(--line);border-radius:5px;
     cursor:pointer;background:transparent;color:var(--dim);font:inherit}
.tab.on{background:var(--accent);border-color:var(--accent);color:#08121f;
        font-weight:600}
#banner{margin-left:auto;font-size:12px;color:var(--dim)}
#banner.bad{color:var(--bad);font-weight:600}
main{padding:14px;max-width:1180px}
section{display:none} section.on{display:block}
.cards{display:grid;gap:8px;
       grid-template-columns:repeat(auto-fill,minmax(168px,1fr))}
.card{background:var(--panel);border:1px solid var(--line);border-radius:6px;
      padding:8px 10px}
.k{color:var(--dim);font-size:11px;text-transform:uppercase;letter-spacing:.5px}
.v{font-size:17px;margin-top:2px}
.v.bad{color:var(--bad)} .v.ok{color:var(--ok)} .v.warn{color:var(--warn)}
.grp{margin:0 0 16px}
.grp h2{font-size:12px;color:var(--dim);text-transform:uppercase;
        letter-spacing:.6px;margin:0 0 3px;font-weight:600}
.grp p.why{margin:0 0 8px;color:var(--dim);font-size:12px}
.node{display:flex;align-items:center;gap:10px;background:var(--panel);
      border:1px solid var(--line);border-radius:6px;padding:8px 10px;
      margin-bottom:6px;flex-wrap:wrap}
.dot{width:9px;height:9px;border-radius:50%;flex:none;background:var(--dim)}
.dot.up{background:var(--ok)} .dot.down{background:#454e5c}
.nm{font-weight:600;min-width:170px}
.note{color:var(--dim);font-size:12px;flex:1;min-width:180px}
button{font:inherit;padding:4px 11px;border-radius:5px;cursor:pointer;
       border:1px solid var(--line);background:#232a34;color:var(--fg)}
button:hover{border-color:var(--accent)}
button[disabled]{opacity:.4;cursor:not-allowed}
button.danger{border-color:var(--bad);color:var(--bad)}
button.go{border-color:var(--ok);color:var(--ok)}
.locked{color:var(--dim);font-size:12px;font-style:italic;max-width:520px}
#toast{position:fixed;right:14px;bottom:14px;background:var(--panel);
       border:1px solid var(--ok);color:var(--fg);padding:9px 13px;
       border-radius:6px;max-width:min(560px,86vw);display:none;z-index:9;
       white-space:pre-wrap}
#toast.bad{border-color:var(--bad)}
#mapwrap{position:relative;background:var(--panel);border:1px solid var(--line);
         border-radius:6px}
#map{width:100%;height:min(58vh,540px);display:block;cursor:grab}
#mapbar{display:flex;gap:6px;align-items:center;padding:7px 9px;
        border-bottom:1px solid var(--line);flex-wrap:wrap}
#logs{background:#0c1015;border:1px solid var(--line);border-radius:6px;
      padding:8px;height:min(60vh,560px);overflow:auto;font-size:12.5px}
.lg{display:flex;gap:8px;padding:1px 0;white-space:pre-wrap;word-break:break-word}
.lg .t{color:var(--dim);flex:none} .lg .n{color:var(--accent);flex:none}
.lg.WARN .m{color:var(--warn)} .lg.ERROR .m,.lg.FATAL .m{color:var(--bad)}
.lg.DEBUG{opacity:.62}
.bar{display:flex;gap:7px;align-items:center;margin-bottom:9px;flex-wrap:wrap}
select,input{font:inherit;background:#232a34;color:var(--fg);
             border:1px solid var(--line);border-radius:5px;padding:4px 7px}
.hint{color:var(--dim);font-size:12px;margin:8px 0 0}
</style></head><body>
<header>
  <h1>rx26_uav</h1>
  <div id="tabs"></div>
  <div id="banner">connecting…</div>
</header>
<main>
  <section id="s-nodes"><div id="nodes"></div>
    <p class="hint">Presence comes from the ROS graph <b>and</b> /proc, so a node
    started by systemd or by hand in another terminal shows here too.</p></section>
  <section id="s-tel"><div class="cards" id="tel"></div>
    <p class="hint" id="telhint"></p></section>
  <section id="s-map">
    <div id="mapwrap">
      <div id="mapbar">
        <button onclick="zoom(1.4)">+</button><button onclick="zoom(0.71)">−</button>
        <button id="followb" onclick="toggleFollow()">follow: on</button>
        <button onclick="clearTrail()">clear trail</button>
        <span id="mapinfo" class="note"></span>
      </div>
      <canvas id="map"></canvas>
    </div>
    <p class="hint">The polygon is the <b>same <code>geofence</code> parameter
    telemetry_bridge uploads</b> — what you see is what the autopilot was told.
    Drag to pan. The autopilot enforces the fence; this is a readout.</p>
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
    <p class="hint">From <code>/rosout</code>, not journalctl — we are inside a
    container and the host journal is on the other side of that boundary. It
    misses output written straight to stdout, and anything printed before a node
    finished constructing, which is exactly when a bad parameter kills one.</p>
  </section>
  <section id="s-cam"><div id="camrec"></div><div id="cam"></div>
    <p class="hint">The video is served by <code>camera_node</code> on its own
    port, not proxied through this one — megabytes of MJPEG through the
    ground station's snapshot path would make a stalled camera look like a
    stalled ground station. The tab waits for that port to actually accept a
    connection before pointing at it, because a process appears in the table
    seconds before its server binds.</p></section>
  <section id="s-sys"><div class="cards" id="sys"></div><div id="power"></div></section>
</main>
<div id="toast"></div>
<script>
var POLL=__POLL_MS__, S={}, tab='nodes', logs=[], logSeq=0, dropped=0;
var TABS=[['nodes','Nodes'],['tel','Telemetry'],['map','Map'],['cam','Camera'],['logs','Logs'],['sys','System']];
function el(i){return document.getElementById(i)}
function esc(s){return String(s==null?'':s).replace(/[&<>"]/g,function(c){
  return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function fmt(v,n){return (v===null||v===undefined||(typeof v==='number'&&isNaN(v)))
  ?'—':Number(v).toFixed(n)}
function toast(m,bad){var t=el('toast');t.textContent=m;t.className=bad?'bad':'';
  t.style.display='block';clearTimeout(t._h);t._h=setTimeout(function(){
  t.style.display='none'},bad?7000:3200)}
function post(p,b){return fetch(p,{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify(b||{})}).then(function(r){return r.json()}).then(function(j){
  toast(j.message||(j.ok?'ok':'failed'),!j.ok);return j}).catch(function(e){
  toast('request failed: '+e,true)})}
function show(t){tab=t;TABS.forEach(function(p){
  el('s-'+p[0]).className=(p[0]===t?'on':'');
  el('tb-'+p[0]).className='tab'+(p[0]===t?' on':'')});
  if(t==='map'){resize();draw()} if(t==='logs')repaintLogs(); render()}
(function(){el('tabs').innerHTML=TABS.map(function(p){
  return '<button class="tab" id="tb-'+p[0]+'" onclick="show(\''+p[0]+'\')">'+p[1]+'</button>'
  }).join('')})();

/* ---- nodes ---- */
function renderNodes(){
  var g=S.groups||[],out=[];
  g.forEach(function(grp){
    out.push('<div class="grp"><h2>'+esc(grp.label)+'</h2><p class="why">'+esc(grp.why)+'</p>');
    (grp.nodes||[]).forEach(function(n){
      var b='';
      if(!n.running) b='<button class="go" onclick="nodeAct(\'start\',\''+n.name+'\')">start</button>';
      else if(n.may_stop) b='<button class="danger" onclick="nodeAct(\'stop\',\''+n.name+'\')">stop</button>';
      else b='<span class="locked">'+esc(n.stop_reason)+'</span>';
      out.push('<div class="node"><span class="dot '+(n.running?'up':'down')+'"></span>'+
        '<span class="nm">'+esc(n.label)+'</span>'+
        '<span class="note">'+esc(n.detail||n.note||'')+'</span>'+b+'</div>');
    });
    out.push('</div>');
  });
  el('nodes').innerHTML=out.join('')||'<p class="hint">no registry</p>';
}
function nodeAct(v,n){post('/node/'+v,{name:n}).then(poll)}

/* ---- telemetry ---- */
function card(k,v,cls){return '<div class="card"><div class="k">'+esc(k)+
  '</div><div class="v '+(cls||'')+'">'+v+'</div></div>'}
function renderTel(){
  var t=S.tel||{},o=[];
  var stale=function(ok){return ok?'':'bad'};
  o.push(card('mode',esc(t.mode||'—'),stale(t.fcu_ok)));
  o.push(card('armed',t.fcu_ok?(t.armed?'ARMED':'disarmed'):'—',
        t.fcu_ok?(t.armed?'bad':'ok'):'bad'));
  o.push(card('flight phase',esc(t.landed||'—'),
        t.flight_ok?(t.landed==='IN_AIR'||t.landed==='TAKEOFF'?'warn':'ok'):'bad'));
  o.push(card('latitude',fmt(t.lat,7),stale(t.pose_ok)));
  o.push(card('longitude',fmt(t.lon,7),stale(t.pose_ok)));
  o.push(card('alt rel (m)',fmt(t.alt_rel,1),stale(t.pose_ok)));
  o.push(card('alt AMSL (m)',fmt(t.alt_amsl,1),stale(t.pose_ok)));
  o.push(card('alt HAE (m)',fmt(t.alt_hae,1),stale(t.pose_ok)));
  o.push(card('climb (m/s)',fmt(t.climb,1),stale(t.pose_ok)));
  o.push(card('gnd speed (m/s)',fmt(t.speed,1),stale(t.pose_ok)));
  var hdgBad=!t.pose_ok||t.heading===null||t.heading===undefined;
  o.push(card('heading (deg)',hdgBad?'NaN':fmt(t.heading,1),hdgBad?'bad':''));
  o.push(card('roll (deg)',fmt(t.roll,1),stale(t.att_ok)));
  o.push(card('pitch (deg)',fmt(t.pitch,1),stale(t.att_ok)));
  o.push(card('yaw (deg)',fmt(t.yaw,1),stale(t.att_ok)));
  o.push(card('inside fence',t.pose_ok?(t.inside?'yes':'NO'):'—',
        t.pose_ok?(t.inside?'ok':'bad'):'bad'));
  var L=S.ocs||{};
  o.push(card('OCS link',L.present?(L.connected?'up':'down'):'not running',
        L.present?(L.connected?'ok':'bad'):''));
  o.push(card('OCS sent',L.present?L.sent:'—',''));
  o.push(card('OCS skipped',L.present?L.skipped:'—',L.skipped?'warn':''));
  el('tel').innerHTML=o.join('');
  var h=[];
  if(hdgBad&&t.pose_ok) h.push('Heading is NaN: GPS yaw is unresolved. This is real data — the OCS validator refuses a heartbeat carrying it, deliberately.');
  if(L.present&&L.quiet_reason) h.push('OCS is being sent nothing: '+L.quiet_reason);
  if(L.present&&L.phase_source==='fallback') h.push('flight_phase is coming from armed+altitude, NOT the autopilot. Set SR0_EXT_STAT > 0.');
  el('telhint').innerHTML=esc(h.join('  '));
}

/* ---- map ---- */
var view={x:0,y:0},scale=3,follow=true,trail=[],drag=null,W=0,H=0;
function sx(x){return W/2+(x-view.x)*scale}
function sy(y){return H/2-(y-view.y)*scale}
function resize(){var c=el('map');W=c.clientWidth;H=c.clientHeight;
  c.width=W*devicePixelRatio;c.height=H*devicePixelRatio;
  c.getContext('2d').setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0)}
function zoom(k){scale=Math.max(0.05,Math.min(200,scale*k));draw()}
function toggleFollow(){follow=!follow;
  el('followb').textContent='follow: '+(follow?'on':'off');draw()}
function clearTrail(){trail=[];post('/map/clear_trail');draw()}
(function(){var c=el('map');
  c.addEventListener('mousedown',function(e){drag={x:e.clientX,y:e.clientY};
    follow=false;el('followb').textContent='follow: off';c.style.cursor='grabbing'});
  addEventListener('mouseup',function(){drag=null;c.style.cursor='grab'});
  addEventListener('mousemove',function(e){if(!drag)return;
    view.x-=(e.clientX-drag.x)/scale;view.y+=(e.clientY-drag.y)/scale;
    drag={x:e.clientX,y:e.clientY};draw()});
  c.addEventListener('wheel',function(e){e.preventDefault();
    zoom(e.deltaY<0?1.12:0.89)},{passive:false});
  addEventListener('resize',function(){if(tab==='map'){resize();draw()}})})();
function draw(){
  var c=el('map');if(!W)resize();var g=c.getContext('2d');
  g.clearRect(0,0,W,H);
  var m=S.map||{},f=m.fence||[],v=m.veh;
  if(follow&&v){view.x=v.x;view.y=v.y}
  /* fence */
  if(f.length>1){
    g.beginPath();
    f.forEach(function(p,i){i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1]))});
    g.closePath();
    g.fillStyle='rgba(87,166,255,.07)';g.fill();
    g.strokeStyle='#57a6ff';g.lineWidth=1.6;g.setLineDash([7,5]);g.stroke();
    g.setLineDash([]);
    g.fillStyle='#57a6ff';
    f.forEach(function(p){g.fillRect(sx(p[0])-2.5,sy(p[1])-2.5,5,5)});
  }
  /* trail */
  if(trail.length>1){
    g.beginPath();
    trail.forEach(function(p,i){i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1]))});
    g.strokeStyle='rgba(78,194,123,.55)';g.lineWidth=1.4;g.stroke();
  }
  /* vehicle */
  if(v){
    var X=sx(v.x),Y=sy(v.y),a=(v.heading||0)*Math.PI/180;
    g.save();g.translate(X,Y);g.rotate(a);
    g.beginPath();g.moveTo(0,-11);g.lineTo(7,8);g.lineTo(0,4);g.lineTo(-7,8);
    g.closePath();
    g.fillStyle=m.inside===false?'#e2564a':'#4ec27b';g.fill();
    g.restore();
    g.strokeStyle='rgba(255,255,255,.18)';g.beginPath();
    g.arc(X,Y,Math.max(6,3*scale),0,6.284);g.stroke();
  }
  /* scale bar */
  var m50=50*scale;
  g.strokeStyle='#8b97a8';g.lineWidth=1;g.beginPath();
  g.moveTo(12,H-14);g.lineTo(12+m50,H-14);g.stroke();
  g.fillStyle='#8b97a8';g.font='11px monospace';g.fillText('50 m',14,H-19);
  g.fillText('N ↑',W-34,20);
}
function renderMap(){
  var m=S.map||{};
  if(m.veh){var p=[m.veh.x,m.veh.y];
    if(!trail.length||Math.hypot(p[0]-trail[trail.length-1][0],
        p[1]-trail[trail.length-1][1])>=(m.trail_gate||0.5))trail.push(p);
    if(trail.length>(m.trail_max||600))trail.splice(0,trail.length-(m.trail_max||600));
  }
  el('mapinfo').textContent=m.veh
    ?((m.inside===false?'OUTSIDE FENCE  ':'inside fence  ')+
      'alt '+fmt((S.tel||{}).alt_rel,1)+' m  ·  '+trail.length+' trail pts')
    :'no pose';
  if(tab==='map')draw();
}

/* ---- logs ---- */
function pollLogs(){
  post('/logs',{since:logSeq,limit:400}).then(function(j){
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
function renderSys(){
  var s=S.sys||{},o=[];
  o.push(card('hostname',esc(s.hostname||'—')));
  o.push(card('cpu',s.cpu==null?'—':fmt(s.cpu,0)+' %',s.cpu>90?'bad':''));
  o.push(card('temp',s.temp==null?'—':fmt(s.temp,1)+' °C',s.temp>80?'bad':''));
  o.push(card('memory',s.mem_used==null?'—':fmt(s.mem_used,1)+' / '+fmt(s.mem_total,1)+' GB'));
  o.push(card('disk free',s.disk_free==null?'—':fmt(s.disk_free,1)+' GB',
        s.disk_free!=null&&s.disk_free<2?'bad':''));
  o.push(card('uptime',esc(s.uptime||'—')));
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
  el('power').innerHTML=h.join('');
}
function power(v){post('/power',{verb:v,confirm:(el('pwconf')||{}).value||''})}

/* ---- poll ---- */
/* ---- camera ----
   The <img> src is set ONCE per source change, never on every poll: assigning
   src restarts the MJPEG connection, so re-setting it at the poll rate would
   tear the stream down and rebuild it five times a second. camSrc remembers
   what is already showing so the common case touches nothing. */
var camSrc=null;
/* The REC control lives in its OWN div, deliberately not inside #cam. That box
   is rewritten whenever the video source changes, and a button rebuilt under
   the operator's cursor mid-press is a button that misses the press. */
function sdRecord(on){post('/camera/sd_record',{on:on}).then(poll)}
function renderCamRec(){
  var c=S.cam||{},b=el('camrec'); if(!b)return;
  if(!c.source){b.innerHTML='';return}
  var r=c.recording_sd;
  if(r===null||r===undefined){
    b.innerHTML='<span class="note">SD recording state unknown — '
      +'/uav/camera/status is stale</span>';
    return;
  }
  b.innerHTML='<button onclick="sdRecord('+(r?'false':'true')+')">'
    +(r?'■ stop SD recording':'● start SD recording')+'</button> '
    +'<span class="note">camera 4K → microSD: <b>'+(r?'RECORDING':'off')
    +'</b> — the Jetson .mkv and the frame index record regardless</span>';
}
function renderCam(){
  renderCamRec();
  var c=S.cam||{},host=location.hostname,box=el('cam');
  if(!c.source){
    camSrc=null;
    box.innerHTML='<p class="hint">camera_node is not running. Start it from '
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
  var url='//'+host+':'+c.port+c.path;
  if(camSrc!==url){
    camSrc=url;
    box.innerHTML='<img id="camimg" alt="camera" style="max-width:100%;'
      +'border:1px solid #333" src="'+esc(url)+'">';
  }
}
function render(){
  if(tab==='nodes')renderNodes(); else if(tab==='tel')renderTel();
  else if(tab==='sys')renderSys(); else if(tab==='cam')renderCam();
  renderMap();
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
      b.textContent=bad.length?bad.join(' · '):'ok';
      b.className=bad.length?'bad':'';
    }
    render();
  }).catch(function(e){var b=el('banner');
    b.textContent='ground station unreachable';b.className='bad'})
}
show('nodes');poll();setInterval(poll,POLL);pollLogs();setInterval(pollLogs,1000);
</script></body></html>
"""


def render(poll_ms: float) -> bytes:
    """The page, with the browser's poll period baked in.

    Rendered once at node start rather than per request: it is a constant, and
    re-templating it on every GET would put string work on the path a laptop
    hits several times a second.
    """
    return PAGE.replace("__POLL_MS__", str(int(poll_ms))).encode("utf-8")
