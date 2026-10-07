"""gcs_page — the operator's page, as one string.

Eight tabs: Nodes, Telemetry, Camera + Map, Map, Camera, Logs, Radio, System. No build
step, no framework, no
CDN — the Jetson serves this to a laptop over field WiFi, and a page that needs
to fetch anything else is a page that does not load at a flight line.

THE PAGE EXPLAINS RULES; IT DOES NOT HOLD THEM. Every control here is re-checked
server-side on arrival (see gcs_server), because anyone can edit JavaScript in a
browser or curl the endpoint. A greyed-out button is a courtesy to the operator,
never a security boundary.

THE MAP DRAWS THE FENCE THE AUTOPILOT HOLDS, read back by telemetry_bridge —
not a copy, not a re-derivation — so what the operator sees is what the
autopilot enforces, whether it came from QGC or from the uploader. Until one has
been read the `geofence` param stands in, and the line under the map says so.

THE BUOY SEARCH'S CONTROLS ARE ON THE MAP TAB and cannot start a flight: ON only
arms the search for the pilot's switch into GUIDED, OFF makes it hold.

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
import json

PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>rx26_uav ground station</title>
<style>
/* Dark is the original palette. Light is built for a laptop in direct sun:
   near-black text on white, darker "dim" text and heavier lines than a typical
   light theme, because washed-out grey is exactly what disappears in glare. */
:root{--bg:#0f1318;--panel:#181d25;--panel2:#1e242e;--line:#2b3240;--fg:#e1e7ef;--dim:#8d99aa;
      --ok:#4ec27b;--warn:#e0a33e;--bad:#e2564a;--accent:#57a6ff;
      --on-accent:#08121f;--btn:#232a35;--btn-hover:#2b3341;--sunk:#0b0e12;--dot-off:#4a5361;
      --grid:rgba(223,230,239,.07);--grid-major:rgba(223,230,239,.17);
      --ring:rgba(223,230,239,.35);--fence-fill:rgba(87,166,255,.07);
      --trail:rgba(78,194,123,.55);--veh-ring:rgba(255,255,255,.18);
      --buoy-off:#000;--buoy-solid:#fff;
      --foot:rgba(224,163,62,.9);--foot-fill:rgba(224,163,62,.08);
      --b-red:#e2564a;--b-green:#4ec27b;--b-blue:#57a6ff;
      --ok-bg:rgba(78,194,123,.13);--warn-bg:rgba(224,163,62,.14);
      --bad-bg:rgba(226,86,74,.16);--accent-bg:rgba(87,166,255,.13);
      --logo-bg:#fff;--video-bg:#000;--shadow:0 1px 2px rgba(0,0,0,.35),0 4px 14px rgba(0,0,0,.22);
      color-scheme:dark}
:root[data-theme=light]{--bg:#f4f6f9;--panel:#fff;--panel2:#f7f9fb;--line:#b4bdc9;--fg:#0a0e13;
      --dim:#3b4655;--ok:#17743a;--warn:#8a5700;--bad:#b3241a;--accent:#0a56bd;
      --on-accent:#fff;--btn:#e8ecf1;--btn-hover:#dde3ea;--sunk:#fbfcfd;--dot-off:#98a2b0;
      --grid:rgba(10,14,19,.10);--grid-major:rgba(10,14,19,.28);
      --ring:rgba(10,14,19,.45);--fence-fill:rgba(10,86,189,.08);
      --trail:rgba(23,116,58,.8);--veh-ring:rgba(10,14,19,.3);
      --buoy-off:#000;--buoy-solid:#0a0e13;
      --foot:rgba(138,87,0,.9);--foot-fill:rgba(138,87,0,.07);
      --b-red:#c42a1d;--b-green:#157d38;--b-blue:#0a5ccf;
      --ok-bg:rgba(23,116,58,.10);--warn-bg:rgba(138,87,0,.11);
      --bad-bg:rgba(179,36,26,.10);--accent-bg:rgba(10,86,189,.09);
      --logo-bg:#fff;--video-bg:#000;--shadow:0 1px 2px rgba(10,14,19,.10),0 4px 14px rgba(10,14,19,.08);
      color-scheme:light}
*{box-sizing:border-box}
/* A normal reading face for words, monospace only where columns must line up
   (logs, coordinates, code). Numbers everywhere use tabular figures so a value
   ticking at the poll rate does not jitter sideways. */
body{margin:0;background:var(--bg);color:var(--fg);
     font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
     font-variant-numeric:tabular-nums}
code,.mono,#logs,#radiolog{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
a{color:var(--accent)}

/* ---- header: one bar of tabs, one row of tiles, on every tab ---- */
header{position:sticky;top:0;z-index:5;background:var(--panel);
       border-bottom:1px solid var(--line);box-shadow:var(--shadow)}
.topbar{display:flex;align-items:center;gap:14px;padding:0 12px;min-height:46px;flex-wrap:wrap}
/* The team's logos, on every tab. Dark line art, so they sit on a white badge
   in both themes; embedded, because at the flight line there is nothing to
   fetch them from (team_logos.py, made by tools/scripts/make_team_logos.py). */
.logos{display:flex;align-items:center;gap:6px;background:var(--logo-bg);
       border-radius:6px;padding:2px 6px;border:1px solid var(--line)}
.logos img{height:26px;width:auto;display:block}
.brand{font-size:17px;font-weight:700;letter-spacing:.2px}
#tabs{display:flex;flex-wrap:wrap;align-self:stretch}
.tab{padding:0 12px;border:0;border-bottom:3px solid transparent;border-radius:0;
     cursor:pointer;background:transparent;color:var(--dim);font:inherit;font-weight:600;
     font-size:14px;min-height:46px}
.tab:hover{color:var(--fg);border-bottom-color:var(--line);background:transparent}
.tab.on{color:var(--accent);border-bottom-color:var(--accent);background:transparent}
.spacer{flex:1}
#banner{font-size:12.5px;font-weight:700;padding:3px 11px;border-radius:12px;
        color:var(--ok);background:var(--ok-bg);white-space:nowrap}
#banner.bad{color:var(--bad);background:var(--bad-bg)}
.iconbtn{background:transparent;border:1px solid var(--line);border-radius:14px;
         padding:3px 11px;color:var(--dim);font-size:12.5px;font-weight:600}

/* The numbers wanted at a glance, on every tab, in ONE row. The coloured edge
   is the verdict; the big number is the reading; the line under it is shown in
   full on up to two lines, and every tile keeps room for both, so a value
   changing length never changes the height of the row. */
#vitals{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));
        gap:8px;padding:8px 12px 10px}
.tile{background:var(--panel2);border:1px solid var(--line);border-left:4px solid var(--line);
      border-radius:8px;padding:5px 10px 6px;min-width:0}
.tile .k{font-size:10.5px;font-weight:700;letter-spacing:.8px;text-transform:uppercase;
         color:var(--dim);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.tile .big{font-size:20px;font-weight:700;line-height:1.25;display:flex;align-items:center;
           gap:7px;white-space:nowrap;overflow:hidden}
.tile .sub{font-size:12px;line-height:1.3;color:var(--dim);min-height:2.6em}
.tile.ok{border-left-color:var(--ok)}
.tile.warn{border-left-color:var(--warn);background:var(--warn-bg)}
.tile.bad{border-left-color:var(--bad);background:var(--bad-bg)}
.tile.armed{border-left-color:var(--accent);background:var(--accent-bg)}
.tile.ok .big{color:var(--ok)} .tile.warn .big{color:var(--warn)}
.tile.bad .big{color:var(--bad)} .tile.armed .big{color:var(--accent)}
.tile.hidden{display:none}
.gauge{height:4px;border-radius:2px;background:var(--line);margin-top:3px;overflow:hidden}
.gauge i{display:block;height:100%;background:var(--dim)}
.tile.ok .gauge i{background:var(--ok)} .tile.warn .gauge i{background:var(--warn)}
.tile.bad .gauge i{background:var(--bad)}
/* The pre-flight checklist is a tile too: a dot per check, the words in each
   dot's tooltip and one click away. Every check while disarmed; once armed,
   only what needs attention, and the tile goes when nothing does. */
#pftile{grid-column:span 2}
@media (min-width:1500px){#pftile{grid-column:span 3}}
#preflight{display:flex;flex-wrap:wrap;gap:3px 5px;margin-top:3px}
.chip{display:inline-flex;align-items:center;gap:5px;font-size:11.5px;line-height:1.55;
      padding:0 7px;border-radius:10px;border:1px solid var(--line);background:var(--panel);
      white-space:nowrap;cursor:pointer;color:var(--fg)}
.chip.warn{border-color:var(--warn);background:var(--warn-bg);font-weight:600}
.chip.bad{border-color:var(--bad);background:var(--bad-bg);font-weight:700}
.dot{width:9px;height:9px;border-radius:50%;flex:none;display:inline-block;background:var(--dot-off)}
.big .dot{width:13px;height:13px}
.dot.ok{background:var(--ok)} .dot.warn{background:var(--warn)} .dot.bad{background:var(--bad)}
.dot.armed{background:var(--accent)}

/* ---- the page body ---- */
main{padding:12px;max-width:2200px;margin:0 auto}
section{display:none} section.on{display:block}
/* Tabs that are one big view FILL the window under the header: --hdr is the
   header's measured height (layoutVars), so nothing needs scrolling to be
   seen. */
#s-map.on{display:flex;gap:12px;height:calc(100vh - var(--hdr,120px) - 24px);min-height:460px}
#s-cam.on,#s-logs.on,#s-radio.on{display:flex;flex-direction:column;gap:10px;
  height:calc(100vh - var(--hdr,120px) - 24px);min-height:420px}
h3.sec{font-size:11px;font-weight:700;letter-spacing:.8px;text-transform:uppercase;
       color:var(--dim);margin:0 0 8px}
.pane{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px 12px;min-width:0}
.ph{font-size:11px;font-weight:700;letter-spacing:.8px;text-transform:uppercase;color:var(--dim);
    margin:0 0 6px;display:flex;align-items:center;gap:8px}
.ph .spacer{flex:1}
.cards{display:grid;gap:8px;grid-template-columns:repeat(auto-fill,minmax(180px,1fr))}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:8px 12px}
.card.ok{border-left:4px solid var(--ok)} .card.warn{border-left:4px solid var(--warn)}
.card.bad{border-left:4px solid var(--bad)}
.k{color:var(--dim);font-size:12px;font-weight:600}
.v{font-size:19px;font-weight:600;margin-top:1px;display:flex;align-items:center;gap:8px}
.v.bad{color:var(--bad)} .v.ok{color:var(--ok)} .v.warn{color:var(--warn)}
/* label : value rows, the Telemetry tab's panels */
.kvgrid{display:grid;gap:12px;grid-template-columns:repeat(auto-fill,minmax(290px,1fr));align-items:start}
.kv{display:flex;align-items:baseline;gap:12px;padding:5px 0;border-top:1px solid var(--line)}
.kv:first-of-type{border-top:0}
.kv .kk{color:var(--dim);font-size:13px;flex:1;min-width:0}
.kv .vv{font-size:16px;font-weight:600;display:flex;align-items:center;gap:7px;text-align:right}
.kv .vv.bad{color:var(--bad)} .kv .vv.ok{color:var(--ok)} .kv .vv.warn{color:var(--warn)}
.note{color:var(--dim);font-size:12.5px}
.bar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;flex:none}
.inl{display:contents}
.callout{margin:0 0 12px;padding:8px 12px;border-radius:8px;border-left:4px solid var(--warn);
         background:var(--warn-bg);font-size:13.5px}
.callout:empty{display:none}
.hint{color:var(--dim);font-size:13px;margin:8px 0}

/* ---- controls ---- */
button{font:inherit;font-size:13.5px;font-weight:600;padding:5px 12px;border-radius:6px;
       cursor:pointer;border:1px solid var(--line);background:var(--btn);color:var(--fg)}
button:hover{border-color:var(--accent);background:var(--btn-hover)}
button[disabled]{opacity:.4;cursor:not-allowed}
button.danger{border-color:var(--bad);color:var(--bad)}
button.go{border-color:var(--ok);color:var(--ok)}
button.warnb{border-color:var(--warn);color:var(--warn)}
button.toggle.on{background:var(--accent);border-color:var(--accent);color:var(--on-accent)}
select,input{font:inherit;font-size:13.5px;background:var(--btn);color:var(--fg);
             border:1px solid var(--line);border-radius:6px;padding:4px 8px}
.locked{color:var(--dim);font-size:12.5px;font-style:italic}
.badge{font-size:11.5px;font-weight:700;padding:2px 9px;border-radius:10px;
       border:1px solid var(--line);color:var(--dim);cursor:help;white-space:nowrap}
.spill{font-size:12px;font-weight:600;padding:2px 9px;border-radius:10px;white-space:nowrap;
       background:var(--panel2);border:1px solid var(--line);color:var(--dim)}
.spill.ok{color:var(--ok);background:var(--ok-bg);border-color:transparent}
.spill.bad{color:var(--bad);background:var(--bad-bg);border-color:transparent}
.spill.warn{color:var(--warn);background:var(--warn-bg);border-color:transparent}
#toast{position:fixed;right:16px;bottom:16px;background:var(--panel);
       border:1px solid var(--ok);border-left:6px solid var(--ok);color:var(--fg);
       padding:10px 14px;border-radius:8px;max-width:min(560px,86vw);display:none;
       z-index:9;white-space:pre-wrap;box-shadow:var(--shadow)}
#toast.bad{border-color:var(--bad)}
/* A tab's description lives behind its "?" and drops down over the page,
   instead of standing in paragraphs under everything. */
details.help{position:relative;flex:none}
details.help>summary{list-style:none;cursor:pointer;width:24px;height:24px;border-radius:50%;
  border:1px solid var(--line);display:flex;align-items:center;justify-content:center;
  font-size:13px;font-weight:700;color:var(--dim);background:var(--btn);user-select:none}
details.help>summary::-webkit-details-marker{display:none}
details.help[open]>summary{color:var(--on-accent);background:var(--accent);border-color:var(--accent)}
.helpbody{position:absolute;right:0;top:30px;z-index:20;width:min(460px,86vw);max-height:65vh;
  overflow:auto;background:var(--panel);border:1px solid var(--line);border-radius:10px;
  padding:10px 14px;font-size:13px;line-height:1.5;box-shadow:var(--shadow);color:var(--fg)}
.helpbody p{margin:0 0 8px} .helpbody p:last-child{margin:0}
.helpbody b{color:var(--fg)}

/* A tab whose panels start at the top keeps its "?" in the corner, over the
   empty end of the first panel's title row, instead of on a row of its own. */
#s-tel{position:relative}
.cornerhelp{position:absolute;right:10px;top:7px;z-index:3}

/* ---- Nodes ---- */
#nodes{display:grid;gap:12px;grid-template-columns:repeat(auto-fill,minmax(440px,1fr));align-items:start}
.grp{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px 12px 4px}
.gh{display:flex;align-items:center;gap:8px;margin-bottom:6px}
.gh h2{font-size:11px;color:var(--dim);text-transform:uppercase;letter-spacing:.8px;
       margin:0;font-weight:700;flex:1}
.node{display:flex;align-items:center;gap:10px;padding:7px 0;border-top:1px solid var(--line)}
.gh+.node{border-top:0}
.nmwrap{flex:1;min-width:0;display:flex;align-items:baseline;gap:8px;flex-wrap:wrap}
.nm{font-weight:700;font-size:14.5px}
.nstate{font-size:12px;color:var(--dim)}
.node.up .nstate{color:var(--ok)} .node.restarting .nstate{color:var(--warn)}
.ndet{font-size:11.5px;color:var(--dim);overflow:hidden;text-overflow:ellipsis;white-space:nowrap;max-width:240px}

/* ---- Map ---- */
#mapwrap{flex:1;min-width:0;display:flex;flex-direction:column;background:var(--panel);
         border:1px solid var(--line);border-radius:10px;overflow:hidden}
#mapbar,#searchbar{display:flex;gap:6px;align-items:center;padding:6px 8px;
        border-bottom:1px solid var(--line);flex-wrap:wrap;flex:none}
#mapbar .sep{width:1px;height:22px;background:var(--line);margin:0 3px}
#mapbar button{padding:4px 10px}
/* The buoy search's controls sit on the map, in the split view too: it is the
   tab open while it flies. */
#searchbar{gap:10px;background:var(--panel2)}
#searchbar .sk{font-size:10.5px;font-weight:700;letter-spacing:.8px;
               text-transform:uppercase;color:var(--dim)}
#searchbar label{display:flex;align-items:center;gap:6px;font-size:12.5px;color:var(--dim)}
#searchn{width:60px}
/* A two-position switch, not a button labelled with its own state: the lit half
   is where the switch IS. A single button reading "Search OFF" was read as a
   switch already in the ON position, and a search nobody had switched on looked
   like a search that would not start. */
.seg{display:inline-flex}
.seg button{border-radius:0;margin-left:-1px;min-width:46px;padding:4px 10px}
.seg button:first-child{border-radius:6px 0 0 6px;margin-left:0}
.seg button:last-child{border-radius:0 6px 6px 0}
.seg button.on{background:var(--accent);border-color:var(--accent);color:var(--on-accent)}
#searchstat{display:flex;align-items:center;gap:8px;font-weight:600;min-width:0;font-size:13px}
/* The canvas is absolutely placed in a box that the layout sizes, so the map
   fills exactly what is left -- the canvas's own pixel size never pushes back. */
#mapbox{position:relative;flex:1;min-height:0}
#map{position:absolute;left:0;top:0;width:100%;height:100%;display:block;cursor:grab}
#map.measuring{cursor:crosshair}
#mapinfo{position:absolute;right:8px;bottom:8px;font-size:12px;color:var(--dim);
         background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:2px 8px;
         pointer-events:none;white-space:nowrap}
#mapinfo.warn{color:var(--warn);border-color:var(--warn)}
#mapinfo.bad{color:var(--bad);border-color:var(--bad);font-weight:700}
#mapexp{font-size:12px;color:var(--dim);white-space:nowrap}
#mapexp a{color:var(--accent)}
/* Beside the map: the aircraft's attitude, its GPS antenna and the buoys. The
   buoy list scrolls inside its own pane, so the page never has to. */
#mapside{flex:none;width:300px;display:flex;flex-direction:column;gap:12px;min-height:0;
        overflow-y:auto;scrollbar-width:thin}
#mapside>.pane{flex:none}
#att3d{width:100%;height:150px;display:block}
#attspark{width:100%;height:46px;display:block;margin-top:6px}
.attnums{display:grid;grid-template-columns:repeat(3,1fr);gap:4px;text-align:center}
.attnums .v{font-size:20px;font-weight:700;justify-content:center}
.attnums .k{font-size:10.5px;color:var(--dim);text-transform:uppercase;letter-spacing:.05em}
#atttilt{font-size:12.5px;font-weight:600;text-align:center;border-radius:6px;padding:3px 6px;margin-top:6px}
/* A pane that folds: its title is the handle, and the fold is remembered. */
details.pane>summary{list-style:none;cursor:pointer;margin:0}
details.pane>summary::-webkit-details-marker{display:none}
details.pane>summary::after{content:'▾';margin-left:auto;font-size:12px}
details.pane:not([open])>summary::after{content:'▸'}
details.pane[open]>summary{margin-bottom:6px}
#antpane{flex:none}
#antpane .kv{padding:2px 0;align-items:flex-start}
#antpane .kv .kk{font-size:12px}
#antpane .kv .vv{font-size:12.5px;font-weight:500;flex-direction:column;align-items:flex-end;gap:0}
#buoypane{flex:1 0 170px!important;min-height:170px;display:flex;flex-direction:column}
#buoylist{flex:1;min-height:0;overflow:auto;margin:0 -4px;padding:0 4px}
.brow{display:flex;align-items:center;gap:8px;padding:5px 0;border-top:1px solid var(--line);cursor:default}
.brow:first-child{border-top:0}
.bdot{width:12px;height:12px;border-radius:50%;flex:none;border:1px solid var(--dim)}
.bstate{font-weight:600;flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
#buoystats{font-size:11.5px;color:var(--dim);margin-top:6px;flex:none}
/* A short window: the attitude view gives up height before the buoy list does. */
@media (max-height:820px){#att3d{height:118px}#attspark{height:34px}
  .attnums .v{font-size:17px}#atttilt{margin-top:4px;padding:2px 6px}}
@media (max-width:1000px){
  /* On a narrow window the tiles wrap to two rows: the header scrolls away
     rather than holding a third of the screen. */
  header{position:static}
  #s-map.on{flex-direction:column;height:auto}
  #mapwrap{flex:none;height:70vh}
  #mapside{width:auto}}

/* ---- Camera, and Camera + Map side by side ---- */
#cam{flex:1;min-height:0;display:flex;align-items:center;justify-content:center;
     background:var(--video-bg);border:1px solid var(--line);border-radius:10px;overflow:hidden}
#camimg{display:block;max-width:100%;max-height:100%;object-fit:contain}
#cam .hint{margin:0;color:#c9d1db}
main.split{max-width:none;display:grid;gap:12px;align-items:start;
           grid-template-columns:minmax(0,1.25fr) minmax(0,1fr)}
main.split #s-cam{grid-column:1;grid-row:1}
main.split #s-map{grid-column:2;grid-row:1}
/* Housekeeping stays on the full Map tab; in flight the split view keeps the
   map's height for the map. */
main.split #mapside,main.split #mapexp,main.split .wide-only,main.split #s-map details.help{display:none}
@media (max-width:900px){main.split{grid-template-columns:1fr}
  main.split #s-map{grid-column:1;grid-row:2}}

/* ---- Logs and Radio ---- */
#logs,#radiolog{flex:1;min-height:200px;background:var(--sunk);border:1px solid var(--line);
      border-radius:10px;overflow:auto;font-size:12.5px}
#logs{padding:8px 10px}
.lg{display:flex;gap:8px;padding:1px 0;white-space:pre-wrap;word-break:break-word}
.lg .t{color:var(--dim);flex:none} .lg .n{color:var(--accent);flex:none}
.lg.WARN .m{color:var(--warn)} .lg.ERROR .m,.lg.FATAL .m{color:var(--bad)}
.lg.DEBUG{opacity:.62}
#radiolog{padding:0 10px 8px}
#radiolog table{border-collapse:collapse;width:100%}
#radiolog th{position:sticky;top:0;background:var(--sunk);text-align:left;
             color:var(--dim);font-weight:600;padding:7px 12px 4px 0}
#radiolog td{padding:2px 12px 2px 0;vertical-align:top;white-space:nowrap}
#radiolog td.sum{white-space:normal;word-break:break-word;width:100%}
.dir{font-weight:700;font-size:11px;letter-spacing:.5px}
.dir.TX{color:var(--accent)} .dir.RX{color:var(--ok)}
.rgrid{display:grid;gap:12px;grid-template-columns:minmax(0,1fr) minmax(0,1fr);flex:none}
@media (max-width:900px){.rgrid{grid-template-columns:1fr}}

/* ---- System ---- */
#power{display:grid;gap:12px;grid-template-columns:repeat(auto-fill,minmax(360px,1fr));margin-top:12px}
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
    <div class="logos">__LOGOS__</div>
    <div class="brand" title="rx26_uav ground station">Ekko</div>
    <nav id="tabs"></nav>
    <div class="spacer"></div>
    <div id="banner">connecting…</div>
    <button id="themeb" class="iconbtn" onclick="toggleTheme()">theme</button>
  </div>
  <div id="vitals">
    <div id="modetile" class="tile"></div>
    <div id="battery" class="tile"></div>
    <div id="flighttime" class="tile"></div>
    <div id="gpstile" class="tile"></div>
    <div id="alttile" class="tile"></div>
    <div id="linktile" class="tile hidden"></div>
    <div id="searchtile" class="tile hidden"></div>
    <div id="pftile" class="tile hidden"><div class="k" id="pfk">Pre-flight</div><div id="preflight"></div></div>
  </div>
</header>
<main>
  <section id="s-nodes">
    <div class="bar" style="margin-bottom:10px"><span id="nodesum" class="note"></span><span class="spacer"></span>
      <details class="help"><summary title="About this tab">?</summary><div class="helpbody">
      <p>A node shows as running whether it was started here, by systemd, or by hand
      in a terminal: presence comes from the ROS graph <b>and</b> /proc.</p>
      <p>Nodes with a systemd unit (camera_node, ocs_client) come straight back when
      killed, so they get <b>Restart</b>, never Stop. Each group's <b>?</b> says
      what its nodes do.</p></div></details></div>
    <div id="nodes"></div>
  </section>
  <section id="s-tel">
    <div class="cornerhelp">
      <details class="help"><summary title="About this tab">?</summary><div class="helpbody">
      <p><b>Battery current and mAh</b> come from a sensor that is not calibrated
      yet: trust the volts. Time to failsafe is worked out from the voltage trend
      and needs a minute of armed flight.</p>
      <p>A value in red is stale or missing, never the last reading held over.</p>
      </div></details></div>
    <div class="callout" id="telhint"></div>
    <div id="tel" class="kvgrid"></div>
  </section>
  <section id="s-map">
    <div id="mapwrap">
      <div id="mapbar">
        <button onclick="zoom(1.4)" title="zoom in">+</button>
        <button onclick="zoom(0.71)" title="zoom out">−</button>
        <button onclick="fitView()" title="frame the fence, every buoy and the boat">Fit</button>
        <button id="followb" class="toggle on" onclick="toggleFollow()"
          title="keep the aircraft on the map: it pans only when the aircraft nears an edge">Follow</button>
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
        <details class="help"><summary title="About this tab">?</summary><div class="helpbody">
        <p>The polygon is the <b>fence read back from the autopilot</b>, the one it
        enforces. Until one has been read the <code>geofence</code> parameter stands
        in, and the corner of the map says so. Drag to pan; scroll or +/− to zoom.
        The grid is fixed to the ground and re-spaces itself as you zoom.</p>
        <p><b>Buoy search</b>: set the count and switch it ON. That alone moves
        nothing: <b>flip SC into GUIDED</b> to start. It climbs to 10 m, sweeps the
        fence 2 m inside it (the blue path; faded legs are flown), hovers over each
        UNKNOWN buoy until it locks, gives up on one after 10 s, and asks for RTL
        once the count is confirmed. SB takes over at any time; SC off and on
        resumes; OFF here makes Ekko hold. Confirmed buoys count, so clear the
        buoys before a fresh run.</p>
        <p><b>Measure</b>: click two points. A click on a buoy snaps to its centre.
        <b>The dashed amber box</b> is what the camera sees (thick edge = top of the
        image), drawn only at nadir. <b>Lock beep</b> sounds on this laptop when a
        buoy's state is decided; after a reload, click the page once to allow
        sound.</p>
        <p>A buoy reads <b>UNKNOWN</b> until watched in full view for 4 s. Its dashed
        ring is the <b>spread</b> of its sightings: small means they agree. Hover a
        buoy in the list for its position and counts. Downloads are the map as it
        is now; the Jetson also writes them on every disarm.</p>
        </div></details>
      </div>
      <div id="searchbar">
        <span class="sk">Buoy search</span>
        <span class="seg" title="ON never starts a flight: flip SC into GUIDED to start. OFF makes Ekko hold position."><button
          id="search-off" onclick="setSearch(false)" disabled>Off</button><button
          id="search-on" onclick="setSearch(true)" disabled>On</button></span>
        <label>Task
          <select id="searchtask" onchange="setPick('task',this.value)" disabled>
            <option value="task1">1 &mdash; Safe Passage</option></select></label>
        <label>Tier
          <select id="searchtier" onchange="setPick('tier',this.value)" disabled>
            <option value="advanced">Advanced &mdash; map it, then home</option>
            <option value="disruptive">Disruptive &mdash; stay with the boat</option>
          </select></label>
        <label>Buoys to find
          <input id="searchn" type="number" min="1" max="50" step="1" value="10"
            onchange="setCount()" disabled></label>
        <span id="searchstat"></span>
      </div>
      <div id="mapbox"><canvas id="map"></canvas><div id="mapinfo"></div></div>
    </div>
    <aside id="mapside">
      <div class="pane" id="attpanel">
        <div class="ph">Attitude</div>
        <canvas id="att3d" title="Ekko seen from the south, north away from you, turned, rolled and pitched as the autopilot reports it"></canvas>
        <div class="attnums">
          <div><div class="v" id="att-r">&mdash;</div><div class="k">roll</div></div>
          <div><div class="v" id="att-p">&mdash;</div><div class="k">pitch</div></div>
          <div><div class="v" id="att-h">&mdash;</div><div class="k">heading</div></div>
        </div>
        <div id="atttilt">no attitude yet</div>
        <canvas id="attspark" title="roll and pitch over the last 10 seconds: wobble shows as ripple"></canvas>
      </div>
      <div class="pane" id="buoypane">
        <div class="ph">Buoys<span class="spacer"></span><span id="buoycount"></span></div>
        <div id="buoylist"></div><div id="buoystats"></div></div>
      <details class="pane" id="antpane" style="display:none" open ontoggle="paneToggle(this)"
        title="Ekko's GPS antenna as the receiver reports it, to compare with another receiver: lat/lon to 1e-7 deg (~1 cm), height above the ELLIPSOID, ECEF computed from those, in the datum of the corrections in use (CRTN = NAD83)">
        <summary class="ph">GPS antenna</summary><div id="mapant"></div></details>
    </aside>
  </section>
  <section id="s-logs">
    <div class="bar">
      <select id="lvl" onchange="repaintLogs()">
        <option value="10">DEBUG+</option><option value="20" selected>INFO+</option>
        <option value="30">WARN+</option><option value="40">ERROR+</option>
      </select>
      <select id="lnode" onchange="repaintLogs()"><option value="">all nodes</option></select>
      <button onclick="clearLogs()">Clear</button>
      <span class="note" id="loginfo"></span>
      <span class="spacer"></span>
      <details class="help"><summary title="About this tab">?</summary><div class="helpbody">
      <p>From <code>/rosout</code>, not journalctl: the host journal is on the other
      side of the container boundary. It misses output written straight to stdout,
      and anything printed before a node finished starting, which is exactly when a
      bad parameter kills one. For those, <code>journalctl -u uav-&lt;unit&gt;</code>
      on the Jetson.</p></div></details>
    </div>
    <div id="logs"></div>
  </section>
  <section id="s-radio">
    <div class="rgrid">
      <div><h3 class="sec">On the radio</h3><div class="cards" id="radiosys"><p class="note">waiting for the radio log…</p></div></div>
      <div><h3 class="sec">Boat link <span style="text-transform:none;letter-spacing:0;font-weight:600">(estimate)</span></h3><div class="cards" id="radioboat"></div></div>
    </div>
    <div class="bar">
      <select id="rdir" onchange="repaintRadio()"><option value="">sent and heard</option>
        <option value="TX">sent</option><option value="RX">heard</option></select>
      <select id="rwho" onchange="repaintRadio()"><option value="">all systems</option></select>
      <select id="rname" onchange="repaintRadio()"><option value="">all messages</option></select>
      <label class="note"><input type="checkbox" id="rhb" onchange="repaintRadio()"> heartbeats</label>
      <button onclick="post('/radio/send_test')">Send test message</button>
      <button onclick="clearRadio()">Clear</button>
      <span class="note" id="radioinfo"></span>
      <span class="spacer"></span>
      <details class="help"><summary title="About this tab">?</summary><div class="helpbody">
      <p>Every frame <code>telemetry_bridge</code> sends to another system, and every
      frame another system puts on the radio. Ekko's own autopilot telemetry is not
      listed. Not visible here: frames the Cube does not pass to the Jetson (only
      TUNNEL and standard messages arrive), and signal strength.</p>
      <p><b>The boat numbers are an estimate.</b> Boat packets carry no sequence
      number, so the tab compares how many arrive per second with the 1 Hz the boat
      should send.</p>
      <p><b>Send test message</b> puts one inert text TUNNEL (type
      <code>0x80FE</code>) on the air, addressed to the boat. Watch for it in QGC's
      MAVLink Inspector, or with <code>check_mesh.py</code> on a laptop radio.</p>
      </div></details>
    </div>
    <div id="radiolog"></div>
  </section>
  <section id="s-cam">
    <div class="bar" id="cambar"><span id="cammodel" class="inl"></span><span
      id="cammodelnote" class="inl"></span><span id="camrec" class="inl"></span>
      <span class="spacer"></span>
      <details class="help"><summary title="About this tab">?</summary><div class="helpbody">
      <p><b>Stills</b> follow the ARM switch. <b>Keep session</b> keeps a session
      that never armed (bench capture) and also runs the camera's 4K SD recording.
      The coloured pill says whether this session's video will be kept.</p>
      <p><b>Show detections</b> draws boxes on this view only; the recorded stills
      stay clean. <b>Restart camera</b> after unplugging or power-cycling the camera,
      or if the video freezes.</p>
      <p>The video comes straight from <code>camera_node</code>'s own port, not
      through this page, so a stalled camera never looks like a stalled ground
      station.</p></div></details>
    </div>
    <div id="cam"></div>
  </section>
  <section id="s-sys"><div class="cards" id="sys"></div><div id="power"></div></section>
</main>
<div id="toast"></div>
<script>
var POLL=__POLL_MS__, S={}, tab='nodes', logs=[], logSeq=0, dropped=0;
var radio=[], radioSeq=0, radioStats=null;
var TABS=[['nodes','Nodes'],['tel','Telemetry'],['fly','Camera + Map'],['map','Map'],['cam','Camera'],['logs','Logs'],['radio','Radio'],['sys','System']];
var SECTIONS=['nodes','tel','map','cam','logs','radio','sys'];
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
  if(mapVisible()){resize();draw();drawAttitude();drawAttSpark()}
  if(t==='logs')repaintLogs(); if(t==='radio')pollRadio(); render()}
/* The header's height, which the views that fill the window subtract. Measured,
   not guessed: it changes when the checklist tile comes and goes and when the
   tiles wrap on a narrow window. Everything below the header is laid out by
   flexbox, and the map canvas follows its box through a ResizeObserver. */
var lastLayout='';
function layoutVars(){
  var h=document.querySelector('header').offsetHeight;
  if(String(h)===lastLayout)return;
  lastLayout=String(h);
  document.documentElement.style.setProperty('--hdr',h+'px');
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
  ['fg','dim','ok','warn','bad','accent','panel','grid','grid-major','ring',
   'fence-fill','trail','veh-ring','buoy-off','buoy-solid','b-red','b-green',
   'b-blue','foot','foot-fill'].forEach(function(k){
    PAL[k.replace(/-(\w)/g,function(_,c){return c.toUpperCase()})]=
      cs.getPropertyValue('--'+k).trim()})}
function setTheme(t){
  document.documentElement.setAttribute('data-theme',t);
  try{localStorage.setItem('rx26-theme',t)}catch(e){}
  var b=el('themeb');b.textContent=t==='light'?'\u263e dark':'\u2600 light';
  b.title='switch to the '+(t==='light'?'dark':'light')+' theme';
  readPalette();if(mapVisible()){draw();drawAttitude();drawAttSpark()}}
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
/* An open "?" closes on a click anywhere else, like any dropdown. */
document.addEventListener('click',function(e){
  Array.prototype.forEach.call(document.querySelectorAll('details.help[open]'),function(d){
    if(!d.contains(e.target))d.open=false})});
/* A "?" that drops down its words over the page, instead of paragraphs that
   stand under everything all the time. */
function help(html){
  return '<details class="help"><summary title="What these do">?</summary>'
    +'<div class="helpbody">'+html+'</div></details>'}
/* One card per group, one line per node: a dot, the name, its state, and the
   one button that applies. What a group and its nodes are FOR is behind the
   group's "?". */
function renderNodes(){
  var g=S.groups||[],out=[],up=0,all=0;
  g.forEach(function(grp){
    var notes=[],rows=[];
    (grp.nodes||[]).forEach(function(n){
      all++;if(n.running)up++;
      /* A node its systemd unit brings back gets RESTART, never stop, and no
         start button while it is on its way back -- a start in that gap runs a
         second copy. The server holds both rules; see node_registry. */
      var b='',st=n.running?'up':(n.restarting?'restarting':'down');
      if(!n.running&&n.restarting) b='<span class="locked" title="'+esc(n.unit)+' is bringing it back">coming back\u2026</span>';
      else if(!n.running) b='<button class="go" onclick="nodeAct(\'start\',\''+n.name+'\')">Start</button>';
      else if(n.may_stop&&n.verb==='restart') b=restartButton(n.name,'Restart');
      else if(n.may_stop) b='<button class="danger" onclick="nodeAct(\'stop\',\''+n.name+'\')">Stop</button>';
      else b='<span class="badge" title="'+esc(n.stop_reason)+'">protected</span>';
      if(n.note)notes.push('<p><b>'+esc(n.label)+'</b>: '+esc(n.note)+'</p>');
      rows.push('<div class="node '+st+'"><span class="dot '
        +{up:'ok',restarting:'warn',down:''}[st]+'"></span>'
        +'<div class="nmwrap"><span class="nm">'+esc(n.label)+'</span>'
        +'<span class="nstate">'+{up:'running',restarting:'restarting',down:'stopped'}[st]+'</span>'
        +(n.running&&n.detail?'<span class="ndet mono" title="'+esc(n.detail)+'">'+esc(n.detail)+'</span>':'')
        +'</div>'+b+'</div>');
    });
    out.push('<div class="grp"><div class="gh"><h2>'+esc(grp.label)+'</h2>'
      +help((grp.why?'<p>'+esc(grp.why)+'</p>':'')+notes.join(''))+'</div>'+rows.join('')+'</div>');
  });
  paint('nodes',out.join('')||'<p class="hint">no registry</p>');
  el('nodesum').textContent=all?up+' of '+all+' nodes running':'';
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
var QUALITY={ok:'Good',warn:'Degraded',bad:'Poor',unknown:'No data'};
/* The GPS label: the receiver's own mode when the radio carries it ("HAS" --
   the autopilot has no fix type for that), else the autopilot's fix name. */
function gpsName(G){return G.mode||G.fix_name}
/* GPS accuracy, metres: the receiver's own 95% figures (2DRMS) when the radio
   carries them; else the autopilot's h_acc -- which for the Septentrio is HALF
   that (ArduPilot scales HAccuracy by 0.005), so it reads optimistic. */
function gpsAcc(){
  var X=S.gnss,G=S.gps;
  if(X&&X.h_acc_m!=null)return {h:X.h_acc_m,v:X.v_acc_m,rx:true};
  return {h:G?G.h_acc_m:null,v:null,rx:false}}
/* Telemetry: one panel per subject, a row per reading. */
/* A reading with its unit, or a dash alone when there is none. */
function fu(v,n,u){var s=fmt(v,n);return s==='—'?s:s+u}
function kv(k,v,cls,dot,title){
  return '<div class="kv"'+(title?' title="'+esc(title)+'"':'')+'><span class="kk">'+esc(k)
    +'</span><span class="vv '+(cls||'')+'">'+(dot?'<span class="dot '+dot+'"></span>':'')+v+'</span></div>'}
function panel(title,rows){return '<div class="pane"><div class="ph">'+esc(title)+'</div>'+rows.join('')+'</div>'}
function renderTel(){
  var t=S.tel||{},out=[];
  var stale=function(ok){return ok?'':'bad'};
  var hdgBad=!t.pose_ok||t.heading===null||t.heading===undefined;
  out.push(panel('Flight',[
    kv('Mode',esc(t.mode||'\u2014'),stale(t.fcu_ok)),
    kv('Armed',t.fcu_ok?(t.armed?'ARMED':'disarmed'):'\u2014',
       t.fcu_ok?(t.armed?'bad':'ok'):'bad'),
    kv('Flight phase',esc((t.landed||'\u2014').replace('_',' ').toLowerCase()),
       t.flight_ok?(t.landed==='IN_AIR'||t.landed==='TAKEOFF'?'warn':'ok'):'bad'),
    kv('Altitude',fu(t.alt_rel,1,' m'),stale(t.pose_ok)),
    kv('Climb',fu(t.climb,1,' m/s'),stale(t.pose_ok)),
    kv('Ground speed',fu(t.speed,1,' m/s'),stale(t.pose_ok)),
    kv('Heading',hdgBad?'unresolved':fu(t.heading,1,'\u00b0'),hdgBad?'bad':'')]));
  out.push(panel('Position',[
    kv('Latitude','<span class="mono">'+fmt(t.lat,7)+'</span>',stale(t.pose_ok)),
    kv('Longitude','<span class="mono">'+fmt(t.lon,7)+'</span>',stale(t.pose_ok)),
    kv('Altitude AMSL',fu(t.alt_amsl,1,' m'),stale(t.pose_ok)),
    kv('Altitude HAE',fu(t.alt_hae,1,' m'),stale(t.pose_ok),'','height above the ellipsoid'),
    (S.map||{}).fence_desc&&(t.inside===true||t.inside===false)
      ?kv('Inside fence',t.pose_ok?(t.inside?'yes':'NO'):'\u2014',t.pose_ok?(t.inside?'ok':'bad'):'bad',
          '',S.map.fence_desc)
      :kv('Inside fence','unknown','','','no fence on the autopilot to judge by')]));
  out.push(panel('Attitude',[
    kv('Roll',fu(t.roll,1,'\u00b0'),stale(t.att_ok)),
    kv('Pitch',fu(t.pitch,1,'\u00b0'),stale(t.att_ok)),
    kv('Yaw',fu(t.yaw,1,'\u00b0'),stale(t.att_ok))]));
  /* GPS and battery take their colour from the pre-flight checks, so the
     thresholds live in one place (preflight_core) and a reading can never be
     green while the checklist above it is red. */
  var G=S.gps,B=S.batt,gs=G?(chipState('gps')||'unknown'):'unknown',
      bs=B?chipState('batt'):'bad',A=gpsAcc(),pct=A.rx?' (95%)':'';
  out.push(panel('GPS',[
    kv('Quality',QUALITY[gs],gs==='unknown'?'':gs,gs==='unknown'?'':gs),
    kv('Fix',G?esc(gpsName(G)):'\u2014',''),
    kv('Satellites',G&&G.satellites!==255?G.satellites:'\u2014',''),
    kv('HDOP',G?fmt(G.hdop,2):'\u2014',''),
    kv('Accuracy H'+pct,fu(A.h,2,' m'),''),
    kv('Accuracy V'+pct,fu(A.v,2,' m'),'')]));
  out.push(panel('Battery',[
    kv('Voltage',B?fmt(B.voltage,2)+' V':'\u2014',bs,bs),
    kv('Per cell',B&&B.per_cell!=null?fmt(B.per_cell,2)+' V ('+B.cells+'S)':'\u2014',''),
    kv('Above failsafe',B&&B.margin_v!=null?fmt(B.margin_v,2)+' V':'\u2014',bs),
    kv('Time to failsafe',B&&B.minutes!=null?'~'+fmt(B.minutes,0)+' min':'\u2014','','',B&&B.basis||''),
    kv('Current',B?fmt(B.current,1)+' A':'\u2014','','','sensor not calibrated: trust the volts'),
    kv('Used',B?fmt(B.consumed_mah,0)+' mAh':'\u2014','','','sensor not calibrated: trust the volts')]));
  var L=S.ocs||{};
  out.push(panel('Operator control station',[
    kv('Link',L.present?(L.connected?'connected':'not connected'):'ocs_client not running',
       L.present?(L.connected?'ok':'warn'):'','',
       'ocs_client sends Ekko\'s status at 2 Hz to the team\'s OCS laptop, the only thing that talks to '
       +'RoboNation\'s RoboCommand. Not connected is normal whenever the OCS program is not running.'),
    kv('Sent',L.present?L.sent:'\u2014',''),
    kv('Skipped',L.present?L.skipped:'\u2014',L.skipped?'warn':'')]));
  paint('tel',out.join(''));
  /* Only what needs attention gets words above the panels. */
  var h=[];
  if(hdgBad&&t.pose_ok) h.push('Heading is unresolved: GPS yaw is not available yet. The OCS refuses a heartbeat without it, deliberately.');
  if(L.present&&L.quiet_reason) h.push('The OCS is being sent nothing: '+L.quiet_reason);
  if(L.present&&L.phase_source==='fallback') h.push('Flight phase is coming from armed + altitude, NOT the autopilot. telemetry_bridge requests EXTENDED_SYS_STATE itself; check its log for "no EXTENDED_SYS_STATE yet".');
  paint('telhint',h.map(function(x){return '<div>'+esc(x)+'</div>'}).join(''));
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
/* Follow keeps the aircraft ON the map rather than in the middle of it: a map
   framed on the fence stays put while Ekko flies inside it, so the buoy field
   does not slide off the narrower map, and it pans only when the aircraft
   nears an edge. */
function keepInView(v){
  var mx=0.35*W/scale,my=0.35*H/scale;
  if(v.x<view.x-mx)view.x=v.x+mx;else if(v.x>view.x+mx)view.x=v.x-mx;
  if(v.y<view.y-my)view.y=v.y+my;else if(v.y>view.y+my)view.y=v.y-my}
/* Frame the fence, every buoy and the boat, with a margin. Done by the Fit
   button, and once each time the map re-anchors on a fence. */
var needFit=true;
function fitView(){
  var m=S.map||{},pts=(m.fence||[]).slice(),bm=m.buoys,fc=m.fence_circle;
  if(fc)pts.push([fc.x-fc.r,fc.y-fc.r],[fc.x+fc.r,fc.y+fc.r]);
  if(bm&&bm.buoys)bm.buoys.forEach(function(b){pts.push([b.x,b.y])});
  if(m.boat)pts.push([m.boat.x,m.boat.y]);
  /* Nothing from another venue: the params stand-in fence can be half a world
     from a map centred on the aircraft (map_origin), and fitting it zooms out
     to nothing. */
  pts=pts.filter(function(p){return Math.abs(p[0])<5e4&&Math.abs(p[1])<5e4});
  if(pts.length<2)return false;
  resize();
  /* Not laid out yet (the tab is still opening): try again on the next poll
     rather than fit to a zero-size map. */
  if(W<50||H<50)return false;
  var x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  pts.forEach(function(p){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);
    y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1])});
  view.x=(x0+x1)/2;view.y=(y0+y1)/2;
  scale=Math.max(0.05,Math.min(200,
    Math.min(W/Math.max(x1-x0,4),H/Math.max(y1-y0,4))/1.25));
  try{localStorage.setItem('rx26-map-scale',scale)}catch(e){}
  draw();return true}
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
    if(mapVisible()){resize();draw()}});
  /* The map's box is sized by the layout -- toolbars wrapping, the header
     changing height, the window -- and the canvas follows it. */
  if(window.ResizeObserver)new ResizeObserver(function(){
    if(mapVisible()){resize();draw()}}).observe(el('mapbox'))})();
function draw(){
  var c=el('map');if(!W)resize();var g=c.getContext('2d');
  g.clearRect(0,0,W,H);
  var m=S.map||{},f=m.fence||[],v=m.veh;
  if(follow&&v)keepInView(v);
  var st=drawGrid(g);
  /* fence: the autopilot's polygon in blue; the params stand-in, which the
     autopilot does NOT enforce, only as a faint outline */
  if(f.length>1){
    var held=m.fence_src==='autopilot';
    g.beginPath();
    f.forEach(function(p,i){i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1]))});
    g.closePath();
    if(held){g.fillStyle=PAL.fenceFill;g.fill()}
    g.strokeStyle=held?PAL.accent:PAL.dim;g.lineWidth=held?1.6:1;g.setLineDash(held?[7,5]:[2,5]);
    g.stroke();g.setLineDash([]);
    if(held){g.fillStyle=PAL.accent;
      f.forEach(function(p){g.fillRect(sx(p[0])-2.5,sy(p[1])-2.5,5,5)})}
  }
  /* the FENCE_RADIUS circle around home, when the autopilot enforces one */
  var fc=m.fence_circle;
  if(fc){
    var CX=sx(fc.x),CY=sy(fc.y);
    g.beginPath();g.arc(CX,CY,fc.r*scale,0,6.284);
    g.fillStyle=PAL.fenceFill;g.fill();
    g.strokeStyle=PAL.accent;g.lineWidth=1.6;g.setLineDash([7,5]);g.stroke();g.setLineDash([]);
    g.fillStyle=PAL.accent;g.beginPath();g.arc(CX,CY,3.5,0,6.284);g.fill();
    g.font=MAPFONT;g.fillText('home',CX+7,CY-6);
  }
  drawSearch(g,m.search);
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
     while UNKNOWN; dashed ring = spread of the sightings; a thick amber ring on
     the ones the search is hovering over */
  var bm=m.buoys,se=m.search||{},hov=se.hover||[],skip=se.skipped||[];
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
      if(hov.indexOf(b.id)>=0){g.beginPath();g.arc(X,Y,r+6,0,6.284);
        g.strokeStyle=PAL.warn;g.lineWidth=3;g.stroke()}
    });
    /* Labels go on AFTER every marker, and each takes the first of right, left,
       below, above that covers no marker and no earlier label. At the
       competition's 3 m gate two buoys sit ~50 px apart at the default zoom,
       and a label drawn blindly to the right lands on its neighbour. */
    g.font=MAPFONT;g.fillStyle=PAL.fg;
    var taken=[];
    bm.buoys.forEach(function(b,i){
      var t='B'+b.id+' '+buoyShort(b)+(skip.indexOf(b.id)>=0?' · skipped':''),
          w=g.measureText(t).width,h=13,k=marks[i];
      var spots=[[k.x+k.r+4,k.y-h/2],[k.x-k.r-4-w,k.y-h/2],
                 [k.x-w/2,k.y+k.r+3],[k.x-w/2,k.y-k.r-3-h]];
      var at=spots.filter(function(p){return labelClear(p[0],p[1],w,h,marks,taken)})[0]||spots[0];
      taken.push([at[0],at[1],w,h]);
      g.fillText(t,at[0],at[1]+h-1);
    });
  }
  /* Crusader, from the radio. Drawn whenever the boat is being heard; the label
     says what it is doing, because "where is the boat" and "what is it waiting
     for" are the same question during a Disruptive run. */
  var bt=m.boat;
  if(bt){
    var BX=sx(bt.x),BY=sy(bt.y);
    g.beginPath();g.arc(BX,BY,Math.max(7,0.9*scale),0,6.284);
    g.fillStyle=PAL.warn;g.fill();g.strokeStyle=PAL.fg;g.lineWidth=1.2;g.stroke();
    g.font='600 12px '+FONTS;g.fillStyle=PAL.fg;
    g.fillText('Crusader'+(bt.doing?' · '+bt.doing:''),BX+12,BY+4);
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
  drawAltitude(g);
}
/* Altitude, big, where the eye already is: at SUAS a judge watches the ground
   station for exactly this. Judged against the fence the autopilot holds:
   green up to where climbs stop (FENCE_ALT_MAX - FENCE_MARGIN, the checklist's
   number -- Ekko's 10 m working altitude, on purpose, so it must read green),
   amber above that, red within 0.5 m of FENCE_ALT_MAX itself, where the fence
   acts. A stale pose shows a dash, never the last altitude heard. */
var ALTFONT='700 40px '+FONTS;
function drawAltitude(g){
  var t=S.tel||{},c=(S.map||{}).ceiling||{},a=(t.pose_ok&&t.alt_rel!=null)?t.alt_rel:null,
      col=PAL.fg,sub;
  if(c.state==='on'){
    sub='above home \u00b7 climbs stop at '+fmt(c.stop,1)+' m \u00b7 fence '+fmt(c.alt_max,1)+' m';
    if(a!=null)col=a>=c.alt_max-0.5?PAL.bad:(a>c.stop+0.3?PAL.warn:PAL.ok)}
  else sub='above home \u00b7 '+(c.state==='off'?'no altitude fence'
                                  :'ceiling not read from the autopilot yet');
  var big=(a==null?'\u2014':a.toFixed(1))+' m';
  g.font=ALTFONT;var w1=g.measureText(big).width;
  g.font=MAPFONT;var w2=g.measureText(sub).width;
  g.fillStyle=PAL.panel;g.globalAlpha=0.9;g.fillRect(8,8,Math.max(w1,w2)+22,70);
  g.globalAlpha=1;
  g.font=ALTFONT;g.fillStyle=col;g.fillText(big,18,50);
  g.font=MAPFONT;g.fillStyle=PAL.dim;g.fillText(sub,18,69)}
/* ---- buoy search ----
   The dotted outline is the fence shrunk by the distance the search keeps from
   it; the blue path is the current pass, faded where it has been flown; the
   ring is where it is heading. Nothing is drawn from a stale status. */
function drawSearch(g,se){
  if(!se||se.phase==null)return;
  var ins=se.inset||[],plan=se.plan||[];
  if(ins.length>2){g.beginPath();
    ins.forEach(function(p,i){i?g.lineTo(sx(p[0]),sy(p[1])):g.moveTo(sx(p[0]),sy(p[1]))});
    g.closePath();g.strokeStyle=PAL.dim;g.lineWidth=1;g.setLineDash([2,4]);g.stroke();
    g.setLineDash([])}
  if(plan.length>1){
    var done=Math.max(0,(se.leg||1)-1);
    g.lineWidth=2;g.strokeStyle=PAL.accent;
    for(var i=1;i<plan.length;i++){
      g.globalAlpha=i<done?0.18:0.6;g.beginPath();
      g.moveTo(sx(plan[i-1][0]),sy(plan[i-1][1]));g.lineTo(sx(plan[i][0]),sy(plan[i][1]));
      g.stroke()}
    g.globalAlpha=1}
  if(se.target){var X=sx(se.target[0]),Y=sy(se.target[1]);
    g.strokeStyle=PAL.accent;g.lineWidth=2;g.beginPath();g.arc(X,Y,8,0,6.284);g.stroke();
    g.beginPath();g.moveTo(X-13,Y);g.lineTo(X+13,Y);g.moveTo(X,Y-13);g.lineTo(X,Y+13);g.stroke()}
}
/* OFF asks first while it is flying: it is safe (Ekko holds), but a slipped
   click stops a search mid-pass. ON never asks: it starts nothing. */
function setSearch(on){
  var s=(S.map||{}).search||{};
  if(on===!!s.enabled)return;
  if(!on&&s.flying&&!confirm('Switch the search OFF? Ekko stops and holds position in GUIDED. '
     +'Flip SC off and on to resume after switching it back on.'))return;
  post('/search/config',{enabled:on}).then(poll)}
/* The task and tier selectors. The tier decides what happens once the field is
   mapped: Advanced goes home, Disruptive stays over the boat's next gate and
   re-reads the lights when one changes. Switching mid-flight is allowed --
   search_node applies it to the running search. */
function setPick(what,value){var b={};b[what]=value;post('/search/config',b).then(poll)}
function setCount(){
  var v=Number(el('searchn').value);
  if(!(v>=1&&v<=50&&Math.floor(v)===v)){toast('Buoys to find: a whole number from 1 to 50',true);return}
  post('/search/config',{buoys_to_find:v}).then(poll)}
var lastSearchPhase=null;
function renderSearch(){
  var s=(S.map||{}).search||{running:false},on=el('search-on'),off=el('search-off'),
      n=el('searchn'),tier=el('searchtier'),task=el('searchtask'),
      live=!!s.running&&s.phase!=null;
  on.disabled=off.disabled=n.disabled=tier.disabled=task.disabled=!live;
  /* Follow the node, never under the operator's hand. */
  if(live&&document.activeElement!==tier&&s.tier&&tier.value!==s.tier)tier.value=s.tier;
  on.classList.toggle('on',live&&!!s.enabled);
  off.classList.toggle('on',live&&!s.enabled);
  /* The count follows the node, but never under the pilot's typing. */
  if(live&&document.activeElement!==n&&String(s.count)!==n.value)n.value=s.count;
  var html;
  if(!s.running)html='<span class="note">search_node is not running — start it on the Nodes tab</span>';
  else if(!live)html='<span class="note">no status from search_node</span>';
  else{var st=chipState('search')||'off',w=s.waiting||[];
    html='<span class="dot '+st+'"></span><span>'+esc(s.text)+'</span>'
      +(w.length>1?'<span class="note" title="'+esc(w.join('\n'))+'">+'+(w.length-1)
        +' more</span>':'')}
  paint('searchstat',html);
  /* Two tones when the last buoy is confirmed and it turns for home. */
  if(live&&s.phase==='rtl'&&lastSearchPhase!==null&&lastSearchPhase!=='rtl'&&soundOn){
    beep();setTimeout(beep,260)}
  lastSearchPhase=live?s.phase:null}
/* ---- attitude, in 3D ----
   Ekko seen from the south and above, north away from the viewer, rolled,
   pitched and TURNED exactly as the autopilot reports it, 20 times a second from
   GET /attitude (the whole /state is far too big to fetch that often, and wobble
   is invisible at 5 Hz). Chris chose the turning model over a chase view (19
   Sep): heading reads at a glance, at the price that with Ekko facing the viewer
   (heading ~180) a roll to its right shows as a tilt to the viewer's left. The
   compass ring is fixed to the ground. A stale reading greys it all out and says
   so: an attitude held over is exactly what this must never show.
   The model is EKKO'S OWN, from the Onshape assembly: ekko_model.py, made by
   tools/scripts/make_ekko_model.py -- cut to ~2500 triangles, props drawn as
   the discs they sweep, RED at the front (props, camera, GPS), BLUE at the
   rear. A stand-in quad draws only if that file is missing. Body frame x
   forward, y right, z down, metres; the camera and the ground ring scale to
   the model's size (MODEL_K). */
var ATT={ok:false},attHist=[],attBusy=false,ATT_SPAN_S=10,TILT_WARN=20,TILT_BAD=30;
function attLoop(){
  setTimeout(attLoop,50);
  if(tab!=='map'||document.hidden||attBusy)return;
  attBusy=true;
  fetch('/attitude',{cache:'no-store'}).then(function(r){return r.json()})
    .then(function(j){ATT=j;attBusy=false;attTick()})
    .catch(function(){ATT={ok:false};attBusy=false;attTick()})}
function attTick(){
  var now=Date.now()/1000;
  if(ATT.ok)attHist.push([now,ATT.roll,ATT.pitch]);
  while(attHist.length&&attHist[0][0]<now-ATT_SPAN_S)attHist.shift();
  drawAttitude();drawAttSpark()}
function hexRgb(h){h=(h||'#888').replace('#','');
  if(h.length===3)h=h[0]+h[0]+h[1]+h[1]+h[2]+h[2];
  return [parseInt(h.substr(0,2),16),parseInt(h.substr(2,2),16),parseInt(h.substr(4,2),16)]}
function shade(key,k){var c=hexRgb(PAL[key]);
  return 'rgb('+c.map(function(v){return Math.round(Math.min(255,v*k))}).join(',')+')'}
var EKKO_MESH=__EKKO_MESH__;
/* ekko_model.DATA: hex of uint16 nv, uint16 nt, int16 x/y/z mm per vertex,
   uint16 x3 per triangle, uint8 group per triangle; groups are [PAL key, alpha]. */
function meshFaces(M){
  var n=M.hex.length/2,b=new Uint8Array(n),i;
  for(i=0;i<n;i++)b[i]=parseInt(M.hex.substr(2*i,2),16);
  var dv=new DataView(b.buffer),nv=dv.getUint16(0,true),nt=dv.getUint16(2,true),
      o=4,V=[],F=[];
  for(i=0;i<nv;i++){V.push([dv.getInt16(o,true)/1000,dv.getInt16(o+2,true)/1000,
                            dv.getInt16(o+4,true)/1000]);o+=6}
  var gi=o+nt*6;
  for(i=0;i<nt;i++){var g=M.groups[b[gi+i]],q=o+6*i;
    F.push({p:[V[dv.getUint16(q,true)],V[dv.getUint16(q+2,true)],V[dv.getUint16(q+4,true)]],
            c:g[0],a:g[1]})}
  return F}
var MODEL=EKKO_MESH?meshFaces(EKKO_MESH):(function(){
  var F=[];
  function face(pts,c,al){F.push({p:pts,c:c,a:al||1})}
  function box(c,x0,x1,y0,y1,z0,z1,rot,dx,dy){
    var v=[[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]]
      .map(function(q){var cr=Math.cos(rot||0),sr=Math.sin(rot||0);
        return [q[0]*cr-q[1]*sr+(dx||0),q[0]*sr+q[1]*cr+(dy||0),q[2]]});
    [[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]].forEach(function(f){
      face(f.map(function(i){return v[i]}),c)})}
  function disc(c,x,y,z,r,n,al){var p=[];
    for(var i=0;i<n;i++){var a=i*2*Math.PI/n;p.push([x+r*Math.cos(a),y+r*Math.sin(a),z])}
    face(p,c,al)}
  function can(c,x,y,z0,z1,r){var n=8;
    for(var i=0;i<n;i++){var a=i*2*Math.PI/n,b=(i+1)*2*Math.PI/n;
      face([[x+r*Math.cos(a),y+r*Math.sin(a),z0],[x+r*Math.cos(b),y+r*Math.sin(b),z0],
            [x+r*Math.cos(b),y+r*Math.sin(b),z1],[x+r*Math.cos(a),y+r*Math.sin(a),z1]],c)}
    disc(c,x,y,z0,r,n)}
  box('dim',-0.11,0.11,-0.08,0.08,-0.05,0.04);            /* body */
  box('fg',-0.09,0.09,-0.045,0.045,-0.1,-0.05);          /* battery on top */
  box('bRed',0.1,0.16,-0.03,0.03,0.04,0.1);              /* the camera: the FRONT */
  [45,135,225,315].forEach(function(deg){
    var a=deg*Math.PI/180,R=0.38,x=R*Math.cos(a),y=R*Math.sin(a),front=deg===45||deg===315;
    box('fg',0.06,R,-0.014,0.014,-0.02,0.008,a);          /* arm */
    can('dim',x,y,-0.06,-0.02,0.03);                      /* motor */
    disc(front?'bRed':'accent',x,y,-0.066,0.18,16,0.5)});  /* prop disc */
  [-1,1].forEach(function(sgn){
    box('dim',-0.15,0.15,sgn*0.13-0.01,sgn*0.13+0.01,0.19,0.21);          /* skid */
    [-0.08,0.08].forEach(function(x){
      box('dim',x-0.008,x+0.008,sgn*0.13-0.008,sgn*0.13+0.008,0.04,0.19)})});
  return F})();
/* How big the model is next to the stand-in the camera was set up for (0.56 m
   from the centre to a prop tip), so any model sits the same in the view. */
var MODEL_K=MODEL.reduce(function(m,F){return F.p.reduce(function(m2,q){
  return Math.max(m2,Math.hypot(q[0],q[1]))},m)},0)/0.56||1;
function drawAttitude(){
  var c=el('att3d');if(!c||!c.clientWidth)return;
  var w=c.clientWidth,h=c.clientHeight,dpr=devicePixelRatio||1;
  if(c.width!==Math.round(w*dpr)){c.width=Math.round(w*dpr);c.height=Math.round(h*dpr)}
  var g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,w,h);
  var ok=!!ATT.ok,r=ok?ATT.roll:0,p=ok?ATT.pitch:0,hd=ok?ATT.heading:0;
  /* judged on the whole degrees shown, so the number and its colour agree */
  var tilt=Math.round(Math.acos(Math.cos(r*Math.PI/180)*Math.cos(p*Math.PI/180))*180/Math.PI);
  var lvl=!ok?'':(tilt>=TILT_BAD?'bad':(tilt>=TILT_WARN?'warn':'ok'));
  function sgn(v){return (v>0?'+':'')+v.toFixed(0)+'\u00b0'}
  el('att-r').textContent=ok?sgn(r):'\u2014';el('att-p').textContent=ok?sgn(p):'\u2014';
  el('att-h').textContent=ok?('00'+Math.round(hd)%360).slice(-3)+'\u00b0':'\u2014';
  /* each angle coloured by its own size, so the axis that is going is the red one */
  [['att-r',r],['att-p',p]].forEach(function(k){var v=Math.abs(Math.round(k[1]));
    el(k[0]).style.color=!ok?'':(v>=TILT_BAD?'var(--bad)':(v>=TILT_WARN?'var(--warn)':''))});
  var tb=el('atttilt');
  tb.textContent=!ok?'no attitude from the autopilot':('tilt '+tilt.toFixed(0)+'\u00b0'
    +(lvl==='bad'?' \u2014 about to tip':(lvl==='warn'?' \u2014 steep':' \u2014 level')));
  tb.style.background=lvl?'var(--'+lvl+'-bg)':'var(--sunk)';
  tb.style.color=lvl?'var(--'+lvl+')':'var(--dim)';
  /* camera: south of Ekko and above, looking north and down at A */
  var A=0.5,D=1.55*MODEL_K,ca=Math.cos(A),sa=Math.sin(A),f=Math.min(w,h)*1.35,cx=w/2,cy=h*0.52;
  function proj(q){var px=q[0],py=q[1]-D*sa,pz=q[2]+D*ca,
      yc=py*ca+pz*sa,zc=-py*sa+pz*ca;
    return [cx+f*px/zc,cy-f*yc/zc,zc]}
  /* body (FRD) -> ground (NED) by roll, pitch, then heading -> view (x east,
     y up, z north) */
  var cr=Math.cos(r*Math.PI/180),sr=Math.sin(r*Math.PI/180),
      cp=Math.cos(p*Math.PI/180),sp=Math.sin(p*Math.PI/180),
      ch=Math.cos(hd*Math.PI/180),shd=Math.sin(hd*Math.PI/180);
  function world(b){var y1=b[1]*cr-b[2]*sr,z1=b[1]*sr+b[2]*cr,
      x2=b[0]*cp+z1*sp,z2=-b[0]*sp+z1*cp,
      xn=x2*ch-y1*shd,yn=x2*shd+y1*ch;
    return [yn,-z2,xn]}
  /* compass ring on the ground: fixed to the world, so the model's tilt reads against it */
  var RG=0.52*MODEL_K,YG=-0.24*MODEL_K,i,pts=[];
  g.lineWidth=1.2;g.strokeStyle=PAL.ring;g.beginPath();
  for(i=0;i<=48;i++){var a=i*2*Math.PI/48,q=proj([RG*Math.sin(a),YG,RG*Math.cos(a)]);
    if(i)g.lineTo(q[0],q[1]);else g.moveTo(q[0],q[1])}
  g.stroke();
  g.font='600 12px '+FONTS;g.textAlign='center';
  [['N',0],['E',90],['S',180],['W',270]].forEach(function(k){
    var al=k[1]*Math.PI/180,q=proj([1.18*RG*Math.sin(al),YG,1.18*RG*Math.cos(al)]);
    g.fillStyle=k[0]==='N'?PAL.accent:PAL.dim;g.fillText(k[0],q[0],q[1]+4)});
  g.textAlign='left';
  /* the model, far faces first */
  var L=[-0.35,0.8,-0.48],faces=MODEL.map(function(F){
    var W3=F.p.map(world),S2=W3.map(proj),z=0;
    S2.forEach(function(q){z+=q[2]});
    var u=[W3[1][0]-W3[0][0],W3[1][1]-W3[0][1],W3[1][2]-W3[0][2]],
        v=[W3[2][0]-W3[0][0],W3[2][1]-W3[0][1],W3[2][2]-W3[0][2]],
        n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],
        nl=Math.hypot(n[0],n[1],n[2])||1,
        lit=Math.abs((n[0]*L[0]+n[1]*L[1]+n[2]*L[2])/nl);
    return {s:S2,z:z/S2.length,c:F.c,a:F.a,k:0.55+0.5*lit}});
  faces.sort(function(a,b){return b.z-a.z});
  faces.forEach(function(F){
    g.globalAlpha=(ok?1:0.3)*F.a;g.fillStyle=shade(F.c,F.k);g.beginPath();
    F.s.forEach(function(q,j){if(j)g.lineTo(q[0],q[1]);else g.moveTo(q[0],q[1])});
    g.closePath();g.fill()});
  g.globalAlpha=1;
  if(!ok){g.font='600 13px '+FONTS;g.fillStyle=PAL.dim;g.textAlign='center';
    g.fillText('no attitude',w/2,18);g.textAlign='left'}}
/* Roll and pitch over the last ATT_SPAN_S seconds. The scale grows with the
   largest angle, so a 2 degree wobble still shows as ripple. */
function drawAttSpark(){
  var c=el('attspark');if(!c||!c.clientWidth)return;
  var w=c.clientWidth,h=c.clientHeight,dpr=devicePixelRatio||1;
  if(c.width!==Math.round(w*dpr)){c.width=Math.round(w*dpr);c.height=Math.round(h*dpr)}
  var g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,w,h);
  var now=Date.now()/1000,big=5;
  attHist.forEach(function(e){big=Math.max(big,Math.abs(e[1]),Math.abs(e[2]))});
  big=Math.min(60,Math.ceil(big/5)*5);
  function X(t){return w*(1-(now-t)/ATT_SPAN_S)}
  function Y(v){return h/2-(v/big)*(h/2-8)}
  g.strokeStyle=PAL.gridMajor;g.lineWidth=1;g.beginPath();g.moveTo(0,h/2+.5);g.lineTo(w,h/2+.5);g.stroke();
  [[1,'accent'],[2,'warn']].forEach(function(k){
    g.strokeStyle=PAL[k[1]];g.lineWidth=1.5;g.beginPath();
    attHist.forEach(function(e,j){if(j)g.lineTo(X(e[0]),Y(e[k[0]]));else g.moveTo(X(e[0]),Y(e[k[0]]))});
    g.stroke()});
  g.font='11px '+FONTS;
  g.fillStyle=PAL.accent;g.fillText('roll',4,11);
  g.fillStyle=PAL.warn;g.fillText('pitch',30,11);
  g.fillStyle=PAL.dim;g.textAlign='right';
  g.fillText('\u00b1'+big+'\u00b0 \u00b7 last '+ATT_SPAN_S+' s',w-4,11);g.textAlign='left'}

/* Ekko's GPS ANTENNA, as the receiver reports it: the numbers to hold against
   another receiver. ECEF is computed from the ELLIPSOID height (GRS80; WGS84
   differs by well under a millimetre), so it is blank when no ellipsoid height
   came over -- an MSL height in its place would put Z ~35 m out. */
function ecef(lat,lon,h){
  var a=6378137,f=1/298.257222101,e2=f*(2-f),p=lat*Math.PI/180,l=lon*Math.PI/180,
      s=Math.sin(p),N=a/Math.sqrt(1-e2*s*s);
  return [(N+h)*Math.cos(p)*Math.cos(l),(N+h)*Math.cos(p)*Math.sin(l),(N*(1-e2)+h)*s]}
function renderAntenna(){
  var G=S.gps||{},A=gpsAcc(),pane=el('antpane');
  /* Only the radio page (gcs_radio.py) carries the antenna position; where the
     field is absent the pane is hidden rather than claiming there is no fix. */
  pane.style.display=('lat' in G)?'':'none';
  if(!('lat' in G))return;
  if(G.lat==null||G.lon==null){paint('mapant','<p class="note" style="margin:0">no position (no 3D fix)</p>');return}
  var h=G.alt_ellipsoid_m,x=h!=null?ecef(G.lat,G.lon,h):null,m=function(v){
    return '<span class="mono">'+v+'</span>'};
  paint('mapant',[
    kv('Lat, lon',m(G.lat.toFixed(7)+', '+G.lon.toFixed(7))),
    kv('Height (ellipsoid)',h!=null?m(h.toFixed(2)+' m'):'—'),
    kv('ECEF',x?m('X '+x[0].toFixed(2))+m('Y '+x[1].toFixed(2))+m('Z '+x[2].toFixed(2)):'—'),
    kv('Fix',esc(gpsName(G))+(A.h!=null?' ±'+fmt(A.h,2)+' m':''))].join(''))}
/* Folding panes keep their state in this browser across reloads. */
function paneToggle(d){try{localStorage.setItem('rx26-fold-'+d.id,d.open?'open':'shut')}catch(e){}}
(function(){try{['antpane'].forEach(function(id){
  if(localStorage.getItem('rx26-fold-'+id)==='shut')el(id).open=false})}catch(e){}})();

var originSeen=null;
function renderMap(){
  var m=S.map||{};
  /* The map re-anchors when the autopilot's fence is first read; a trail drawn
     about the old origin would be in the wrong place. */
  if(m.origin_id!==originSeen){trail=[];originSeen=m.origin_id;needFit=true}
  if(needFit&&mapVisible()&&(m.fence||[]).length>2&&fitView())needFit=false;
  renderSearch();
  if(m.veh){var p=[m.veh.x,m.veh.y];
    if(!trail.length||Math.hypot(p[0]-trail[trail.length-1][0],
        p[1]-trail[trail.length-1][1])>=(m.trail_gate||0.5))trail.push(p);
    if(trail.length>(m.trail_max||600))trail.splice(0,trail.length-(m.trail_max||600));
  }
  /* The map's own status, in its corner: where Ekko is against the fence, and
     whose fence it is. GPS and altitude are in the header already. */
  /* Judged against what the autopilot ENFORCES sideways (fence_desc): its
     circle, its polygon, or both. */
  var mi=el('mapinfo'),fd=m.fence_desc;
  mi.textContent=fd
    ?(m.veh?(m.inside===false?'OUTSIDE THE FENCE':(m.inside===true?'inside the fence':'not judged'))
      :'no position')+' · '+fd
    :'no fence on the autopilot to judge by'+(m.fence_problem?' · '+m.fence_problem:'');
  mi.className=m.inside===false?'bad':(fd?'':'warn');
  renderAntenna();
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
  /* Flight mode and armed state, first on every tab: what a pilot glances for
     before anything else. Blank when the autopilot's status is stale -- never
     the last mode it reported, which would look current. */
  if(!t.fcu_ok||!t.mode){tile('modetile','bad','Flight mode','—','no autopilot status')}
  else{tile('modetile',t.armed?'armed':'','Flight mode',esc(t.mode),
    t.armed?'<b>ARMED</b>':'disarmed')}
  if(!B){tile('battery','bad','Battery','\u2014','no battery reading')}
  else{
    var sub=B.margin_v!=null?'+'+fmt(B.margin_v,2)+' V over failsafe':'failsafe not read yet';
    if(t.armed&&B.minutes!=null)sub=(B.minutes>=60?'60+':'~'+fmt(B.minutes,0))+' min \u00b7 '+sub;
    else if(B.per_cell!=null)sub=fmt(B.per_cell,2)+' V/cell \u00b7 '+sub;
    /* The bar spans failsafe (empty) to a full pack at 4.2 V/cell. Under load it
       reads low -- it is the loaded voltage the failsafe watches, too. */
    var frac=(B.low_volt!=null&&B.cells)?Math.max(0,Math.min(1,
      (B.voltage-B.low_volt)/(4.2*B.cells-B.low_volt))):null;
    tile('battery',chipState('batt'),'Battery',fmt(B.voltage,2)+' V',sub,
      frac==null?'':'<div class="gauge"><i style="width:'+Math.round(frac*100)+'%"></i></div>',
      (B.basis||'')+(B.current!=null?' \u00b7 '+fmt(B.current,1)+' A (sensor uncalibrated)':''))}
  /* Armed time this power-on: Chris's flight-log number, so it survives a
     ground station restart and resets only when the aircraft is powered off. */
  if(!A){tile('flighttime','','Armed time','\u2014','',"",'armed time this power-on')}
  else{tile('flighttime',A.armed?'armed':'','Armed time',hms(A.seconds),
    A.flights+' flight'+(A.flights===1?'':'s')+' this power-on',
    '','armed time this power-on'+(A.resumed?' \u00b7 resumed after a ground station restart':''))}
  var gs=G?(chipState('gps')||'unknown'):'unknown',GA=gpsAcc();
  tile('gpstile',gs==='unknown'?'':gs,'GPS','<span class="dot '+gs+'"></span>'+QUALITY[gs],
    G?esc(gpsName(G))+(G.satellites!==255?' \u00b7 '+G.satellites+' sats':'')
      +(GA.h!=null?' \u00b7 \u00b1'+fmt(GA.h,2)+' m':''):'no GPS data','',
    G?esc(gpsName(G))+(G.satellites!==255?' \u00b7 '+G.satellites+' satellites':'')
      +(G.hdop!=null?' \u00b7 HDOP '+fmt(G.hdop,2):'')
      +(GA.h!=null?' \u00b7 accuracy \u00b1'+fmt(GA.h,2)+' m':''):'');
  var altOk=t.pose_ok&&t.alt_rel!=null;
  tile('alttile',altOk?'':'bad','Altitude',altOk?fmt(t.alt_rel,1)+' m':'\u2014',
    altOk?(t.landed?esc(t.landed.replace('_',' ').toLowerCase()):''):'position stale')
  /* The RADIO LINK, on every tab: what it is actually carrying, and how far
     BEHIND it is. Hidden unless the snapshot carries a link block, so the
     aircraft's own page -- which is not on the far end of a radio -- is
     unchanged. Lag is the number an operator cannot otherwise see: a link can
     be delivering full bandwidth and still be showing the past, which is how
     a 3D model kept moving ten seconds after the aircraft had landed. */
  var L=S.link,lt=el('linktile');
  if(!L){lt.className='tile hidden'}
  else if(!L.radio){
    tile('linktile','bad','Radio link','\u2014',
      'no telemetry \u2014 is QGC connected, with MAVLink forwarding on?')}
  else{
    /* USED, and what share of the air rate that is. Percentage first, because
       "9 kbit/s" means nothing without the ceiling beside it. */
    var lcls='',lsub=[];
    if(L.pct!=null)lsub.push(fmt(L.pct,0)+'% used');   /* of the air rate */
    lsub.push(L.lag_ms!=null?'lag '+L.lag_ms+' ms':'lag unknown');
    if(L.lag_ms!=null&&L.lag_ms>1500)lcls='warn';
    if(L.pct!=null&&L.pct>50)lcls='warn';
    /* dBm, not the raw 0-255 byte: 180 means nothing, -32 dBm does.
       Local/remote, then the margin over noise, which is what closes
       first when a link is about to fail. */
    var sig=L.dbm!=null?fmt(L.dbm,0)+' / '+fmt(L.rem_dbm,0)+' dBm (here / Ekko). '
           :(L.rssi!=null?'rssi '+L.rssi+' / '+L.remrssi+'. ':'');
    if(L.margin_db!=null)lsub.push(fmt(L.margin_db,0)+' dB margin');
    if(L.txbuf!=null&&L.txbuf<90){lsub.push('buf '+L.txbuf+'%');lcls='warn'}
    /* Used in the big number, the share of the air rate first on the line
       under it; the ceiling and everything else are in the tooltip. */
    tile('linktile',lcls,'Radio link',
      (L.kbit_s!=null?fmt(L.kbit_s,1)+' kbit/s':'\u2014'),
      lsub.join(' \u00b7 '),'',
      (L.kbit_s!=null?fmt(L.kbit_s,1)+' of '+fmt(L.air_kbit_s,0)+' kbit/s air rate. ':'')+sig
      +'Used: the bytes actually received over the last few seconds on '
      +esc(L.source||'')+'. The percentage is of the RAW air rate; real '
      +'throughput is roughly two thirds of it and is shared between every '
      +'radio on the mesh, so Ekko\'s share falls when Crusader joins. '
      +'Lag is how far BEHIND this data is, from the autopilot\'s own clock. '
      +'Signal is converted from the radio\'s raw 0-255 byte as dBm = raw/1.9 - 127 (SiK firmware); the margin over noise is what '
      +'closes first when a link is failing.')}
  /* The search tile shows only while search_node runs: found of wanted, big,
     and what it is doing in words. Blue while it is the one flying. */
  var se=(S.map||{}).search||{},st=el('searchtile');
  if(!se.running){st.className='tile hidden';return}
  if(se.phase==null){tile('searchtile','bad','Buoy search','\u2014','no status from search_node');return}
  var cls=se.flying?'armed':(chipState('search')==='warn'?'warn':
          (se.phase==='rtl'||se.phase==='complete'?'ok':''));
  tile('searchtile',cls,'Buoy search',se.found+' / '+se.count+' found',esc(se.text),'',
    (se.waiting||[]).join('\n'))}
/* Every chip while disarmed; once armed only what needs attention, so the strip
   stays out of the way in flight and still shouts when something goes wrong. */
/* The words for each check are in its tooltip, and a click puts them up where
   they can be read; the tile says how many need a look. */
function renderPreflight(){
  var armed=(S.tel||{}).armed,all=S.preflight||[],list=all.filter(function(c){
    return !armed||(c.state!=='ok'&&c.state!=='off')}),
      worry=all.filter(function(c){return c.state==='warn'||c.state==='bad'}),
      unknown=all.filter(function(c){return c.state==='unknown'}).length,
      bad=all.some(function(c){return c.state==='bad'});
  paint('preflight',list.map(function(c){
    var label=c.label.charAt(0).toUpperCase()+c.label.slice(1);
    return '<span class="chip '+esc(c.state)+'" title="'+esc(label+': '+c.detail)+'" onclick="pfInfo(\''
      +esc(c.key)+'\')"><span class="dot '+esc(c.state)+'"></span>'+esc(label)+'</span>'}).join(''));
  var t=el('pftile');
  t.className='tile '+(list.length?(bad?'bad':(worry.length?'warn':'')):'hidden');
  el('pfk').textContent=(armed?'Needs attention':'Pre-flight')
    +(worry.length?' · '+worry.length+' to check'
      :(unknown?' · '+unknown+' unknown':(all.length?' · all clear':'')))}
function pfInfo(key){
  var c=(S.preflight||[]).filter(function(x){return x.key===key})[0];
  if(c)toast(c.label.charAt(0).toUpperCase()+c.label.slice(1)+': '+c.detail,c.state==='bad')}
/* Export links and the buoy table. paint() rewrites a div only when its markup
   changes, and these hold no buttons, so a poll-rate rewrite costs nothing. */
function renderBuoys(){
  var m=S.map||{},mp=m.mapper,bm=m.buoys,x;
  if(mp&&mp.serving){
    /* Protocol-relative, like the video: no absolute URL in the page. */
    var base='//'+(mp.host||location.hostname)+':'+mp.port+'/buoys.';
    x='download '+['kml','csv','plan','json'].map(function(f){
      return '<a href="'+base+f+'" download>'+f+'</a>'
    }).join(' · ');
  }else if(mp){x='buoy_mapper starting…'}
  else{x=''}  /* the buoy pane says it is not running */
  paint('mapexp',x);
  if(!bm){
    paint('buoylist','<p class="hint" style="margin:4px 0">'+(mp?'no buoy map in the last 3 s'
      :'buoy_mapper is not running')+'</p>');
    paint('buoycount','');paint('buoystats','');
    return;
  }
  /* One line per buoy: its colour, its id, its state and how tight its position
     is. Everything else is in the line's tooltip. */
  var list=bm.buoys.slice().sort(function(a,b){return a.id-b.id}),locked=0;
  var rows=list.map(function(b){
    if(b.locked)locked++;
    var fill=b.state==='UNKNOWN'?'transparent':(b.state==='OFF'?'var(--buoy-off)'
          :(BVAR[b.colour]||'var(--fg)')),
        tip='B'+b.id+' '+b.label+'\n'+b.lat.toFixed(7)+', '+b.lon.toFixed(7)
          +'\nspread ±'+fmt(b.spread_m,2)+' m · '+b.sightings+' sightings · watched '
          +fmt(b.observed_s,1)+' s · lit '+Math.round(100*b.lit_fraction)+'%';
    return '<div class="brow" title="'+esc(tip)+'"><span class="bdot" style="background:'+fill
      +'"></span><b>B'+b.id+'</b><span class="bstate" style="color:'
      +(b.state==='UNKNOWN'?'var(--dim)':(BVAR[b.colour]||'var(--fg)'))+'">'+esc(b.label.replace(/_/g,' '))
      +(b.locked?'':' <span class="note">· watching</span>')+'</span>'
      +'<span class="note">±'+fmt(b.spread_m,2)+' m</span></div>'}).join('');
  paint('buoylist',rows||'<p class="hint" style="margin:4px 0">no buoys yet</p>');
  paint('buoycount',list.length?'<span class="note" style="text-transform:none;letter-spacing:0">'
    +locked+' of '+list.length+' locked</span>':'');
  paint('buoystats','<span title="frames used · boxes clipped by the edge · low confidence · frames refused'
    +(bm.reject_reason?' (last: '+esc(bm.reject_reason)+')':'')+'">map '+esc(bm.stem)
    +' · used '+bm.used+' · refused '+bm.rejected+'</span>');
}

/* ---- logs ---- */
/* Same guard as pollRadio, for the same race: an overlapping poll repeated
   log lines. */
var logBusy=false;
function pollLogs(){
  if(logBusy)return;
  logBusy=true;
  post('/logs',{since:logSeq,limit:400},true).then(function(j){
    logBusy=false;
    if(!j||!j.lines)return;
    if(j.dropped)dropped=j.dropped;
    j.lines.forEach(function(l){if(l.seq>logSeq){logs.push(l);logSeq=l.seq}});
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

/* ---- radio ----
   Asked for only while the tab is open. The per-system counts and the boat
   estimate are kept by the ground station, so nothing is missed while it is
   closed; the page asks only for frames it has not seen. */
/* ONE REQUEST AT A TIME, AND NEVER A ROW TWICE. This runs every second AND on
   every switch to the tab; a reply slower than a second let two or three
   requests carry the same `since`, and each appended the same rows -- every
   frame from the boat showed up three times while the ground station's own
   count said once. The busy flag stops the pile-up; the seq check makes an
   overlap harmless if one still happens. */
var radioBusy=false;
function pollRadio(){
  if(tab!=='radio'||radioBusy)return;
  radioBusy=true;
  post('/radio',{since:radioSeq,limit:400},true).then(function(j){
    radioBusy=false;
    if(!j||!j.rows)return;
    radioStats=j;
    j.rows.forEach(function(r){if(r.seq>radioSeq){radio.push(r);radioSeq=r.seq}});
    if(radio.length>3000)radio.splice(0,radio.length-3000);
    addOptions('rwho',j.rows.map(function(r){return r.who}));
    addOptions('rname',j.rows.map(function(r){return r.name}));
    renderRadio();
  })
}
function addOptions(id,vals){
  var sel=el(id),have={};
  Array.prototype.forEach.call(sel.options,function(o){have[o.value]=1});
  vals.forEach(function(v){if(v&&!have[v]){have[v]=1;
    var o=document.createElement('option');o.value=o.textContent=v;sel.appendChild(o)}})}
function ago(s){return s==null?'never':(s<1.5?'just now':fmt(s,0)+' s ago')}
function renderRadio(){
  var j=radioStats; if(!j)return;
  var sys=j.systems||[];
  paint('radiosys',sys.length?sys.map(function(v){
    var cls=v.heard_s==null?'':(v.heard_s<5?'ok':(v.heard_s<30?'warn':'bad'));
    return '<div class="card '+cls+'"><div class="k">'+esc(v.name)+' · system '+v.sys+'</div>'
      +'<div class="v">'+(v.heard_s==null?'not heard':'heard '+ago(v.heard_s))+'</div>'
      +'<div class="note">'+v.rx+' heard · '+v.tx+' sent'
      +(v.last?' · last '+esc(v.last):'')+'</div></div>'
  }).join(''):'<p class="note">Nothing sent or heard on the radio yet.</p>');
  var b=j.boat||{};
  if(b.sys==null)paint('radioboat','<p class="note">'+esc(b.why||'')+'</p>');
  else if(b.rate_hz==null)paint('radioboat','<p class="note">No packets from '
    +esc(b.name)+' (system '+b.sys+') yet.</p>');
  else{
    var pc=b.pct,cls=pc>=90?'ok':(pc>=60?'warn':'bad');
    paint('radioboat',
      card('Boat packets per second',fmt(b.rate_hz,2)+' <span class="note">of '
        +fmt(b.expected_hz,1)+' expected</span>',cls)
      +card('Estimated delivery',fmt(pc,0)+'%',cls)
      +card('Longest silence, last '+fmt(b.window_s,0)+' s',
        fmt(b.longest_gap_s,1)+' s',b.longest_gap_s>3?'warn':'')
      +card('Last boat packet',ago(b.heard_s)));
  }
  repaintRadio();
}
function repaintRadio(){
  var box=el('radiolog'); if(!box)return;
  var d=el('rdir').value,w=el('rwho').value,n=el('rname').value,hb=el('rhb').checked;
  var near=box.scrollTop+box.clientHeight>=box.scrollHeight-40;
  var rows=radio.filter(function(r){
    return (!d||r.dir===d)&&(!w||r.who===w)&&(!n||r.name===n)&&(hb||r.name!=='HEARTBEAT')})
    .slice(-800).map(function(r){
      return '<tr><td>'+esc(r.t)+'</td><td class="dir '+r.dir+'">'
        +(r.dir==='TX'?'SENT →':'HEARD ←')+'</td>'
        +'<td>'+esc(r.who)+' <span class="note">'+r.src+':'+r.comp+' → '+r.dst+'</span></td>'
        +'<td>'+esc(r.name)+'</td><td class="sum">'+esc(r.summary)+'</td><td>'+r.bytes+' B</td></tr>'});
  box.innerHTML='<table><thead><tr><th>time</th><th></th><th>system</th><th>message</th>'
    +'<th>contents</th><th>size</th></tr></thead><tbody>'
    +(rows.join('')||'<tr><td colspan="6" class="note">Nothing to show.</td></tr>')
    +'</tbody></table>';
  if(near)box.scrollTop=box.scrollHeight;
  el('radioinfo').textContent=radio.length+' held'
    +(radioStats&&radioStats.dropped?'  ·  '+radioStats.dropped+' dropped':'');
}
function clearRadio(){radio=[];post('/radio/clear').then(function(){repaintRadio();pollRadio()})}

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
  paint('sys',o.join(''));
  var p=S.power||{},w=s.workspace||{},h=[];
  /* The workspace either is a bind mount (fine, one line) or is not (a real
     problem, said in full). */
  h.push('<div class="pane"><div class="ph">Workspace<span class="spacer"></span>'
    +(w.persists?'<span class="spill ok">bind-mounted</span>':'<span class="spill bad">NOT a bind mount</span>')
    +'</div>'+(w.persists
      ? '<div class="note"><span class="mono">'+esc(w.source||'?')+'</span> at <span class="mono">'
        +esc(w.mount||'?')+'</span>: a <code>git pull</code> on the host is visible in here.</div>'
      : '<div style="color:var(--bad)">A <code>git pull</code> on the host is invisible to this container, and a rebuild will silently change nothing. Recreate the container with <code>-v ~/robotx_ws:/root/robotx_ws</code> (see the README).</div>')
    +'</div>');
  h.push('<div class="pane"><div class="ph">Power</div>');
  if(!p.allowed){
    h.push('<div class="locked">'+esc(p.reason||'power is disabled')+'</div>');
  }else{
    h.push('<div class="bar"><input id="pwconf" placeholder="type '+esc(s.hostname)+' to confirm" size="22">'+
      '<button class="danger" onclick="power(\'shutdown\')">Shut down</button>'+
      '<button class="danger" onclick="power(\'reboot\')">Reboot</button></div>'+
      '<div class="note" style="margin-top:6px">Refused while armed, and while the armed state is unknown.</div>');
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
    parts.push('<span class="spill warn" title="/uav/camera/status is stale">capture state unknown</span>');
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
    parts.push('<span class="spill warn">detector loading\u2026</span>');
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
    parts.push('<span class="spill'+(stills?' ok':'')+'" title="stills follow the ARM switch">stills '
      +(stills?'on':'off')+(armed?' \u00b7 armed':(r?' \u00b7 forced':''))+'</span>');
    parts.push('<span class="spill'+(r?' ok':'')+'" title="the camera\'s own 4K recording to its SD card">4K SD '
      +(r?'on':'off')+'</span>');
  }
  /* The gate is the important half: recording always runs, and a session
     heading for the bin must never be a silent surprise. */
  if(g)parts.push('<span class="spill '+(keep?'ok':'bad')+'">'+esc(g)+'</span>');
  paint('camrec',parts.join(''));
}
/* The detector's Model choice. The <select> has its OWN element, painted only
   when the list or the choice changes: the poll runs five times a second, and
   rebuilding a <select> closes it under the operator's cursor mid-pick. The
   status beside it is a separate element for the same reason. */
function setModel(p){
  post('/detector/model',{path:p}).then(function(){
    /* A refused switch leaves the server's choice unchanged, so the select's
       HTML would compare equal and never repaint -- still showing the pick
       that was refused. Forget what was painted and draw it again. */
    var e=el('cammodel');if(e)e.__html=null;poll()})}
function renderCamModel(){
  var d=S.detector;
  if(!d||!d.models||d.models.length<2){paint('cammodel','');paint('cammodelnote','');return}
  var chosen=null;
  paint('cammodel','<label class="note" style="display:flex;align-items:center;gap:6px">Model <select onchange="setModel(this.value)">'
    +d.models.map(function(m){
      if(m.path===d.chosen)chosen=m;
      return '<option value="'+esc(m.path)+'"'+(m.path===d.chosen?' selected':'')+'>'
        +esc(m.label)+'</option>'}).join('')+'</select></label>');
  var name=String(d.chosen||'').split('/').pop(),s,c;
  /* "loaded" is what the DETECTOR says it read, not what this page asked for. */
  if(!d.running){s='detector off';c='';}
  else if(!d.loaded){s='loading…';c='warn';}
  else if(d.loaded===name){s='loaded';c='ok';}
  else{s='running '+esc(d.loaded)+', not this';c='bad';}
  var tip=!d.running?'picking a model starts the detector':'';
  if(chosen&&!chosen.buoy){s+=' · no mapping';
    tip='buoy_mapper and the search are refused while this model is loaded: they would map what it finds as buoys'}
  paint('cammodelnote','<span class="spill '+c+'"'+(tip?' title="'+tip+'"':'')+'>'+s+'</span>');
}
function renderCam(){
  renderCamModel();
  renderCamRec();
  /* host: the camera and the map downloads are served by the AIRCRAFT. On its
     own page that is this page's host; on a copy running on a laptop off the
     radio (tools/scripts/gcs_radio.py) it is not, so the snapshot may name it. */
  var c=S.cam||{},host=c.host||location.hostname,box=el('cam');
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
/* One failure here used to stop the rest: an exception thrown while drawing
   left the MAP frozen on its last frame while the tiles above it kept updating,
   so the aircraft sat at its takeoff point looking parked while it flew a whole
   search. A frozen picture that still looks live is the worst kind of readout,
   so every part draws independently and any failure says so in the banner. */
var renderFails=0;
function part(name,fn){
  try{fn()}catch(e){
    renderFails++;
    var b=el('banner');
    b.textContent='page error in '+name+': '+e.message;
    b.className='bad';
    if(console&&console.error)console.error('render '+name,e)}}
function render(){
  part('vitals',renderVitals);part('preflight',renderPreflight);
  if(tab==='nodes')part('nodes',renderNodes); else if(tab==='tel')part('telemetry',renderTel);
  else if(tab==='sys')part('system',renderSys);
  if(camVisible())part('camera',renderCam);
  part('map',renderMap);
  part('layout',layoutVars);
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
    b.textContent='ground station unreachable';b.className='bad';
    /* Blanks over guesses: an unreachable ground station must not leave the last
       altitude, battery and mode on screen looking live. Read off a header that
       had stopped updating, they say the aircraft is flying when it is parked. */
    S={};render()})
}
setToggle('soundb',soundOn);
var startTab='nodes';
try{var t0=localStorage.getItem('rx26-tab');
  if(TABS.some(function(p){return p[0]===t0}))startTab=t0}catch(e){}
show(startTab);poll();setInterval(poll,POLL);attLoop();
pollLogs();setInterval(pollLogs,1000);setInterval(pollRadio,1000);
</script></body></html>
"""


def _ekko_mesh():
    """Ekko's 3D model for the Map tab, as the page's JSON, or "null" (the
    page then draws its stand-in quad) if the generated file is missing."""
    try:
        from uav_groundstation import ekko_model
    except ImportError:
        return "null"
    return json.dumps({"groups": ekko_model.GROUPS, "hex": "".join(ekko_model.DATA)})


def _logos():
    """The team logos as <img> tags, or nothing if the generated file is
    missing -- a page without its logos still flies."""
    try:
        from uav_groundstation import team_logos
    except ImportError:
        return ""
    return "".join('<img src="%s" alt="%s" title="%s">' % (uri, name, name)
                   for name, uri in team_logos.LOGOS)


def render(poll_ms: float) -> bytes:
    """The page, with the browser's poll period and Ekko's model baked in.

    Rendered once at node start rather than per request: it is a constant, and
    re-templating it on every GET would put string work on the path a laptop
    hits several times a second.
    """
    return (PAGE.replace("__POLL_MS__", str(int(poll_ms)))
            .replace("__EKKO_MESH__", _ekko_mesh())
            .replace("__LOGOS__", _logos()).encode("utf-8"))
