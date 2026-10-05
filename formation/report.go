package formation

import (
	"encoding/json"
	"fmt"
	"os"
	"strings"
)

// ReportData 是嵌入 HTML 的全部回放数据。
type ReportData struct {
	Dt        float64  `json:"dt"`
	DSafe     float64  `json:"dSafe"`
	DAvoid    float64  `json:"dAvoid"`
	ErrThresh float64  `json:"errThresh"`
	Amax      float64  `json:"aMax"`
	Names     []string `json:"names"`
	Frames    []Frame  `json:"frames"`
}

// BuildReport 从仿真结果构造报告数据。
func (s *Sim) BuildReport() ReportData {
	names := make([]string, len(s.Followers))
	for i, f := range s.Followers {
		names[i] = f.Name
	}
	return ReportData{
		Dt: s.Dt, DSafe: s.P.DSafe, DAvoid: s.P.DAvoid,
		ErrThresh: s.P.ErrThresh, Amax: s.P.Amax,
		Names: names, Frames: s.Frames,
	}
}

// WriteReport 生成单一静态 HTML 文件（数据与逻辑全部内嵌，无任何外部依赖）。
func WriteReport(path string, rd ReportData) error {
	js, err := json.Marshal(rd)
	if err != nil {
		return err
	}
	html := strings.Replace(reportTemplate, "{{DATA}}", string(js), 1)
	return os.WriteFile(path, []byte(html), 0o644)
}

// String 返回报告摘要（终端打印用）。
func (rd ReportData) String() string {
	avoid, over := 0, 0
	for _, fr := range rd.Frames {
		for _, f := range fr.Followers {
			if f.Avoid {
				avoid++
			}
			if f.Over {
				over++
			}
		}
	}
	return fmt.Sprintf("frames=%d duration=%.1fs avoidFrames=%d overFrames=%d",
		len(rd.Frames), rd.Dt*float64(len(rd.Frames)), avoid, over)
}

const reportTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>多机器人编队回放</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin:0; font:14px/1.5 -apple-system,"PingFang SC","Segoe UI",sans-serif;
         background:#0f141b; color:#dbe4ee; }
  header { padding:12px 18px; border-bottom:1px solid #22303f; display:flex;
           align-items:baseline; gap:16px; flex-wrap:wrap; }
  header h1 { font-size:17px; margin:0; }
  header .stat { color:#8fa3b8; font-size:12px; }
  main { display:flex; gap:0; height:calc(100vh - 54px); }
  #stage { flex:1; position:relative; min-width:0; }
  canvas { position:absolute; inset:0; width:100%; height:100%; }
  aside { width:300px; border-left:1px solid #22303f; padding:12px; overflow-y:auto;
          display:flex; flex-direction:column; gap:12px; }
  .card { background:#161e29; border:1px solid #22303f; border-radius:8px; padding:10px 12px; }
  .card h2 { font-size:12px; text-transform:uppercase; letter-spacing:.08em;
             color:#8fa3b8; margin:0 0 8px; }
  button { background:#22303f; color:#dbe4ee; border:1px solid #31435a; border-radius:6px;
           padding:5px 12px; cursor:pointer; font-size:13px; }
  button:hover { background:#2b3d52; }
  button.on { background:#2f6fed; border-color:#2f6fed; color:#fff; }
  input[type=range] { width:100%; }
  .row { display:flex; align-items:center; gap:8px; }
  .mono { font-family:ui-monospace,Menlo,monospace; font-size:12px; }
  .legend span { display:inline-flex; align-items:center; gap:5px; margin-right:12px; font-size:12px; }
  .dot { width:10px; height:10px; border-radius:50%; display:inline-block; }
  .ring { width:10px; height:10px; border-radius:50%; border:2px solid; display:inline-block; }
  #events { max-height:220px; overflow-y:auto; font-size:12px; }
  #events div { padding:2px 4px; border-radius:4px; cursor:pointer; }
  #events div:hover { background:#22303f; }
  #events .av { color:#ff7b72; }
  #events .ov { color:#ffa657; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  td, th { text-align:left; padding:2px 4px; }
  th { color:#8fa3b8; font-weight:normal; }
</style>
</head>
<body>
<header>
  <h1>多机器人编队回放（领航-跟随）</h1>
  <span class="stat" id="meta"></span>
</header>
<main>
  <div id="stage"><canvas id="cv"></canvas></div>
  <aside>
    <div class="card">
      <h2>播放控制</h2>
      <div class="row" style="margin-bottom:8px">
        <button id="play">▶ 播放</button>
        <button data-speed="0.5">0.5×</button>
        <button data-speed="1" class="on">1×</button>
        <button data-speed="2">2×</button>
        <button data-speed="4">4×</button>
      </div>
      <input type="range" id="scrub" min="0" max="0" step="1" value="0">
      <div class="row mono" style="justify-content:space-between">
        <span id="clock"></span><span id="frameNo"></span>
      </div>
    </div>
    <div class="card legend">
      <h2>图例</h2>
      <span><i class="dot" style="background:#e6edf3"></i>领航者</span>
      <span><i class="ring" style="border-color:#ff7b72"></i>避碰触发</span>
      <span><i class="ring" style="border-color:#ffa657"></i>偏差超限</span>
      <span><i class="dot" style="background:#555;opacity:.5"></i>期望位置</span>
      <span><i class="ring" style="border-color:#31435a"></i>安全半径 dSafe/2</span>
    </div>
    <div class="card">
      <h2>跟随者状态（当前帧）</h2>
      <table id="tbl"><tr><th></th><th>偏差 m</th><th>|a| m/s²</th><th>状态</th></tr></table>
    </div>
    <div class="card" style="flex:1">
      <h2>事件（点击跳转）</h2>
      <div id="events"></div>
    </div>
  </aside>
</main>
<script>
"use strict";
var DATA = {{DATA}};
var COLORS = ["#79c0ff","#d2a8ff","#7ee787","#f778ba","#76e3ea","#e3b341"];
var cv = document.getElementById("cv"), ctx = cv.getContext("2d");
var N = DATA.frames.length, NF = DATA.frames[0].followers.length;
var idx = 0, playing = false, speed = 1, acc = 0, lastTs = 0;

// ---- 视图范围 ----
var minX=1e18,maxX=-1e18,minY=1e18,maxY=-1e18,i,j,p;
for (i=0;i<N;i++) {
  var fr = DATA.frames[i], pts = [fr.leader];
  for (j=0;j<NF;j++) { pts.push(fr.followers[j].p); pts.push(fr.followers[j].d); }
  for (j=0;j<pts.length;j++) {
    p=pts[j];
    if(p[0]<minX)minX=p[0]; if(p[0]>maxX)maxX=p[0];
    if(p[1]<minY)minY=p[1]; if(p[1]>maxY)maxY=p[1];
  }
}
var pad = Math.max(2, 0.08*Math.max(maxX-minX, maxY-minY));
minX-=pad; maxX+=pad; minY-=pad; maxY+=pad;

function fit() {
  var r = cv.getBoundingClientRect();
  cv.width = r.width * devicePixelRatio; cv.height = r.height * devicePixelRatio;
}
window.addEventListener("resize", fit);

function xform() {
  var w=cv.width, h=cv.height;
  var s = Math.min(w/(maxX-minX), h/(maxY-minY));
  var ox = (w - s*(maxX-minX))/2, oy = (h - s*(maxY-minY))/2;
  return { s:s, ox:ox, oy:oy };
}
function toPx(x, y, xf) { return [xf.ox + (x-minX)*xf.s, cv.height - xf.oy - (y-minY)*xf.s]; }

function circle(x, y, r) { ctx.beginPath(); ctx.arc(x, y, r, 0, 6.2832); }

function draw() {
  fit();
  var xf = xform(), fr = DATA.frames[idx];
  ctx.clearRect(0,0,cv.width,cv.height);
  var m = Math.max(2, 0.35*xf.s); // 1m 对应像素（半径 0.35m 的机器人）

  // 轨迹尾迹
  function trail(get, color) {
    ctx.strokeStyle = color; ctx.lineWidth = 1.2; ctx.beginPath();
    for (var k=0;k<=idx;k++) {
      var q = get(DATA.frames[k]), xy = toPx(q[0], q[1], xf);
      if (k===0) ctx.moveTo(xy[0],xy[1]); else ctx.lineTo(xy[0],xy[1]);
    }
    ctx.stroke();
  }
  trail(function(f){return f.leader;}, "rgba(230,237,243,.45)");
  for (i=0;i<NF;i++) (function(i){
    trail(function(f){return f.followers[i].p;}, COLORS[i%6] + "55");
  })(i);

  // 期望位置（灰色小叉）
  ctx.strokeStyle = "rgba(140,150,160,.6)"; ctx.lineWidth = 1;
  for (i=0;i<NF;i++) {
    var d = fr.followers[i].d, xy = toPx(d[0],d[1],xf);
    ctx.beginPath();
    ctx.moveTo(xy[0]-4,xy[1]); ctx.lineTo(xy[0]+4,xy[1]);
    ctx.moveTo(xy[0],xy[1]-4); ctx.lineTo(xy[0],xy[1]+4);
    ctx.stroke();
  }

  // 领航者（朝向三角形）
  var L = toPx(fr.leader[0], fr.leader[1], xf), hd = -fr.heading;
  ctx.save(); ctx.translate(L[0],L[1]); ctx.rotate(hd);
  ctx.fillStyle = "#e6edf3"; ctx.beginPath();
  ctx.moveTo(m*1.1,0); ctx.lineTo(-m*0.7, m*0.7); ctx.lineTo(-m*0.7,-m*0.7);
  ctx.closePath(); ctx.fill(); ctx.restore();

  // 跟随者
  for (i=0;i<NF;i++) {
    var f = fr.followers[i], xy = toPx(f.p[0], f.p[1], xf);
    // 安全半径
    ctx.strokeStyle = "rgba(49,67,90,.9)"; ctx.lineWidth = 1;
    circle(xy[0], xy[1], DATA.dSafe/2*xf.s); ctx.stroke();
    // 与期望位置的连线（偏差大时可见）
    var dxy = toPx(f.d[0], f.d[1], xf);
    ctx.strokeStyle = COLORS[i%6] + "44";
    ctx.beginPath(); ctx.moveTo(xy[0],xy[1]); ctx.lineTo(dxy[0],dxy[1]); ctx.stroke();
    // 本体
    ctx.fillStyle = COLORS[i%6];
    circle(xy[0], xy[1], m); ctx.fill();
    // 高亮
    if (f.avoid) { ctx.strokeStyle="#ff7b72"; ctx.lineWidth=2.5; circle(xy[0],xy[1],m+5); ctx.stroke(); }
    if (f.over)  { ctx.strokeStyle="#ffa657"; ctx.lineWidth=2;   circle(xy[0],xy[1],m+9); ctx.stroke(); }
    ctx.fillStyle = "#0f141b"; ctx.font = "bold " + Math.max(9,m) + "px sans-serif";
    ctx.textAlign="center"; ctx.textBaseline="middle";
    ctx.fillText(DATA.names[i], xy[0], xy[1]);
  }
  updatePanel(fr);
}

// ---- 侧栏 ----
var tbl = document.getElementById("tbl");
for (i=0;i<NF;i++) {
  var tr = tbl.insertRow();
  tr.insertCell().innerHTML = '<span class="dot" style="background:'+COLORS[i%6]+'"></span> '+DATA.names[i];
  tr.insertCell(); tr.insertCell(); tr.insertCell();
}
function updatePanel(fr) {
  for (var k=0;k<NF;k++) {
    var f = fr.followers[k], cells = tbl.rows[k+1].cells;
    cells[1].textContent = f.err.toFixed(2);
    cells[2].textContent = Math.hypot(f.a[0],f.a[1]).toFixed(2);
    cells[3].textContent = f.avoid ? "避碰" : (f.over ? "超限" : "—");
    cells[3].style.color = f.avoid ? "#ff7b72" : (f.over ? "#ffa657" : "#8fa3b8");
  }
  document.getElementById("clock").textContent = "t = " + fr.t.toFixed(2) + " s";
  document.getElementById("frameNo").textContent = (idx+1) + " / " + N;
  var sc = document.getElementById("scrub");
  if (document.activeElement !== sc) sc.value = idx;
}

// 事件（上升沿）
var ev = document.getElementById("events"), evCount = 0;
for (i=0;i<N;i++) for (j=0;j<NF;j++) {
  var cur = DATA.frames[i].followers[j];
  var prev = i>0 ? DATA.frames[i-1].followers[j] : {avoid:false, over:false};
  var kind = null;
  if (cur.avoid && !prev.avoid) kind = ["av","避碰触发"];
  else if (cur.over && !prev.over) kind = ["ov","偏差超限"];
  if (kind) (function(frame, cls, txt){
    var d = document.createElement("div");
    d.className = cls;
    d.textContent = "t=" + DATA.frames[frame].t.toFixed(2) + "s  " + DATA.names[j] + " " + txt;
    d.onclick = function(){ idx = frame; draw(); };
    ev.appendChild(d); evCount++;
  })(i, kind[0], kind[1]);
}
if (!evCount) ev.innerHTML = "<div>无事件</div>";

document.getElementById("meta").textContent =
  N + " 帧 · " + (N*DATA.dt).toFixed(1) + " s · dSafe=" + DATA.dSafe +
  "m · dAvoid=" + DATA.dAvoid + "m · 偏差阈值=" + DATA.errThresh + "m · aMax=" + DATA.aMax + "m/s²";

// ---- 播放 ----
var playBtn = document.getElementById("play");
playBtn.onclick = function(){
  playing = !playing;
  playBtn.textContent = playing ? "⏸ 暂停" : "▶ 播放";
  playBtn.className = playing ? "on" : "";
  if (playing && idx >= N-1) idx = 0;
};
document.getElementById("scrub").max = N-1;
document.getElementById("scrub").oninput = function(){ idx = +this.value; draw(); };
var spBtns = document.querySelectorAll("[data-speed]");
spBtns.forEach(function(b){
  b.onclick = function(){
    speed = +b.dataset.speed;
    spBtns.forEach(function(x){ x.className = x===b ? "on" : ""; });
  };
});
function loop(ts) {
  if (playing) {
    if (!lastTs) lastTs = ts;
    acc += (ts-lastTs)/1000 * speed;
    var step = Math.floor(acc / DATA.dt);
    if (step > 0) {
      acc -= step*DATA.dt;
      idx = Math.min(N-1, idx+step);
      if (idx >= N-1) { playing=false; playBtn.textContent="▶ 播放"; playBtn.className=""; }
      draw();
    }
  }
  lastTs = playing ? ts : 0;
  requestAnimationFrame(loop);
}
draw();
requestAnimationFrame(loop);
</script>
</body>
</html>
`
