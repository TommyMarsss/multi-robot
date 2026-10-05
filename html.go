package formation

import (
	"encoding/json"
	"math"
	"os"
	"strings"
)

// ---- 序列化结构（浮点保留 4 位小数以控制文件体积）----

type jsonLog struct {
	Dt       float64      `json:"dt"`
	Duration float64      `json:"duration"`
	DSafe    float64      `json:"dsafe"`
	ErrLimit float64      `json:"errLimit"`
	Offsets  [][2]float64 `json:"offsets"`
	Frames   []jsonFrame  `json:"frames"`
}

type jsonFrame struct {
	T float64      `json:"t"`
	L [2]float64   `json:"l"`
	H float64      `json:"h"`
	P [][2]float64 `json:"p"`
	M []int        `json:"m"`
	E []float64    `json:"e"`
	D []float64    `json:"d"`
	A bool         `json:"a,omitempty"` // 避碰激活
	X bool         `json:"x,omitempty"` // 队形误差超限
}

func r4(v float64) float64 { return math.Round(v*1e4) / 1e4 }

// WriteHTML 将回放日志写成单一自包含 HTML 文件（数据与逻辑全部内嵌）。
func WriteHTML(log *Log, path string) error {
	jl := jsonLog{
		Dt:       log.Params.Dt,
		Duration: log.Duration,
		DSafe:    log.Params.DSafe,
		ErrLimit: log.Params.ErrLimit,
	}
	for _, o := range log.Offsets {
		jl.Offsets = append(jl.Offsets, [2]float64{o.X, o.Y})
	}
	for _, fr := range log.Frames {
		jf := jsonFrame{
			T: r4(fr.T),
			L: [2]float64{r4(fr.Leader.X), r4(fr.Leader.Y)},
			H: r4(fr.Heading),
			A: fr.Avoid,
			X: fr.ErrExceed,
		}
		for i := range fr.Pos {
			jf.P = append(jf.P, [2]float64{r4(fr.Pos[i].X), r4(fr.Pos[i].Y)})
			jf.M = append(jf.M, int(fr.Mode[i]))
			jf.E = append(jf.E, r4(fr.Err[i]))
			d := fr.MinDist[i]
			if math.IsInf(d, 1) {
				d = -1
			}
			jf.D = append(jf.D, r4(d))
		}
		jl.Frames = append(jl.Frames, jf)
	}
	data, err := json.Marshal(jl)
	if err != nil {
		return err
	}
	html := strings.Replace(htmlTemplate, "/*__DATA__*/", string(data), 1)
	return os.WriteFile(path, []byte(html), 0o644)
}

const htmlTemplate = `<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>多机器人编队回放</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, "PingFang SC", "Helvetica Neue", sans-serif;
         background: #14171c; color: #d7dce2; display: flex; height: 100vh; }
  #left { flex: 1; display: flex; flex-direction: column; padding: 10px; min-width: 0; }
  #cv { background: #1b2027; border-radius: 8px; flex: 1; width: 100%; }
  #bar { display: flex; align-items: center; gap: 10px; padding: 10px 4px 0; flex-wrap: wrap; }
  button, select { background: #2a313b; color: #d7dce2; border: 1px solid #3a4350;
                   border-radius: 6px; padding: 5px 14px; cursor: pointer; font-size: 13px; }
  button:hover { background: #343d4a; }
  input[type=range] { flex: 1; min-width: 160px; accent-color: #4da3ff; }
  #info { font-size: 12px; color: #9aa5b1; font-variant-numeric: tabular-nums; }
  #side { width: 300px; border-left: 1px solid #2a313b; padding: 12px; overflow-y: auto; }
  #side h2 { font-size: 14px; margin-bottom: 8px; }
  .legend { font-size: 12px; line-height: 1.9; color: #9aa5b1; margin-bottom: 12px; }
  .dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 6px; }
  #banner { display: none; padding: 8px 10px; border-radius: 6px; font-size: 13px;
            margin-bottom: 10px; font-weight: 600; }
  #banner.avoid { display: block; background: #4a1d1d; color: #ff8a8a; }
  #banner.err { display: block; background: #4a381a; color: #ffcf7a; }
  #events { list-style: none; font-size: 12px; }
  #events li { padding: 5px 8px; border-radius: 5px; cursor: pointer; margin-bottom: 3px;
               background: #1f252e; border-left: 3px solid #555; }
  #events li:hover { background: #2a313b; }
  #events li.avoid { border-left-color: #ff6b6b; }
  #events li.err { border-left-color: #ffb84d; }
  #events li small { color: #7d8894; }
</style>
</head>
<body>
<div id="left">
  <canvas id="cv"></canvas>
  <div id="bar">
    <button id="play">暂停</button>
    <input type="range" id="seek" min="0" value="0">
    <select id="speed">
      <option value="0.5">0.5×</option>
      <option value="1" selected>1×</option>
      <option value="2">2×</option>
      <option value="4">4×</option>
    </select>
    <span id="info"></span>
  </div>
</div>
<div id="side">
  <div id="banner"></div>
  <h2>图例</h2>
  <div class="legend">
    <div><span class="dot" style="background:#e8e8e8"></span>领航者（三角）</div>
    <div><span class="dot" style="background:#4da3ff"></span>跟随者 · 正常队形</div>
    <div><span class="dot" style="background:#ffb84d"></span>跟随者 · 恢复模式（大偏差渐进汇入）</div>
    <div><span class="dot" style="background:#ff6b6b"></span>跟随者 · 避碰模式（避碰优先仲裁）</div>
    <div>◇ 虚线菱形 = 路径坐标系下的期望位置</div>
    <div>橙色帧标记 = 队形误差超限；红色帧标记 = 避碰触发</div>
  </div>
  <h2>事件（点击跳转）</h2>
  <ul id="events"></ul>
</div>
<script>
const DATA = /*__DATA__*/;
const frames = DATA.frames, N = DATA.offsets.length;
const COLORS = ["#4da3ff","#3ddc97","#c792ea","#f78c6c","#ffcb6b","#5cd6ff","#ff9cac","#b5e853"];
const MODE_COLOR = ["", "#ffb84d", "#ff6b6b"]; // Normal 用本色, Recover, Avoid
const MODE_NAME = ["正常", "恢复", "避碰"];

// ---- 视图范围 ----
let minX=1e18, minY=1e18, maxX=-1e18, maxY=-1e18;
for (const f of frames) {
  const pts = [f.l].concat(f.p);
  for (const p of pts) {
    if (p[0]<minX) minX=p[0]; if (p[0]>maxX) maxX=p[0];
    if (p[1]<minY) minY=p[1]; if (p[1]>maxY) maxY=p[1];
  }
}
const cv = document.getElementById("cv"), ctx = cv.getContext("2d");
function fit() {
  cv.width = cv.clientWidth * devicePixelRatio;
  cv.height = cv.clientHeight * devicePixelRatio;
}
window.addEventListener("resize", fit); fit();

function view() {
  const m = 40 * devicePixelRatio;
  const sc = Math.min((cv.width-2*m)/Math.max(maxX-minX,1e-6),
                      (cv.height-2*m)/Math.max(maxY-minY,1e-6));
  return { sc, ox: m - minX*sc + (cv.width-2*m-(maxX-minX)*sc)/2,
           oy: m - minY*sc + (cv.height-2*m-(maxY-minY)*sc)/2 };
}
function w2s(p, v) { return [p[0]*v.sc+v.ox, cv.height-(p[1]*v.sc+v.oy)]; }

// ---- 期望位置（路径坐标系偏移 → 世界系，与控制律一致）----
function desired(leader, h, off) {
  const c=Math.cos(h), s=Math.sin(h);
  return [leader[0]+off[0]*c-off[1]*s, leader[1]+off[0]*s+off[1]*c];
}

// ---- 事件列表（避碰/超限的开始帧）----
const events = [];
for (let i=0;i<frames.length;i++) {
  const f=frames[i], prev=i>0?frames[i-1]:null;
  if (f.a && (!prev || !prev.a)) events.push({i, kind:"avoid", text:"避碰触发"});
  if (f.x && (!prev || !prev.x)) events.push({i, kind:"err", text:"队形偏差超限"});
}
const ul = document.getElementById("events");
for (const e of events) {
  const li = document.createElement("li");
  li.className = e.kind;
  li.innerHTML = e.text + " <small>t=" + frames[e.i].t.toFixed(1) + "s · 帧 " + e.i + "</small>";
  li.onclick = () => { idx = e.i; draw(); };
  ul.appendChild(li);
}

// ---- 回放控制 ----
let idx = 0, playing = true, speed = 1;
const seek = document.getElementById("seek");
seek.max = frames.length - 1;
seek.oninput = () => { idx = +seek.value; draw(); };
document.getElementById("play").onclick = function() {
  playing = !playing; this.textContent = playing ? "暂停" : "播放";
};
document.getElementById("speed").onchange = function() { speed = +this.value; };

function draw() {
  const f = frames[idx], v = view();
  ctx.clearRect(0, 0, cv.width, cv.height);

  // 轨迹（已走过的部分）
  ctx.lineWidth = 1.2 * devicePixelRatio;
  function trail(get, color) {
    ctx.strokeStyle = color; ctx.globalAlpha = 0.45; ctx.beginPath();
    for (let i=0;i<=idx;i++) {
      const q = w2s(get(frames[i]), v);
      i===0 ? ctx.moveTo(q[0],q[1]) : ctx.lineTo(q[0],q[1]);
    }
    ctx.stroke(); ctx.globalAlpha = 1;
  }
  trail(fr=>fr.l, "#888");
  for (let k=0;k<N;k++) trail(fr=>fr.p[k], COLORS[k%COLORS.length]);

  // 期望位置（虚线菱形）
  ctx.setLineDash([4,4]); ctx.lineWidth = 1 * devicePixelRatio;
  for (let k=0;k<N;k++) {
    const d = w2s(desired(f.l, f.h, DATA.offsets[k]), v);
    ctx.strokeStyle = COLORS[k%COLORS.length]; ctx.globalAlpha = 0.6;
    ctx.beginPath();
    ctx.moveTo(d[0],d[1]-6); ctx.lineTo(d[0]+6,d[1]); ctx.lineTo(d[0],d[1]+6); ctx.lineTo(d[0]-6,d[1]);
    ctx.closePath(); ctx.stroke(); ctx.globalAlpha = 1;
  }
  ctx.setLineDash([]);

  // 领航者（三角，指向航向）
  const lp = w2s(f.l, v), r = 9 * devicePixelRatio;
  ctx.fillStyle = "#e8e8e8"; ctx.beginPath();
  for (let k=0;k<3;k++) {
    const a = f.h + k*2*Math.PI/3;
    const x = lp[0]+r*Math.cos(a), y = lp[1]-r*Math.sin(a);
    k===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  }
  ctx.closePath(); ctx.fill();

  // 跟随者
  for (let k=0;k<N;k++) {
    const p = w2s(f.p[k], v);
    const mode = f.m[k];
    const color = mode>0 ? MODE_COLOR[mode] : COLORS[k%COLORS.length];
    // 安全距离圈（避碰模式下显示）
    if (mode===2) {
      ctx.strokeStyle = "rgba(255,107,107,0.5)"; ctx.lineWidth = 1*devicePixelRatio;
      ctx.beginPath(); ctx.arc(p[0],p[1], DATA.dsafe*v.sc, 0, 2*Math.PI); ctx.stroke();
    }
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(p[0],p[1], 6*devicePixelRatio, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = "#0d0f12"; ctx.font = (9*devicePixelRatio)+"px sans-serif";
    ctx.textAlign="center"; ctx.textBaseline="middle";
    ctx.fillText(String(k+1), p[0], p[1]);
  }

  // 顶部横幅与信息
  const banner = document.getElementById("banner");
  if (f.a) { banner.className="avoid"; banner.textContent="⚠ 避碰仲裁激活：避碰优先于队形保持"; }
  else if (f.x) { banner.className="err"; banner.textContent="⚠ 队形偏差超过阈值 " + DATA.errLimit + " m"; }
  else banner.className="";

  let minD = 1e18;
  for (const d of f.d) if (d>0 && d<minD) minD=d;
  document.getElementById("info").textContent =
    "t=" + f.t.toFixed(2) + "s · 帧 " + idx + "/" + (frames.length-1) +
    " · 最近间距 " + (minD<1e17 ? minD.toFixed(2)+"m" : "—") +
    " · 模式 [" + f.m.map(m=>MODE_NAME[m]).join(", ") + "]";
  if (+seek.value !== idx) seek.value = idx;
}

let last = performance.now(), acc = 0;
function tick(now) {
  const dt = (now-last)/1000; last = now;
  if (playing) {
    acc += dt*speed;
    const step = Math.floor(acc / DATA.dt);
    if (step>0) { acc -= step*DATA.dt; idx = Math.min(idx+step, frames.length-1); draw(); }
  }
  requestAnimationFrame(tick);
}
draw();
requestAnimationFrame(tick);
</script>
</body>
</html>
`
