package formation

import "math"

// Seg 路径段：直线或圆弧（Angle 正为左转，负为右转）。
type Seg struct {
	Arc           bool
	Length        float64 // 直线长度（米）
	Radius, Angle float64 // 圆弧半径与转角（弧度，带符号）
}

// Line 构造直线段。
func Line(length float64) Seg { return Seg{Length: length} }

// Arc 构造圆弧段，angle 带符号（正=左转）。
func Arc(radius, angle float64) Seg { return Seg{Arc: true, Radius: radius, Angle: angle} }

// LeaderState 领航者在某时刻的运动状态。
type LeaderState struct {
	Pos     Vec
	Heading float64 // 航向角（弧度）
	Curv    float64 // 曲率 κ = 1/R，左转正、右转负、直线为 0
	Speed   float64 // 速率（标量，沿切向）
}

type compiledSeg struct {
	Seg
	s0         float64 // 该段起点的累计弧长
	x0, y0, th float64 // 该段起点位姿
}

// Path 领航者路径：匀速沿分段线/弧路径行驶。
type Path struct {
	Speed float64
	segs  []compiledSeg
	total float64 // 总弧长
}

// NewPath 编译路径：从 start、heading 出发依次拼接各段。
func NewPath(speed float64, start Vec, heading float64, segs ...Seg) *Path {
	p := &Path{Speed: speed}
	x, y, th := start.X, start.Y, heading
	var s0 float64
	for _, sg := range segs {
		cs := compiledSeg{Seg: sg, s0: s0, x0: x, y0: y, th: th}
		p.segs = append(p.segs, cs)
		if sg.Arc {
			sign := 1.0
			if sg.Angle < 0 {
				sign = -1
			}
			// 圆心 = 起点 + sign*R*左法向
			nx, ny := -math.Sin(th), math.Cos(th)
			cx := x + sign*sg.Radius*nx
			cy := y + sign*sg.Radius*ny
			phi1 := th - sign*math.Pi/2 + sg.Angle
			x = cx + sg.Radius*math.Cos(phi1)
			y = cy + sg.Radius*math.Sin(phi1)
			th += sg.Angle
			s0 += sg.Radius * math.Abs(sg.Angle)
		} else {
			x += sg.Length * math.Cos(th)
			y += sg.Length * math.Sin(th)
			s0 += sg.Length
		}
	}
	p.total = s0
	return p
}

// Duration 全程耗时（秒）。
func (p *Path) Duration() float64 { return p.total / p.Speed }

// State 返回 t 时刻领航者状态（t 超出全程则停在终点）。
func (p *Path) State(t float64) LeaderState {
	s := p.Speed * t
	if s < 0 {
		s = 0
	}
	if s > p.total {
		s = p.total
	}
	cs := p.segs[len(p.segs)-1]
	for _, c := range p.segs {
		segLen := c.Length
		if c.Arc {
			segLen = c.Radius * math.Abs(c.Angle)
		}
		if s <= c.s0+segLen {
			cs = c
			break
		}
	}
	si := s - cs.s0 // 段内弧长
	if !cs.Arc {
		return LeaderState{
			Pos:     Vec{cs.x0 + si*math.Cos(cs.th), cs.y0 + si*math.Sin(cs.th)},
			Heading: cs.th,
			Curv:    0,
			Speed:   p.Speed,
		}
	}
	sign := 1.0
	if cs.Angle < 0 {
		sign = -1
	}
	nx, ny := -math.Sin(cs.th), math.Cos(cs.th)
	cx := cs.x0 + sign*cs.Radius*nx
	cy := cs.y0 + sign*cs.Radius*ny
	phi := cs.th - sign*math.Pi/2 + sign*si/cs.Radius
	return LeaderState{
		Pos:     Vec{cx + cs.Radius*math.Cos(phi), cy + cs.Radius*math.Sin(phi)},
		Heading: cs.th + sign*si/cs.Radius,
		Curv:    sign / cs.Radius,
		Speed:   p.Speed,
	}
}
