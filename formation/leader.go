package formation

import "math"

// Segment 是领航者路径的一段：恒定速度 V、恒定转弯率 Omega（rad/s，左转为正）。
// Omega == 0 时为直线段，否则为圆弧段。
type Segment struct {
	Dur   float64 // 段时长 (s)
	V     float64 // 恒定线速度 (m/s)
	Omega float64 // 恒定角速度 (rad/s)
}

// LeaderPath 由若干段顺序拼接而成，起点为原点、初始航向为 +x 方向。
type LeaderPath struct {
	Segs []Segment
}

// LeaderState 是领航者在某一时刻的运动状态。
type LeaderState struct {
	Pos     Vec2
	Vel     Vec2
	Heading float64 // 航向角 (rad)
	Omega   float64 // 当前转弯率 (rad/s)
}

// Duration 返回路径总时长。
func (p LeaderPath) Duration() float64 {
	d := 0.0
	for _, s := range p.Segs {
		d += s.Dur
	}
	return d
}

// State 返回 t 时刻领航者的解析状态。t 超出总时长时返回终点静止状态。
func (p LeaderPath) State(t float64) LeaderState {
	if t < 0 {
		t = 0
	}
	x, y, th := 0.0, 0.0, 0.0
	for _, s := range p.Segs {
		if t <= s.Dur {
			nx, ny, nth := advance(x, y, th, s.V, s.Omega, t)
			return LeaderState{
				Pos:     V(nx, ny),
				Vel:     V(s.V*math.Cos(nth), s.V*math.Sin(nth)),
				Heading: nth,
				Omega:   s.Omega,
			}
		}
		x, y, th = advance(x, y, th, s.V, s.Omega, s.Dur)
		t -= s.Dur
	}
	return LeaderState{Pos: V(x, y), Heading: th}
}

// advance 从位姿 (x,y,th) 以速度 v、转弯率 om 前进 dt 后的位姿（解析解）。
func advance(x, y, th, v, om, dt float64) (nx, ny, nth float64) {
	if math.Abs(om) < 1e-12 {
		return x + v*dt*math.Cos(th), y + v*dt*math.Sin(th), th
	}
	th2 := th + om*dt
	r := v / om // 有符号转弯半径
	return x + r*(math.Sin(th2)-math.Sin(th)),
		y - r*(math.Cos(th2)-math.Cos(th)),
		th2
}
