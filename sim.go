package formation

// Fault 注入的扰动：在时刻 T 将跟随者 ID 位移 Delta（模拟绕障/故障后的大偏差）。
type Fault struct {
	T     float64
	ID    int
	Delta Vec
}

// Sim 编队仿真器。
type Sim struct {
	Path      *Path
	Followers []*Follower
	Params    Params
	TEnd      float64
	Faults    []Fault // 按时间升序
}

// Frame 一帧快照。
type Frame struct {
	T         float64
	Leader    Vec
	Heading   float64
	Pos       []Vec
	Mode      []Mode
	Err       []float64
	MinDist   []float64
	Accel     []Vec
	Avoid     bool // 本帧存在避碰激活
	ErrExceed bool // 本帧存在队形误差超限
}

// Log 完整回放日志。
type Log struct {
	Frames   []Frame
	Offsets  []Vec
	Params   Params
	Duration float64
}

// Run 运行仿真并返回日志。
func (s *Sim) Run() *Log {
	p := s.Params
	log := &Log{Params: p, Duration: s.TEnd}
	for _, f := range s.Followers {
		log.Offsets = append(log.Offsets, f.Offset)
	}
	steps := int(s.TEnd/p.Dt) + 1
	faultIdx := 0
	for k := 0; k < steps; k++ {
		t := float64(k) * p.Dt
		for faultIdx < len(s.Faults) && s.Faults[faultIdx].T <= t {
			fa := s.Faults[faultIdx]
			s.Followers[fa.ID].Pos = s.Followers[fa.ID].Pos.Add(fa.Delta)
			faultIdx++
		}
		ls := s.Path.State(t)

		// 记录当前帧
		fr := Frame{T: t, Leader: ls.Pos, Heading: ls.Heading}
		for _, f := range s.Followers {
			fr.Pos = append(fr.Pos, f.Pos)
			fr.Mode = append(fr.Mode, f.Mode)
			fr.Err = append(fr.Err, f.Err)
			fr.MinDist = append(fr.MinDist, f.MinDist)
			fr.Accel = append(fr.Accel, f.Accel)
			if f.Mode == ModeAvoid {
				fr.Avoid = true
			}
			if f.Err > p.ErrLimit {
				fr.ErrExceed = true
			}
		}
		log.Frames = append(log.Frames, fr)

		// 推进一个控制周期（邻居位置取本周起期初快照，保证对称）
		snap := make([]Vec, len(s.Followers))
		for i, f := range s.Followers {
			snap[i] = f.Pos
		}
		for i, f := range s.Followers {
			others := make([]Vec, 0, len(snap)-1)
			for j, o := range snap {
				if j != i {
					others = append(others, o)
				}
			}
			f.Step(ls, others, p)
		}
	}
	return log
}
