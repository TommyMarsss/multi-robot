package formation

import "math"

// Follower 是一个跟随者机器人（双积分器动力学 + 速度限幅）。
type Follower struct {
	Name        string
	Off         Offset // 路径坐标系期望偏移
	Pos, Vel    Vec2
	AvoidActive bool // 避碰状态（带迟滞）
	LastAcc     Vec2 // 上一步输出的加速度指令
}

// FollowerFrame 是单个跟随者在一帧中的记录。
type FollowerFrame struct {
	P     [2]float64 `json:"p"`     // 实际位置
	V     [2]float64 `json:"v"`     // 实际速度
	D     [2]float64 `json:"d"`     // 期望位置
	A     [2]float64 `json:"a"`     // 加速度指令
	Err   float64    `json:"err"`   // 队形偏差
	Avoid bool       `json:"avoid"` // 避碰触发
	Over  bool       `json:"over"`  // 偏差超限
}

// Frame 是一帧仿真记录。
type Frame struct {
	T         float64         `json:"t"`
	Leader    [2]float64      `json:"leader"`
	Heading   float64         `json:"heading"`
	Followers []FollowerFrame `json:"followers"`
}

// Sim 是编队仿真器。
type Sim struct {
	Path      LeaderPath
	P         Params
	Dt        float64
	Followers []*Follower
	Frames    []Frame
}

// NewSim 创建仿真器；starts[i] 为各跟随者初始位置（nil 表示放在期望位置）。
func (s *Sim) Step(t float64) {
	ls := s.Path.State(t)
	accs := make([]Vec2, len(s.Followers))
	avoids := make([]bool, len(s.Followers))
	dess := make([]DesiredState, len(s.Followers))
	for i, f := range s.Followers {
		dess[i] = Desired(ls, f.Off)
		accs[i], avoids[i] = f.Control(dess[i], s.Followers, i, s.P)
	}
	// 同步积分（所有控制量都基于步初状态，避免更新顺序引入偏差）
	for i, f := range s.Followers {
		f.LastAcc = accs[i]
		f.Vel = clampMag(f.Vel.Add(accs[i].Mul(s.Dt)), s.P.Vmax)
		f.Pos = f.Pos.Add(f.Vel.Mul(s.Dt))
	}
	// 记录帧
	fr := Frame{
		T:         r3(t),
		Leader:    [2]float64{r3(ls.Pos.X), r3(ls.Pos.Y)},
		Heading:   r3(ls.Heading),
		Followers: make([]FollowerFrame, len(s.Followers)),
	}
	for i, f := range s.Followers {
		err := f.Pos.Sub(dess[i].Pos).Len()
		fr.Followers[i] = FollowerFrame{
			P:     [2]float64{r3(f.Pos.X), r3(f.Pos.Y)},
			V:     [2]float64{r3(f.Vel.X), r3(f.Vel.Y)},
			D:     [2]float64{r3(dess[i].Pos.X), r3(dess[i].Pos.Y)},
			A:     [2]float64{r3(accs[i].X), r3(accs[i].Y)},
			Err:   r3(err),
			Avoid: avoids[i],
			Over:  err > s.P.ErrThresh,
		}
	}
	s.Frames = append(s.Frames, fr)
}

// Run 从 t=0 运行到路径结束。
func (s *Sim) Run() {
	for t := 0.0; t <= s.Path.Duration()+1e-9; t += s.Dt {
		s.Step(t)
	}
}

// MinPairDistance 返回所有帧中跟随者间的最小距离。
func (s *Sim) MinPairDistance() float64 {
	m := math.Inf(1)
	for _, fr := range s.Frames {
		for i := 0; i < len(fr.Followers); i++ {
			for j := i + 1; j < len(fr.Followers); j++ {
				d := math.Hypot(fr.Followers[i].P[0]-fr.Followers[j].P[0],
					fr.Followers[i].P[1]-fr.Followers[j].P[1])
				if d < m {
					m = d
				}
			}
		}
	}
	return m
}

func r3(x float64) float64 { return math.Round(x*1000) / 1000 }
