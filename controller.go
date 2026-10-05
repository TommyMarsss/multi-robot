package formation

import "math"

// Mode 跟随者当前控制模式（优先级：Avoid > Recover > Normal）。
type Mode int

const (
	ModeNormal  Mode = iota // 正常队形跟踪
	ModeRecover             // 大偏差渐进恢复
	ModeAvoid               // 避碰优先
)

// Params 控制器与仲裁参数。
type Params struct {
	Dt float64 // 控制周期（秒）

	// 动力学限幅（硬约束，任何模式下都不得突破）
	AMax float64 // 最大加速度 (m/s^2)
	VMax float64 // 最大速度 (m/s)

	// 队形跟踪（Normal 模式 PD + 前馈）
	KP float64 // 位置误差增益
	KV float64 // 速度误差增益

	// 避碰（Avoid 模式）
	DSafe      float64 // 最小安全距离（米）
	AvoidEnter float64 // 进入避碰的距离阈值（滞回内沿）
	AvoidExit  float64 // 退出避碰的距离阈值（滞回外沿，> AvoidEnter）
	KAvoid     float64 // 避碰加速度增益
	// AvoidFormWeight 避碰激活时队形项的权重（<1，避碰主导）
	AvoidFormWeight float64

	// 恢复（Recover 模式）
	RecoverEnter float64 // 进入恢复的队形误差阈值（米）
	RecoverExit  float64 // 退出恢复的队形误差阈值（滞回，< RecoverEnter）
	RecoverSpeed float64 // 恢复纠偏速度上限（米/秒），防止瞬时速度突变
	KRec         float64 // 恢复纠偏比例增益
	KRecV        float64 // 恢复速度跟踪增益

	ErrLimit float64 // 队形误差超限报警阈值（米）
}

// DefaultParams 返回一组经过整定的默认参数。
func DefaultParams() Params {
	return Params{
		Dt:              0.02,
		AMax:            2.0,
		VMax:            2.5,
		KP:              2.0,
		KV:              2.5,
		DSafe:           1.0,
		AvoidEnter:      1.6,
		AvoidExit:       2.0,
		KAvoid:          6.0,
		AvoidFormWeight: 0.25,
		RecoverEnter:    2.0,
		RecoverExit:     0.5,
		RecoverSpeed:    1.0,
		KRec:            0.8,
		KRecV:           3.0,
		ErrLimit:        1.2,
	}
}

// Follower 一个跟随者机器人。Offset 定义在路径坐标系：
// Offset.X = 切向偏移（沿领航者航向，前正后负），
// Offset.Y = 法向偏移（航向左侧为正）。
type Follower struct {
	ID     int
	Offset Vec
	Pos    Vec
	Vel    Vec
	Accel  Vec // 上一步实际执行的加速度（已限幅）

	Mode    Mode
	Err     float64 // 当前队形误差（米）
	MinDist float64 // 与最近邻居的距离（米）

	avoiding   bool // 避碰滞回状态
	recovering bool // 恢复滞回状态
}

// NewFollower 创建跟随者。
func NewFollower(id int, offset, start Vec) *Follower {
	return &Follower{ID: id, Offset: offset, Pos: start, MinDist: math.Inf(1)}
}

// Desired 由领航者状态计算机体在路径坐标系下的期望位置与期望速度前馈。
//
// 坐标变换（关键：偏移定义在随领航者航向旋转的路径坐标系，而非世界系）：
//
//	t = (cosθ, sinθ)        切向
//	n = (-sinθ, cosθ)       左法向
//	p_des = p_L + s·t + d·n
//
// 转弯修正：领航者以曲率 κ 转弯时，法向偏移 d 的跟随者走的是半径
// R-d 的同心圆弧，其速度必须与领航者成比例缩放，否则内外侧速度
// 不一致导致队形剪切甚至碰撞。对 p_des 沿路径求导得前馈速度：
//
//	v_des = v·(1 - κ·d)·t + v·κ·s·n
//
// 其中 (1 - κ·d) 即内外侧速比修正项，v·κ·s 为切向偏移带来的侧滑分量。
func (f *Follower) Desired(l LeaderState) (pDes, vDes Vec) {
	sn, cs := math.Sin(l.Heading), math.Cos(l.Heading)
	t := Vec{cs, sn}
	n := Vec{-sn, cs}
	s, d := f.Offset.X, f.Offset.Y
	pDes = l.Pos.Add(t.Scale(s)).Add(n.Scale(d))
	vDes = t.Scale(l.Speed * (1 - l.Curv*d)).Add(n.Scale(l.Speed * l.Curv * s))
	return pDes, vDes
}

// Step 执行一个控制周期。neighbors 为其他机器人的位置。
// 仲裁优先级（高→低）：避碰 Avoid > 恢复 Recover > 队形 Normal。
// 任何模式下输出加速度都被限幅到 AMax、速度限幅到 VMax，
// 保证不输出超出动力学能力的指令。
func (f *Follower) Step(l LeaderState, neighbors []Vec, p Params) {
	pDes, vDes := f.Desired(l)
	errV := pDes.Sub(f.Pos)
	err := errV.Len()
	f.Err = err

	// ---- 恢复模式滞回 ----
	if !f.recovering && err > p.RecoverEnter {
		f.recovering = true
	}
	if f.recovering && err < p.RecoverExit {
		f.recovering = false
	}

	// ---- 避碰向量与最近距离 ----
	avoid := Vec{}
	minD := math.Inf(1)
	for _, o := range neighbors {
		d := f.Pos.Dist(o)
		if d < minD {
			minD = d
		}
		if d < p.AvoidEnter && d > 1e-9 {
			// 沿远离邻居方向的斥力，距离越近越大
			avoid = avoid.Add(f.Pos.Sub(o).Scale(p.KAvoid * (p.AvoidEnter - d) / d))
		}
	}
	f.MinDist = minD

	// ---- 避碰滞回 ----
	if !f.avoiding && minD < p.AvoidEnter {
		f.avoiding = true
	}
	if f.avoiding && minD > p.AvoidExit {
		f.avoiding = false
	}

	// ---- 队形/恢复加速度 ----
	var aForm Vec
	if f.recovering {
		// 渐进恢复：纠偏速度限幅到 RecoverSpeed，避免比例纠偏
		// 在大误差下产生瞬时速度突变；目标速度再限幅到 VMax。
		vCorr := clampLen(errV.Scale(p.KRec), p.RecoverSpeed)
		vTarget := clampLen(vDes.Add(vCorr), p.VMax)
		aForm = vTarget.Sub(f.Vel).Scale(p.KRecV)
	} else {
		aForm = errV.Scale(p.KP).Add(vDes.Sub(f.Vel).Scale(p.KV))
	}

	// ---- 仲裁 ----
	var a Vec
	switch {
	case f.avoiding:
		// 避碰优先：避碰加速度全量 + 降权队形项，防止两个目标
		// 等权叠加导致震荡；滞回带避免模式来回抖动。
		a = avoid.Add(aForm.Scale(p.AvoidFormWeight))
		f.Mode = ModeAvoid
	case f.recovering:
		a = aForm
		f.Mode = ModeRecover
	default:
		a = aForm
		f.Mode = ModeNormal
	}

	// ---- 动力学硬限幅 ----
	a = clampLen(a, p.AMax)
	f.Accel = a
	f.Vel = clampLen(f.Vel.Add(a.Scale(p.Dt)), p.VMax)
	f.Pos = f.Pos.Add(f.Vel.Scale(p.Dt))
}
