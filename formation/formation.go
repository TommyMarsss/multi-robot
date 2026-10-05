package formation

import "math"

// Offset 定义跟随者在【路径坐标系】（Frenet 系）下的期望偏移：
//
//	Along  —— 沿领航者航向的切向偏移（前为正）
//	Across —— 垂直于航向的法向偏移（左为正）
//
// 关键：偏移不是世界系下的固定笛卡尔向量，而是随领航者航向旋转的。
// 转弯时内侧/外侧跟随者的期望速度会按转弯半径自动差异化（见 Desired），
// 从而避免内外侧速度不一致导致的队形拉扯与相互碰撞。
type Offset struct {
	Along, Across float64
}

// DesiredState 是跟随者的期望位置与期望速度（前馈）。
type DesiredState struct {
	Pos Vec2
	Vel Vec2
}

// Desired 将路径坐标系偏移变换到世界系。
//
// 位置：p = p_L + R(θ)·d，其中 R(θ) 为航向旋转矩阵，d=(Along,Across)。
// 速度：对位置求导，v = v_L + ω·R(θ)·J·d，J 为 90° 旋转算子 J(a,b)=(-b,a)。
// 速度项 ω·R(θ)·J·d 即转弯时的偏移修正：法向偏移越靠外侧，期望线速度越大，
// 与领航者保持相同的角速度 ω（几何上即绕同一瞬时转弯中心运动）。
func Desired(ls LeaderState, off Offset) DesiredState {
	c, s := math.Cos(ls.Heading), math.Sin(ls.Heading)
	// R(θ)·d
	px := c*off.Along - s*off.Across
	py := s*off.Along + c*off.Across
	// R(θ)·J·d
	jx, jy := -off.Across, off.Along
	vx := c*jx - s*jy
	vy := s*jx + c*jy
	return DesiredState{
		Pos: ls.Pos.Add(V(px, py)),
		Vel: ls.Vel.Add(V(vx*ls.Omega, vy*ls.Omega)),
	}
}
