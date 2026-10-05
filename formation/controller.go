package formation

// Params 是跟随者控制器的全部参数。
type Params struct {
	// 编队跟踪增益：vCmd = vDes + Kp·e，a = Kd·(vCmd - v)
	Kp float64 // 位置误差增益 (1/s)
	Kd float64 // 速度跟踪增益 (1/s)

	// 动力学能力限幅（输出指令永不越界）
	Amax float64 // 最大加速度 (m/s^2)
	Vmax float64 // 最大速度 (m/s)

	// 避碰：d < DAvoid 触发避让，d <= DSafe 时避让权重为 1（完全接管）
	DSafe  float64 // 最小安全中心距 (m)
	DAvoid float64 // 避让触发距离 (m)
	KAvoid float64 // 避让加速度增益
	KDamp  float64 // 相向接近速度阻尼增益

	// 恢复：误差超过 RecoverDist 进入渐进恢复，纠偏速度限幅 RecoverVel
	RecoverDist float64
	RecoverVel  float64

	ErrThresh  float64 // 队形偏差超限阈值（报告高亮用）
	Hysteresis float64 // 避碰状态退出迟滞系数（乘在 DAvoid 上，防抖动）
}

// DefaultParams 给出一组可用于轮式机器人的保守参数。
func DefaultParams() Params {
	return Params{
		Kp: 1.0, Kd: 2.0,
		Amax: 3.0, Vmax: 4.0,
		DSafe: 1.0, DAvoid: 2.0,
		KAvoid: 4.0, KDamp: 1.5,
		RecoverDist: 5.0, RecoverVel: 1.5,
		ErrThresh: 1.0, Hysteresis: 1.15,
	}
}

// Control 计算单个跟随者本步的加速度指令，并更新其避碰状态。
//
// 仲裁策略（避碰优先于队形精度）：
//  1. 编队项 aForm：带前馈的速度跟踪律，纠偏速度按恢复策略限幅；
//  2. 避碰项 aAvoid：对进入 DAvoid 的邻居施加沿连线的斥力 + 接近速度阻尼；
//  3. 混合权重 w 是最近距离的【连续】函数：d>=DAvoid 时 w=0（纯编队），
//     d<=DSafe 时 w=1（纯避碰），中间线性过渡。因为 w 连续，指令不会在两个
//     目标间跳变，从机制上避免震荡；事件标志另加迟滞，防止报告抖动；
//  4. 合成指令最后统一按 Amax 限幅，保证动力学可行。
func (f *Follower) Control(des DesiredState, all []*Follower, self int, p Params) (acc Vec2, avoid bool) {
	// ---- 编队项（含恢复限幅）----
	e := des.Pos.Sub(f.Pos)
	vCap := p.Vmax
	if e.Len() > p.RecoverDist {
		// 大偏差恢复：纠偏速度限幅，禁止比例纠偏造成的速度突变
		vCap = p.RecoverVel
	}
	vCorr := clampMag(e.Mul(p.Kp), vCap)
	vCmd := des.Vel.Add(vCorr)
	aForm := vCmd.Sub(f.Vel).Mul(p.Kd)

	// ---- 避碰项 ----
	aAvoid := Vec2{}
	w := 0.0
	minD := -1.0
	for j, o := range all {
		if j == self {
			continue
		}
		d := f.Pos.Sub(o.Pos)
		dist := d.Len()
		if minD < 0 || dist < minD {
			minD = dist
		}
		if dist >= p.DAvoid {
			continue
		}
		var n Vec2
		if dist > 1e-9 {
			n = d.Mul(1 / dist)
		} else {
			// 完全重叠的退化情形：按索引确定性地分开，避免方向未定义
			n = V(float64(self-j), float64((self+j)%3-1)).Norm()
			if n.Len() < 1e-9 {
				n = V(1, 0)
			}
			dist = 1e-9
		}
		wi := (p.DAvoid - dist) / (p.DAvoid - p.DSafe)
		if wi > 1 {
			wi = 1
		}
		closing := -f.Vel.Sub(o.Vel).Dot(n) // >0 表示正在接近
		mag := p.KAvoid * wi
		if closing > 0 {
			mag += p.KDamp * closing
		}
		aAvoid = aAvoid.Add(n.Mul(mag))
		if wi > w {
			w = wi
		}
	}

	// ---- 避碰事件标志（迟滞，仅用于报告，不影响控制连续性）----
	if f.AvoidActive {
		if minD < 0 || minD > p.DAvoid*p.Hysteresis {
			f.AvoidActive = false
		}
	} else if minD >= 0 && minD < p.DAvoid {
		f.AvoidActive = true
	}

	// ---- 优先级仲裁：连续混合 + 动力学限幅 ----
	acc = aForm.Mul(1 - w).Add(aAvoid.Mul(w))
	acc = clampMag(acc, p.Amax)
	return acc, f.AvoidActive
}
