package formation

import (
	"math"
	"testing"
)

// turnPath 构造“直线 → 连续左转 180° → 直线”的领航者路径
// （速度 2 m/s，转弯率 0.25 rad/s，转弯半径 8m）。
func turnPath() LeaderPath {
	return LeaderPath{Segs: []Segment{
		{Dur: 6, V: 2},
		{Dur: math.Pi / 0.25, V: 2, Omega: 0.25},
		{Dur: 4, V: 2},
	}}
}

func settledFollower(path LeaderPath, name string, off Offset) *Follower {
	d := Desired(path.State(0), off)
	return &Follower{Name: name, Off: off, Pos: d.Pos, Vel: d.Vel}
}

// 一、路径坐标系几何：圆弧上外侧跟随者期望半径更大、期望速度按半径比例放大。
func TestPathFrameGeometryOnCircle(t *testing.T) {
	// 领航者在 (8,0)，航向 +y，绕原点逆时针，v=2，R=8 → ω=0.25
	ls := LeaderState{Pos: V(8, 0), Vel: V(0, 2), Heading: math.Pi / 2, Omega: 0.25}

	outer := Desired(ls, Offset{Along: 0, Across: -2}) // 右侧 = 外侧
	inner := Desired(ls, Offset{Along: 0, Across: 2})  // 左侧 = 内侧（朝圆心）

	if r := outer.Pos.Len(); math.Abs(r-10) > 1e-9 {
		t.Fatalf("outer radius = %v, want 10", r)
	}
	if r := inner.Pos.Len(); math.Abs(r-6) > 1e-9 {
		t.Fatalf("inner radius = %v, want 6", r)
	}
	// 角速度一致 → 线速度 v = ω·r，内外侧天然差异化
	if v := outer.Vel.Len(); math.Abs(v-2.5) > 1e-9 {
		t.Fatalf("outer speed = %v, want 2.5", v)
	}
	if v := inner.Vel.Len(); math.Abs(v-1.5) > 1e-9 {
		t.Fatalf("inner speed = %v, want 1.5", v)
	}
}

// 一、连续转弯场景：队形几何形变被正确处理 ——
// 跟踪误差有界、外侧跟随者比内侧快、横向间距保持恒定。
func TestContinuousTurnFormation(t *testing.T) {
	path := turnPath()
	p := DefaultParams()
	sim := &Sim{Path: path, P: p, Dt: 0.02, Followers: []*Follower{
		settledFollower(path, "inner", Offset{Along: 0, Across: 2}),
		settledFollower(path, "outer", Offset{Along: 0, Across: -2}),
	}}
	sim.Run()

	// 转弯中段（弧段为 t ∈ [6, 6+4π≈18.6]）
	maxErr, minSep, maxSep := 0.0, math.Inf(1), 0.0
	sumVIn, sumVOut, nMid := 0.0, 0.0, 0
	for _, fr := range sim.Frames {
		if fr.T < 8 || fr.T > 16 {
			continue
		}
		for _, f := range fr.Followers {
			if f.Err > maxErr {
				maxErr = f.Err
			}
		}
		sep := math.Hypot(fr.Followers[0].P[0]-fr.Followers[1].P[0],
			fr.Followers[0].P[1]-fr.Followers[1].P[1])
		if sep < minSep {
			minSep = sep
		}
		if sep > maxSep {
			maxSep = sep
		}
		if fr.T >= 10 && fr.T <= 15 {
			sumVIn += math.Hypot(fr.Followers[0].V[0], fr.Followers[0].V[1])
			sumVOut += math.Hypot(fr.Followers[1].V[0], fr.Followers[1].V[1])
			nMid++
		}
	}
	if maxErr > 0.4 {
		t.Fatalf("max tracking error in turn = %.3f, want < 0.4", maxErr)
	}
	if minSep < 3.5 || maxSep > 4.5 {
		t.Fatalf("lateral separation in turn ∈ [%.3f, %.3f], want ≈4", minSep, maxSep)
	}

	// 几何形变的直接证据：转弯稳态下外侧速度 ≈ ω(R+2)=2.5，内侧 ≈ ω(R-2)=1.5
	vIn, vOut := sumVIn/float64(nMid), sumVOut/float64(nMid)
	if vOut <= vIn+0.5 {
		t.Fatalf("outer speed %.3f should exceed inner %.3f by >0.5", vOut, vIn)
	}
	if math.Abs(vOut-2.5) > 0.3 || math.Abs(vIn-1.5) > 0.3 {
		t.Fatalf("speeds inner=%.3f outer=%.3f, want ≈1.5 / ≈2.5", vIn, vOut)
	}
}

// 二、避碰与队形冲突：期望间距 < dSafe 的过密队形。
// 仲裁结果必须满足：不违反硬安全距离、避碰事件触发、状态不抖动、加速度不超限。
func TestAvoidanceArbitration(t *testing.T) {
	path := LeaderPath{Segs: []Segment{{Dur: 20, V: 2}}}
	p := DefaultParams() // dSafe=1.0, dAvoid=2.0, Amax=3.0
	// 期望横向间距 0.8m < dSafe=1.0m —— 队形目标本身违反安全距离。
	// 初始放在安全距离上（±1.0m），编队目标会把它们拉向过密的期望位置，
	// 避碰仲裁必须阻止硬安全距离被违反。
	a := settledFollower(path, "A", Offset{Along: 0, Across: 0.4})
	b := settledFollower(path, "B", Offset{Along: 0, Across: -0.4})
	a.Pos = V(0, 1.0)
	b.Pos = V(0, -1.0)
	sim := &Sim{Path: path, P: p, Dt: 0.02, Followers: []*Follower{a, b}}
	sim.Run()

	if d := sim.MinPairDistance(); d < 0.9*p.DSafe {
		t.Fatalf("min pair distance %.3f violates hard safety margin %.3f", d, 0.9*p.DSafe)
	}

	// 加速度指令全程不超限（帧数据有 0.001 量化，留舍入裕量）
	for _, fr := range sim.Frames {
		for _, f := range fr.Followers {
			if m := math.Hypot(f.A[0], f.A[1]); m > p.Amax+0.002 {
				t.Fatalf("t=%.2f accel %.4f exceeds Amax %.2f", fr.T, m, p.Amax)
			}
		}
	}

	// 避碰确实触发，且状态不抖动（触发/退出转换次数有界）
	transitions := [2]int{}
	avoidSeen := false
	prev := [2]bool{}
	for _, fr := range sim.Frames {
		for i := 0; i < 2; i++ {
			cur := fr.Followers[i].Avoid
			if cur {
				avoidSeen = true
			}
			if cur != prev[i] {
				transitions[i]++
			}
			prev[i] = cur
		}
	}
	if !avoidSeen {
		t.Fatal("avoidance never triggered despite infeasible formation")
	}
	for i, tr := range transitions {
		if tr > 4 {
			t.Fatalf("follower %d avoidance toggled %d times (oscillation), want <=4", i, tr)
		}
	}

	// 稳态：仲裁后停在 [dSafe, dAvoid] 附近的折中距离，而不是在两目标间震荡
	last := sim.Frames[len(sim.Frames)-1]
	sep := math.Hypot(last.Followers[0].P[0]-last.Followers[1].P[0],
		last.Followers[0].P[1]-last.Followers[1].P[1])
	if sep < p.DSafe*0.95 || sep > p.DAvoid*1.2 {
		t.Fatalf("steady-state separation %.3f, want within [%.2f, %.2f]",
			sep, p.DSafe*0.95, p.DAvoid*1.2)
	}
}

// 三、大偏差恢复：加速度全程不超限、速度无瞬时跳变、最终收敛回队形。
func TestRecoveryBoundedAcceleration(t *testing.T) {
	path := LeaderPath{Segs: []Segment{{Dur: 40, V: 2}}}
	p := DefaultParams() // RecoverDist=5, RecoverVel=1.5, Amax=3, Vmax=4
	f := settledFollower(path, "R", Offset{Along: -3, Across: 1})
	f.Pos = f.Pos.Add(V(12, -16)) // 偏离期望位置 20m
	f.Vel = V(0, 0)
	dt := 0.02
	sim := &Sim{Path: path, P: p, Dt: dt, Followers: []*Follower{f}}

	maxDv, maxExcess, maxAcc := 0.0, 0.0, 0.0
	prev := f.Vel
	for tm := 0.0; tm <= 40; tm += dt {
		sim.Step(tm)
		if dv := f.Vel.Sub(prev).Len(); dv > maxDv {
			maxDv = dv
		}
		prev = f.Vel
		if a := f.LastAcc.Len(); a > maxAcc {
			maxAcc = a
		}
		// 恢复阶段（误差 > RecoverDist）纠偏速度限幅：
		// 实际速度相对期望（前馈）速度的超出部分不得超过 RecoverVel
		des := Desired(path.State(tm), f.Off)
		if e := f.Pos.Sub(des.Pos).Len(); e > p.RecoverDist {
			if ex := f.Vel.Sub(des.Vel).Len(); ex > maxExcess {
				maxExcess = ex
			}
		}
	}

	// 加速度指令全程不超限（精确值，非帧量化数据）
	if maxAcc > p.Amax+1e-9 {
		t.Fatalf("max accel %.4f exceeds Amax %.2f", maxAcc, p.Amax)
	}
	// 速度无瞬时突变：|Δv| <= Amax·dt
	if maxDv > p.Amax*dt+1e-9 {
		t.Fatalf("max velocity jump %.4f exceeds Amax*dt %.4f", maxDv, p.Amax*dt)
	}
	// 恢复阶段纠偏速度限幅（含数值裕量）
	if maxExcess > p.RecoverVel+0.6 {
		t.Fatalf("corrective speed during recovery %.3f exceeds RecoverVel %.2f",
			maxExcess, p.RecoverVel)
	}
	// 最终收敛回队形
	last := sim.Frames[len(sim.Frames)-1].Followers[0]
	if last.Err > 0.3 {
		t.Fatalf("final error %.3f, want < 0.3 (recovery did not converge)", last.Err)
	}
}
