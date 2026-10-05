package formation

import (
	"math"
	"testing"
)

// 在期望位形上初始化跟随者（位置=期望位置，速度=前馈速度），消除起步瞬态。
func initFollowers(path *Path, offsets []Vec) []*Follower {
	ls := path.State(0)
	var fs []*Follower
	for i, off := range offsets {
		f := NewFollower(i, off, Vec{})
		pDes, vDes := f.Desired(ls)
		f.Pos, f.Vel = pDes, vDes
		fs = append(fs, f)
	}
	return fs
}

func pairMinDist(log *Log, i, j int) float64 {
	m := math.Inf(1)
	for _, fr := range log.Frames {
		d := fr.Pos[i].Dist(fr.Pos[j])
		if d < m {
			m = d
		}
	}
	return m
}

func speeds(log *Log, i int) []float64 {
	var out []float64
	dt := log.Params.Dt
	for k := 1; k < len(log.Frames); k++ {
		d := log.Frames[k].Pos[i].Dist(log.Frames[k-1].Pos[i])
		out = append(out, d/dt)
	}
	return out
}

// TestDesiredFrameTransform 验证路径坐标系 → 世界系的期望位姿变换与转弯速比修正。
func TestDesiredFrameTransform(t *testing.T) {
	// 航向 90°（朝 +Y），切向偏移 2、法向偏移 1（左侧）
	f := NewFollower(0, Vec{2, 1}, Vec{})
	pDes, _ := f.Desired(LeaderState{Pos: Vec{0, 0}, Heading: math.Pi / 2, Speed: 1})
	// t=(0,1), n=(-1,0) → p_des = (0,0)+2*(0,1)+1*(-1,0) = (-1, 2)
	if math.Abs(pDes.X-(-1)) > 1e-9 || math.Abs(pDes.Y-2) > 1e-9 {
		t.Fatalf("期望位置错误: got %v, want (-1, 2)", pDes)
	}
	// 左转 κ=0.1，法向偏移 d=2（内侧）→ 切向速度应为 v(1-κd)=0.8v
	f2 := NewFollower(1, Vec{2, 2}, Vec{})
	_, vDes := f2.Desired(LeaderState{Heading: 0, Curv: 0.1, Speed: 1})
	if math.Abs(vDes.X-0.8) > 1e-9 {
		t.Fatalf("转弯速比修正错误: got vx=%v, want 0.8", vDes.X)
	}
	// 切向偏移 s=2 在转弯时产生法向前馈分量 v·κ·s = 0.2
	if math.Abs(vDes.Y-0.2) > 1e-9 {
		t.Fatalf("切向偏移侧滑分量错误: got vy=%v, want 0.2", vDes.Y)
	}
}

// TestTurnGeometryDeformation 领航者连续转弯时：
// 1) 队形误差有界（路径坐标系偏移被正确保持，而非世界系固定偏移）；
// 2) 外侧跟随者平均速度大于内侧（同心弧几何）；
// 3) 任意两机间距始终不小于安全距离。
func TestTurnGeometryDeformation(t *testing.T) {
	p := DefaultParams()
	p.Dt = 0.02
	path := NewPath(1.2, Vec{0, 0}, 0,
		Line(12), Arc(8, math.Pi/2), Line(6), Arc(6, -math.Pi/2), Line(10))
	offsets := []Vec{{-1.5, 1.5}, {-1.5, -1.5}, {-3, 0}}
	fs := initFollowers(path, offsets)
	sim := &Sim{Path: path, Followers: fs, Params: p, TEnd: path.Duration()}
	log := sim.Run()

	// 1) 队形误差有界
	maxErr := 0.0
	for _, fr := range log.Frames {
		for _, e := range fr.Err {
			if e > maxErr {
				maxErr = e
			}
		}
	}
	if maxErr > 0.6 {
		t.Fatalf("转弯过程队形误差过大: max=%.3f m（路径坐标系形变未被正确处理）", maxErr)
	}

	// 2) 左转弧段（t ∈ [10, 10+8·π/2/1.2]）：d=+1.5 为内侧应更慢，d=-1.5 外侧应更快
	t0, t1 := 12/1.2, (12+8*math.Pi/2)/1.2
	avg := func(i int) float64 {
		sp := speeds(log, i)
		var sum float64
		var n int
		for k, fr := range log.Frames[1:] {
			if fr.T >= t0 && fr.T <= t1 {
				sum += sp[k]
				n++
			}
		}
		return sum / float64(n)
	}
	inner, outer := avg(0), avg(1)
	if outer <= inner+0.05 {
		t.Fatalf("转弯内外侧速比错误: 外侧 %.3f 应明显大于内侧 %.3f", outer, inner)
	}

	// 3) 全程无碰撞
	for i := 0; i < len(offsets); i++ {
		for j := i + 1; j < len(offsets); j++ {
			if d := pairMinDist(log, i, j); d < p.DSafe {
				t.Fatalf("跟随者 %d 与 %d 最小间距 %.3f < 安全距离 %.3f", i, j, d, p.DSafe)
			}
		}
	}
}

// TestConflictArbitrationPriority 队形期望位置本身违反最小安全距离时：
// 1) 避碰优先：实际间距不得跌到安全距离以下；
// 2) 间距小于安全距离时仲裁必须处于避碰模式；
// 3) 仲裁不震荡（滞回使模式切换次数有界）；
// 4) 加速度指令不超限。
func TestConflictArbitrationPriority(t *testing.T) {
	p := DefaultParams()
	p.Dt = 0.02
	path := NewPath(1.0, Vec{0, 0}, 0, Line(30))
	// 期望间距仅 0.6m < DSafe=1.0m：队形目标与避碰约束直接冲突
	offsets := []Vec{{-1.5, 0.3}, {-1.5, -0.3}}
	fs := initFollowers(path, offsets)
	sim := &Sim{Path: path, Followers: fs, Params: p, TEnd: 30}
	log := sim.Run()

	// 初始期望间距 0.6m 本身违反安全距离；控制介入后必须分离开。
	// 全程不得进一步接近（无碰撞），且 3s 后间距稳定在安全距离以上。
	minAll := pairMinDist(log, 0, 1)
	if minAll < 0.55 {
		t.Fatalf("发生碰撞：最小间距 %.3f 低于初始间距 0.6", minAll)
	}
	minAfter := math.Inf(1)
	for _, fr := range log.Frames {
		if fr.T < 3 {
			continue
		}
		if d := fr.Pos[0].Dist(fr.Pos[1]); d < minAfter {
			minAfter = d
		}
	}
	if minAfter < 0.95*p.DSafe {
		t.Fatalf("避碰未优先：3s 后最小间距 %.3f 仍低于安全距离 %.3f", minAfter, p.DSafe)
	}

	switches := 0
	prevMode := log.Frames[0].Mode
	for k, fr := range log.Frames {
		d := fr.Pos[0].Dist(fr.Pos[1])
		if k > 0 && d < p.DSafe && fr.Mode[0] != ModeAvoid && fr.Mode[1] != ModeAvoid {
			t.Fatalf("t=%.2f 间距 %.3f < 安全距离但双方均非避碰模式（仲裁优先级错误）", fr.T, d)
		}
		for i := range fr.Mode {
			if fr.Mode[i] != prevMode[i] {
				switches++
			}
		}
		prevMode = fr.Mode
		for _, a := range fr.Accel {
			if a.Len() > p.AMax+1e-9 {
				t.Fatalf("t=%.2f 加速度 %.3f 超过 AMax=%.2f", fr.T, a.Len(), p.AMax)
			}
		}
	}
	if switches > 8 {
		t.Fatalf("仲裁震荡：模式切换 %d 次，超过上限 8", switches)
	}
}

// TestRecoveryBoundedAcceleration 跟随者大偏差后渐进恢复：
// 1) 全程加速度、速度不超限（无瞬时速度突变）；
// 2) 每步速度增量不超过 AMax·dt；
// 3) 最终收敛回队形。
func TestRecoveryBoundedAcceleration(t *testing.T) {
	p := DefaultParams()
	p.Dt = 0.02
	path := NewPath(1.0, Vec{0, 0}, 0, Line(45))
	fs := initFollowers(path, []Vec{{-1.5, 0}})
	// 模拟绕障/故障后的大偏差：偏离期望位置约 8.5m
	fs[0].Pos = fs[0].Pos.Add(Vec{8, 3})
	fs[0].Vel = Vec{}
	sim := &Sim{Path: path, Followers: fs, Params: p, TEnd: 45}
	log := sim.Run()

	sawRecover := false
	for k, fr := range log.Frames {
		if fr.Mode[0] == ModeRecover {
			sawRecover = true
		}
		a := fr.Accel[0].Len()
		if a > p.AMax+1e-9 {
			t.Fatalf("t=%.2f 恢复加速度 %.3f 超过 AMax=%.2f", fr.T, a, p.AMax)
		}
		if k > 0 {
			dv := fr.Pos[0].Dist(log.Frames[k-1].Pos[0]) / p.Dt
			if dv > p.VMax+1e-6 {
				t.Fatalf("t=%.2f 速度 %.3f 超过 VMax=%.2f", fr.T, dv, p.VMax)
			}
		}
	}
	if !sawRecover {
		t.Fatal("大偏差下未进入恢复模式")
	}
	finalErr := log.Frames[len(log.Frames)-1].Err[0]
	if finalErr > p.RecoverExit {
		t.Fatalf("恢复未收敛：最终队形误差 %.3f > %.3f", finalErr, p.RecoverExit)
	}
}
