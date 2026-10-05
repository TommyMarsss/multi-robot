// 编队仿真演示：生成 formation.html 回放文件。
package main

import (
	"fmt"
	"math"
	"os"

	formation "github.com/TommyMarsss/multi-robot"
)

func main() {
	p := formation.DefaultParams()
	p.Dt = 0.05

	// 领航者路径：直线 → 左转弧 → 直线 → 右转弧 → 直线
	path := formation.NewPath(1.2, formation.V(0, 0), 0,
		formation.Line(14),
		formation.Arc(7, 100*math.Pi/180),
		formation.Line(8),
		formation.Arc(6, -90*math.Pi/180),
		formation.Line(12),
	)

	// 楔形编队：偏移定义在路径坐标系 (切向, 法向)
	offsets := []formation.Vec{
		formation.V(-1.6, 1.6), formation.V(-1.6, -1.6), formation.V(-3.2, 0), formation.V(-4.8, 1.8),
	}
	ls0 := path.State(0)
	var followers []*formation.Follower
	for i, off := range offsets {
		f := formation.NewFollower(i, off, formation.Vec{})
		pDes, vDes := f.Desired(ls0)
		f.Pos, f.Vel = pDes, vDes
		followers = append(followers, f)
	}

	sim := &formation.Sim{
		Path:      path,
		Followers: followers,
		Params:    p,
		TEnd:      path.Duration(),
		Faults: []formation.Fault{
			// 中途大偏差：模拟绕障后偏离队形，验证渐进恢复
			{T: 16, ID: 2, Delta: formation.V(4, 3)},
			// 把 1 号推向 0 号：制造避碰与队形目标的冲突
			{T: 34, ID: 0, Delta: formation.V(0, -2.2)},
		},
	}
	log := sim.Run()

	out := "formation.html"
	if err := formation.WriteHTML(log, out); err != nil {
		fmt.Fprintln(os.Stderr, "写入失败:", err)
		os.Exit(1)
	}
	fmt.Printf("完成：%d 帧，时长 %.1fs → %s\n", len(log.Frames), log.Duration, out)
}
