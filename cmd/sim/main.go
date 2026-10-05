package main

import (
	"fmt"
	"log"
	"math"

	"github.com/TommyMarsss/multi-robot/formation"
)

// 演示场景：领航者直线 → 左转 90° → 直线 → 右转 90° → 直线。
// 跟随者：楔形队形；F3 初始大偏差（演示渐进恢复）；F2/F4 偏移过近（演示避碰仲裁）。
func main() {
	p := formation.DefaultParams()
	path := formation.LeaderPath{Segs: []formation.Segment{
		{Dur: 4, V: 2}, // 直线
		{Dur: math.Pi / 2 / 0.3, V: 2, Omega: 0.3}, // 左转 90°
		{Dur: 4, V: 2}, // 直线
		{Dur: math.Pi / 2 / 0.3, V: 2, Omega: -0.3}, // 右转 90°
		{Dur: 5, V: 2}, // 直线
	}}

	mk := func(name string, off formation.Offset) *formation.Follower {
		d := formation.Desired(path.State(0), off)
		return &formation.Follower{Name: name, Off: off, Pos: d.Pos, Vel: d.Vel}
	}
	f1 := mk("F1", formation.Offset{Along: -2, Across: 2})
	f2 := mk("F2", formation.Offset{Along: -2, Across: -2})
	// F3 初始偏离期望位置约 10m（模拟绕障后归队）
	f3 := mk("F3", formation.Offset{Along: -4, Across: 0})
	f3.Pos = f3.Pos.Add(formation.V(6, -8))
	f3.Vel = formation.V(0, 0)
	// F4 与 F2 的期望间距 0.92m < dSafe=1.0m（过密队形，触发避碰仲裁）
	f4 := mk("F4", formation.Offset{Along: -2.6, Across: -1.3})

	sim := &formation.Sim{
		Path:      path,
		P:         p,
		Dt:        0.05,
		Followers: []*formation.Follower{f1, f2, f3, f4},
	}
	sim.Run()

	rd := sim.BuildReport()
	fmt.Println(rd)
	fmt.Printf("min pair distance: %.3f m (dSafe=%.2f)\n", sim.MinPairDistance(), p.DSafe)
	maxErr := 0.0
	for _, fr := range sim.Frames[len(sim.Frames)/2:] { // 后半程稳态
		for _, f := range fr.Followers {
			if f.Err > maxErr {
				maxErr = f.Err
			}
		}
	}
	fmt.Printf("max formation error (2nd half): %.3f m\n", maxErr)

	if err := formation.WriteReport("report.html", rd); err != nil {
		log.Fatal(err)
	}
	fmt.Println("report.html written")
}
