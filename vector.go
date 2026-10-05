package formation

import "math"

// Vec 二维向量。
type Vec struct{ X, Y float64 }

// V 构造向量（供包外使用，避免未命名字段字面量）。
func V(x, y float64) Vec { return Vec{X: x, Y: y} }

func (a Vec) Add(b Vec) Vec       { return Vec{a.X + b.X, a.Y + b.Y} }
func (a Vec) Sub(b Vec) Vec       { return Vec{a.X - b.X, a.Y - b.Y} }
func (a Vec) Scale(k float64) Vec { return Vec{a.X * k, a.Y * k} }
func (a Vec) Dot(b Vec) float64   { return a.X*b.X + a.Y*b.Y }
func (a Vec) Len() float64        { return math.Hypot(a.X, a.Y) }
func (a Vec) Dist(b Vec) float64  { return a.Sub(b).Len() }

// Norm 单位化；零向量返回零向量。
func (a Vec) Norm() Vec {
	l := a.Len()
	if l < 1e-12 {
		return Vec{}
	}
	return a.Scale(1 / l)
}

// clampLen 将向量模长限制到 max（保持方向）。
func clampLen(v Vec, max float64) Vec {
	l := v.Len()
	if l <= max || l < 1e-12 {
		return v
	}
	return v.Scale(max / l)
}
