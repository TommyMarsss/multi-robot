package formation

import "math"

// Vec2 是二维向量，位置、速度、加速度统一使用。
type Vec2 struct {
	X, Y float64
}

func V(x, y float64) Vec2 { return Vec2{X: x, Y: y} }

func (a Vec2) Add(b Vec2) Vec2    { return Vec2{a.X + b.X, a.Y + b.Y} }
func (a Vec2) Sub(b Vec2) Vec2    { return Vec2{a.X - b.X, a.Y - b.Y} }
func (a Vec2) Mul(s float64) Vec2 { return Vec2{a.X * s, a.Y * s} }
func (a Vec2) Dot(b Vec2) float64 { return a.X*b.X + a.Y*b.Y }
func (a Vec2) Len() float64       { return math.Hypot(a.X, a.Y) }

// Norm 返回单位向量；零向量返回零向量。
func (a Vec2) Norm() Vec2 {
	l := a.Len()
	if l < 1e-12 {
		return Vec2{}
	}
	return a.Mul(1 / l)
}

// clampMag 将向量模长限制到 max，方向不变。
func clampMag(a Vec2, max float64) Vec2 {
	l := a.Len()
	if l > max && l > 1e-12 {
		return a.Mul(max / l)
	}
	return a
}
