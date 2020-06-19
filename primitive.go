package pointcloud

import (
	"fmt"
	"math"
	"os"

	"gonum.org/v1/gonum/spatial/r2"
	"gonum.org/v1/gonum/spatial/r3"
)

type PrimitiveFactory struct {
}

func (pf PrimitiveFactory) Circle(circle Circle, z float64, N int) []r3.Vec {
	circlep := make([]r3.Vec, 0, N)
	for i := 0; i < N; i++ {
		step := float64(i) / float64(N)
		radian := (2 * math.Pi) * step
		x := circle.Center.X + circle.Radius*math.Cos(radian)
		y := circle.Center.Y + circle.Radius*math.Sin(radian)
		circlep = append(circlep, r3.Vec{X: x, Y: y, Z: z})
	}
	return circlep
}

type Circle struct {
	Center r2.Vec
	Radius float64
}

func (c Circle) distance(v r2.Vec) float64 {
	return r2.Norm(c.Center.Sub(v)) - c.Radius
}

type plane struct {
	dir r3.Vec
	p   r3.Vec
}

func (pl *plane) distance(p r3.Vec) float64 {
	return math.Abs(pl.dir.Dot(p)-pl.dir.Dot(pl.p)) / r3.Norm(pl.dir)
}

func (pl plane) IsHorizontal() bool {
	udir := r3.Unit(pl.dir)
	fmt.Fprintf(os.Stderr, "Plane Dir: %+v \n", udir)
	if math.Abs(udir.Z) >= 0.9 {
		return true
	}
	return false
}
