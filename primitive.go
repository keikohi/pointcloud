package pointcloud

import (
	"errors"
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

func (pf PrimitiveFactory) Line(line Line, zmin, zmax float64, N int) []r3.Vec {
	linep := make([]r3.Vec, 0, N)
	botPlane := Plane{dir: r3.Vec{0, 0, 1}, p: r3.Vec{0, 0, zmin - 30.0}}
	topPlane := Plane{dir: r3.Vec{0, 0, 1}, p: r3.Vec{0, 0, zmax + 30.0}}
	botLinep, _ := CrossLinePlane(line, botPlane)
	topLinep, _ := CrossLinePlane(line, topPlane)
	udir := r3.Unit(topLinep.Sub(botLinep))
	step := (zmax - zmin) / float64(N)
	curZ := botPlane.p.Z
	count := 0.0
	for curZ < topLinep.Z {
		linep = append(linep, botLinep.Add(udir.Scale(count*step)))
		curZ += step
		count++
	}
	return linep
}

type Circle struct {
	Center r2.Vec
	Radius float64
}

func (c Circle) distance(v r2.Vec) float64 {
	return r2.Norm(c.Center.Sub(v)) - c.Radius
}

type Plane struct {
	dir r3.Vec
	p   r3.Vec
}

func (pl *Plane) distance(p r3.Vec) float64 {
	return math.Abs(pl.dir.Dot(p)-pl.dir.Dot(pl.p)) / r3.Norm(pl.dir)
}

func (pl Plane) IsHorizontal() bool {
	udir := r3.Unit(pl.dir)
	fmt.Fprintf(os.Stderr, "Plane Dir: %+v \n", udir)
	if math.Abs(udir.Z) >= 0.9 {
		return true
	}
	return false
}

type Line struct {
	udir r3.Vec
	p    r3.Vec
}

func (ln Line) distance(p r3.Vec) float64 {
	v := p.Sub(ln.p)
	// t := r3.Vec{
	// 	X: v.X * ln.udir.X,
	// 	Y: v.Y * ln.udir.Y,
	// 	Z: v.Z * ln.udir.Z}
	t := v.Dot(ln.udir)
	return math.Sqrt(math.Abs(r3.Norm(v)*r3.Norm(v) - t*t))
}

func CrossLinePlane(line Line, plane Plane) (r3.Vec, error) {
	pldir := r3.Unit(plane.dir)
	t1 := pldir.Dot(line.udir)
	if math.Abs(t1) < 1.0e-8 {
		return r3.Vec{}, errors.New("Plane and line don't have a cross point ")
	}
	t2 := pldir.Dot(plane.p.Sub(line.p))
	deltaV := line.udir.Scale(t2 / t1)
	return deltaV.Add(line.p), nil
}

func angle(v1 r3.Vec, v2 r3.Vec) float64 {
	udir1 := r3.Unit(v1)
	udir2 := r3.Unit(v2)
	c := udir1.Dot(udir2)
	if c > 1.0 {
		c = 1
	} else if c < -1.0 {
		c = -1
	}
	return math.Acos(c)
}
