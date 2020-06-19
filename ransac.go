package pointcloud

import (
	"errors"
	"math"
	"math/rand"
	"os"
	"time"

	"golang.org/x/exp/errors/fmt"
	"gonum.org/v1/gonum/spatial/r2"
	"gonum.org/v1/gonum/spatial/r3"
)

const EPS float64 = 1.0e-8

type PlaneRancac struct {
	input     []r3.Vec
	iteration int
	eps       float64
}

func NewPlaneRansac(input []r3.Vec, iteration int, eps float64) (*PlaneRancac, error) {
	if len(input) < 3 {
		return nil, errors.New("input points should be more than 3")
	}
	if eps <= 0 {
		return nil, errors.New("iteration should be more than 0")
	}
	return &PlaneRancac{input: input, iteration: iteration, eps: eps}, nil
}

func (pr *PlaneRancac) Fitting() plane {
	maxScore := 0
	var maxPlane plane
	for i := 0; i < pr.iteration; i++ {
		planePoints := randomSampling(3, pr.input)
		v1 := planePoints[1].Sub(planePoints[0])
		v2 := planePoints[2].Sub(planePoints[0])
		plane := plane{dir: v1.Cross(v2), p: planePoints[0]}
		score := pr.count(plane)
		if score > maxScore {
			maxPlane = plane
			maxScore = score
			fmt.Fprintf(os.Stderr, "%d: max Num:%d, plane: %+v \n", i, score, maxPlane)
		}
	}
	return maxPlane
}

func (pr *PlaneRancac) PlanePoints(plane plane) ([]r3.Vec, []r3.Vec) {
	planep := make([]r3.Vec, 0, 10000)
	otherp := make([]r3.Vec, 0, 100000)
	// udir := r3.Unit(plane.dir)
	for _, p := range pr.input {
		// v := p.Sub(plane.p)
		// vdir := r3.Vec{X: v.X * udir.X, Y: v.Y * udir.Y, Z: v.Z * udir.Z}
		// if r3.Norm(vdir) <= pr.eps {
		// 	planep = append(planep, p)
		// }
		if plane.distance(p) <= pr.eps {
			planep = append(planep, p)
		} else {
			otherp = append(otherp, p)
		}
	}
	return planep, otherp
}

func (pr *PlaneRancac) count(pl plane) int {
	score := 0
	for _, p := range pr.input {
		if pl.distance(p) <= pr.eps {
			score++
		}
	}
	return score
}

func randomSampling(sampleNum int, population []r3.Vec) []r3.Vec {
	rand.Seed(time.Now().UnixNano())
	var sample []r3.Vec
	for i := 0; i < sampleNum; i++ {
		randomIndex := rand.Intn(len(population))
		sample = append(sample, population[randomIndex])
	}
	return sample
}

type CircleRansac struct {
	input     []r3.Vec
	iteration int
	eps       float64
}

func NewCircleRansac(input []r3.Vec, iteration int, eps float64) (*CircleRansac, error) {
	if len(input) < 3 {
		return nil, errors.New("input points should be more than 3")
	}
	if eps <= 0 {
		return nil, errors.New("iteration should be more than 0")
	}
	return &CircleRansac{input: input, iteration: iteration, eps: eps}, nil
}

func (cr *CircleRansac) Fitting() Circle {
	maxScore := 0
	var bestCircle Circle
	for i := 0; i < cr.iteration; i++ {
		points := randomSampling(3, cr.input)
		r2Vecs := [3]r2.Vec{cr.r2Vec(points[0]), cr.r2Vec(points[1]), cr.r2Vec(points[2])}
		circle, ok := cr.calcCircle(r2Vecs)
		if !ok {
			continue
		}
		score := cr.count(circle)
		if score > maxScore {
			maxScore = score
			bestCircle = circle
			fmt.Fprintf(os.Stderr, "%d: max Num:%d, circle: %+v \n", i, score, bestCircle)
		}
	}
	return bestCircle
}

func (cr *CircleRansac) CirclePoints(circle Circle) ([]r3.Vec, []r3.Vec) {
	circlep := make([]r3.Vec, 0, 500)
	otherp := make([]r3.Vec, 0, 100000)
	for _, p := range cr.input {
		if circle.distance(cr.r2Vec(p)) <= cr.eps {
			circlep = append(circlep, p)
		} else {
			otherp = append(otherp, p)
		}
	}
	return circlep, otherp
}

func (cr *CircleRansac) r2Vec(v r3.Vec) r2.Vec {
	return r2.Vec{X: v.X, Y: v.Y}
}

func (cr *CircleRansac) calcCircle(pnts [3]r2.Vec) (Circle, bool) {
	v1 := pnts[0]
	v2 := pnts[1]
	v3 := pnts[2]
	a1 := 2 * (v2.X - v1.X)
	b1 := 2 * (v2.Y - v1.Y)
	c1 := v1.X*v1.X + v1.Y*v1.Y - v2.X*v2.X - v2.Y*v2.Y

	a2 := 2 * (v3.X - v1.X)
	b2 := 2 * (v3.Y - v1.Y)
	c2 := v1.X*v1.X + v1.Y*v1.Y - v3.X*v3.X - v3.Y*v3.Y
	D := a1*b2 - a2*b1
	if math.Abs(D) < EPS {
		return Circle{}, false
	}
	cx := (b1*c2 - b2*c1) / D
	cy := (c1*a2 - c2*a1) / D
	cp := r2.Vec{X: cx, Y: cy}

	return Circle{Center: cp, Radius: r2.Norm(cp.Sub(v1))}, true
}

func (cr *CircleRansac) count(circle Circle) int {
	score := 0
	for _, p := range cr.input {
		if circle.distance(cr.r2Vec(p)) <= cr.eps {
			score++
		}
	}
	return score
}
