package pointcloud

import (
	"errors"
	"math"
	"math/rand"
	"os"
	"sync"
	"time"

	"golang.org/x/exp/errors/fmt"
	"gonum.org/v1/gonum/spatial/r2"
	"gonum.org/v1/gonum/spatial/r3"
)

const EPS float64 = 1.0e-8
const LargeRadius = 25.0
const PARARLLEL = 10

type PlaneRancac struct {
	input     []r3.Vec
	iteration int
	eps       float64
	bestPlane Plane
	bestScore int
	mu        *sync.RWMutex
}

func NewPlaneRansac(input []r3.Vec, iteration int, eps float64) (*PlaneRancac, error) {
	if len(input) < 3 {
		return nil, errors.New("input points should be more than 3")
	}
	if eps <= 0 {
		return nil, errors.New("iteration should be more than 0")
	}
	return &PlaneRancac{input: input, iteration: iteration, eps: eps, bestScore: 0, bestPlane: Plane{}, mu: &sync.RWMutex{}}, nil
}

func (pr *PlaneRancac) Fitting() Plane {
	// maxScore := 0
	// var maxPlane Plane
	// for i := 0; i < pr.iteration; i++ {
	// 	planePoints := randomSampling(3, pr.input)
	// 	v1 := planePoints[1].Sub(planePoints[0])
	// 	v2 := planePoints[2].Sub(planePoints[0])
	// 	plane := Plane{dir: v1.Cross(v2), p: planePoints[0]}
	// 	score := pr.count(plane)
	// 	if score > maxScore {
	// 		maxPlane = plane
	// 		maxScore = score
	// 		fmt.Fprintf(os.Stderr, "%d: max Num:%d, plane: %+v \n", i, score, maxPlane)
	// 	}
	// }
	// return maxPlane
	var wg sync.WaitGroup
	for pi := 0; pi < PARARLLEL; pi++ {
		wg.Add(1)
		go func(pi int) {
			for i := 0; i < pr.iteration/PARARLLEL; i++ {
				planePoints := randomSampling(3, pr.input)
				v1 := planePoints[1].Sub(planePoints[0])
				v2 := planePoints[2].Sub(planePoints[0])
				plane := Plane{dir: v1.Cross(v2), p: planePoints[0]}
				score := pr.count(plane)
				pr.mu.Lock()
				if score > pr.bestScore {
					pr.bestPlane = plane
					pr.bestScore = score
					fmt.Fprintf(os.Stderr, "%d, %d: max Num:%d, plane: %+v \n", pi, i, score, pr.bestPlane)
				}
				pr.mu.Unlock()
			}
			wg.Done()
		}(pi)
	}
	wg.Wait()
	fmt.Println("fin-------------")
	return pr.bestPlane

}

func (pr *PlaneRancac) PlanePoints(plane Plane) ([]r3.Vec, []r3.Vec) {
	planep := make([]r3.Vec, 0, 10000)
	otherp := make([]r3.Vec, 0, 100000)
	// udir := r3.Unit(plane.dir)
	for _, p := range pr.input {
		if plane.distance(p) <= pr.eps {
			planep = append(planep, p)
		} else {
			otherp = append(otherp, p)
		}
	}
	return planep, otherp
}

func (pr *PlaneRancac) count(pl Plane) int {
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
	radius := r2.Norm(cp.Sub(v1))
	if radius > LargeRadius {
		return Circle{}, false
	}

	return Circle{Center: cp, Radius: radius}, true
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

type LineRansac struct {
	input     []r3.Vec
	iteration int
	eps       float64
}

func NewLineRansac(input []r3.Vec, iteration int, eps float64) (*LineRansac, error) {
	if len(input) < 2 {
		return nil, errors.New("input points should be more than 2")
	}
	if eps <= 0 {
		return nil, errors.New("iteration should be more than 0")
	}
	return &LineRansac{input: input, iteration: iteration, eps: eps}, nil
}

func (lr *LineRansac) calcLine(points [2]r3.Vec) Line {
	p := points[0]
	dir := r3.Unit(points[0].Sub(points[1]))
	return Line{udir: dir, p: p}
}

func (lr *LineRansac) count(line Line) int {
	score := 0
	for _, p := range lr.input {
		if line.distance(p) <= lr.eps {
			score++
		}
	}
	return score
}

func (lr *LineRansac) Fitting() Line {
	maxScore := 0
	var bestLine Line
	for i := 0; i < lr.iteration; i++ {
		points := randomSampling(2, lr.input)
		line := lr.calcLine([2]r3.Vec{points[0], points[1]})
		score := lr.count(line)
		if score > maxScore {
			maxScore = score
			bestLine = line
			fmt.Fprintf(os.Stderr, "%d: max Num:%d, line: %+v \n", i, score, bestLine)
		}
	}
	return bestLine
}
