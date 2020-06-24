package pointcloud

import (
	"fmt"
	"math"
	"os"
	"strconv"
	"testing"

	"github.com/keikohi/pointcloud/loader"
	"github.com/keikohi/pointcloud/writer"
	"gonum.org/v1/gonum/spatial/r2"
	"gonum.org/v1/gonum/spatial/r3"
)

const MaxUint = ^uint(0)
const MaxInt = int(MaxUint >> 1)

func TestRansac(t *testing.T) {

	filepath := "D:\\dev\\pc\\tower\\only_tower\\tower_000000.csv"
	outDir := "D:\\dev\\pc\\tower\\detected\\"
	loader := loader.CsvLoader{}
	points, _ := loader.Load(filepath)

	var towerCandidate []r3.Vec
	towerCandidate = points
	for {
		planeRansac, _ := NewPlaneRansac(towerCandidate, 1000, 0.5)
		plane := planeRansac.Fitting()
		if !plane.IsHorizontal() {
			break
		}
		planep, tmpTower := planeRansac.PlanePoints(plane)
		towerCandidate = tmpTower
		writePoints(outDir+strconv.Itoa(len(planep))+".csv", planep)
	}

	writePoints(outDir+"tower.csv", towerCandidate)

}

func TestCalcCircle(t *testing.T) {
	points3 := []r3.Vec{r3.Vec{1, 0, 0}, r3.Vec{0, 1, 0}, r3.Vec{-1, 0, 0}}
	points := [3]r2.Vec{r2.Vec{1, 0}, r2.Vec{0, 1}, r2.Vec{-1, 0}}
	circleRansac, _ := NewCircleRansac(points3, 1, 0.8)
	circle, ok := circleRansac.calcCircle(points)
	if ok {
		fmt.Fprintf(os.Stderr, "%+v \n", circle)
	}
	count := circleRansac.count(circle)
	fmt.Println(count)
}

func TestCircleFitting(t *testing.T) {

	filepath := "D:\\dev\\pc\\tower\\detected\\tower.csv"
	loader := loader.CsvLoader{}
	points, _ := loader.Load(filepath)

	zmin, zmax := zMinMax(points)
	const Interval float64 = 0.05
	sections, zlist := sectionalPoints(points, Interval, zmin.Z, zmax.Z)
	dir := "D:\\dev\\pc\\tower\\only_tower\\detected\\"
	centers := make([]r3.Vec, 0, len(zlist))
	for i, section := range sections {
		if len(section) < 10 {
			continue
		}
		eps := 0.05
		cr, _ := NewCircleRansac(section, 500, eps)
		circle := cr.Fitting()
		pf := PrimitiveFactory{}
		crcl := pf.Circle(circle, points[0].Z, 300)
		writePoints(dir+"cneter"+strconv.Itoa(i)+".csv", crcl)

		centers = append(centers, r3.Vec{X: circle.Center.X, Y: circle.Center.Y, Z: zlist[i]})

	}
	writePoints(dir+"cneters.csv", centers)

	// dir := "D:\\dev\\pc\\tower\\only_tower\\"

	// circleRansac, _ := NewCircleRansac(points, 8000, 0.8)
	// circle := circleRansac.Fitting()
	// pf := PrimitiveFactory{}
	// crcl := pf.Circle(circle, points[0].Z, 300)
	// pointsToStdout(crcl)
	// circlePath := dir + "circle.csv"
	// writePoints(circlePath, crcl)

	// circlep, _ := circleRansac.CirclePoints(circle)
	// path := dir + "circleRansacResult" + ".csv"
	// writePoints(path, circlep)

}

func TestLineRansac(t *testing.T) {
	filepath := "D:\\dev\\pc\\tower\\only_tower\\detected\\cneters.csv"
	loader := loader.CsvLoader{}
	points, _ := loader.Load(filepath)
	zmin, zmax := zMinMax(points)
	dir := "D:\\dev\\pc\\tower\\only_tower\\detected\\"
	lr, _ := NewLineRansac(points, 1000, 0.08)
	line := lr.Fitting()
	pf := PrimitiveFactory{}
	linep := pf.Line(line, zmin.Z, zmax.Z, 300)
	writePoints(dir+"linep.csv", linep)

	groundline := Line{udir: r3.Vec{X: 0, Y: 0, Z: 1}, p: line.p}
	verticalLine := pf.Line(groundline, zmin.Z, zmax.Z, 300)
	writePoints(dir+"verticalLine.csv", verticalLine)
	radius := angle(groundline.udir, line.udir)
	fmt.Println("tilt angle :", radius*180.0/math.Pi)
}

func TestTransformZ(t *testing.T) {
	filepath := "D:\\dev\\pc\\tower\\only_tower\\tower_000000.csv"
	outDir := "D:\\dev\\pc\\tower\\detected\\"
	loader := loader.CsvLoader{}
	points, _ := loader.Load(filepath)

	planeRansac, _ := NewPlaneRansac(points, 3000, 0.3)
	plane := planeRansac.Fitting()
	planep, tower := planeRansac.PlanePoints(plane)

	writePoints(outDir+"plane.csv", planep)
	writePoints(outDir+"tower.csv", tower)

	zmin, zmax := zMinMax(tower)
	const Interval float64 = 0.1
	const EPS float64 = 0.05
	const CIRCLE_ITERATION int = 500
	pf := PrimitiveFactory{}
	sections, zlist := sectionalPoints(tower, Interval, zmin.Z, zmax.Z)
	centers := make([]r3.Vec, 0, len(zlist))
	for i, section := range sections {
		if len(section) < 10 {
			continue
		}
		cr, _ := NewCircleRansac(section, CIRCLE_ITERATION, EPS)
		circle := cr.Fitting()
		//
		// crcl := pf.Circle(circle, zlist[i], 300)
		// writePoints(outDir+"center"+strconv.Itoa(i)+".csv", crcl)
		centers = append(centers, r3.Vec{X: circle.Center.X, Y: circle.Center.Y, Z: zlist[i]})
	}
	writePoints(outDir+"cneters.csv", centers)
	const LINE_ITERATION int = 500
	const PRIMITIVE_POINTS int = 500
	lr, _ := NewLineRansac(centers, LINE_ITERATION, EPS)
	line := lr.Fitting()
	linep := pf.Line(line, zmin.Z, zmax.Z, PRIMITIVE_POINTS)
	writePoints(outDir+"linep.csv", linep)

	Translate(line, tower)
	writePoints(outDir+"transformedTower.csv", tower)

	// groundline := Line{udir: r3.Vec{X: 0, Y: 0, Z: 1}, p: line.p}
	// verticalLine := pf.Line(groundline, zmin.Z, zmax.Z, 300)
	// writePoints(dir+"verticalLine.csv", verticalLine)
	// radius := angle(groundline.udir, line.udir)
	// fmt.Println("tilt angle :", radius*180.0/math.Pi)

}

func writePoints(path string, points []r3.Vec) {
	writer := writer.NewCsvWriter(path, points)
	writer.Write()
}

func pointsToStdout(points []r3.Vec) {
	for _, p := range points {
		fmt.Println(p.X, " ", p.Y, " ", p.Z)
	}
}
