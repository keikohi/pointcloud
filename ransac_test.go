package pointcloud

import (
	"fmt"
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

	filepath := "D:\\dev\\pc\\tower\\only_tower\\tower_000000.csv"
	loader := loader.CsvLoader{}
	points, _ := loader.Load(filepath)

	// planeRansac, _ := NewPlaneRansac(points, 3000, 0.003)
	// plane := planeRansac.Fitting()

	// _, otherp := planeRansac.PlanePoints(plane)
	// otherWriter := writer.NewCsvWriter("D:\\dev\\pc\\tower\\only_tower\\noGround.csv", otherp)
	// otherWriter.Write()

	zmin, zmax := zMinMax(points)
	const Interval float64 = 0.5
	sections, zlist := sectionalPoints(points, Interval, zmin.Z, zmax.Z)
	dir := "D:\\dev\\pc\\tower\\only_tower\\detected\\"
	for i, section := range sections {
		if len(section) < 10 {
			continue
		}
		circleRansac, _ := NewCircleRansac(section, 2000, 0.01)
		circle := circleRansac.Fitting()
		if circle.Radius > 20 {
			continue
		}
		pf := PrimitiveFactory{}
		crcl := pf.Circle(circle, zlist[i], 500)
		fmt.Fprintf(os.Stderr, "%+v \n", circle)
		path := dir + "circlep" + strconv.Itoa(i) + ".csv"
		writePoints(path, crcl)
		// writer := writer.NewCsvWriter(path, circlep)
		// writer.Write()
	}

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

func writePoints(path string, points []r3.Vec) {
	writer := writer.NewCsvWriter(path, points)
	writer.Write()
}

func pointsToStdout(points []r3.Vec) {
	for _, p := range points {
		fmt.Println(p.X, " ", p.Y, " ", p.Z)
	}
}
