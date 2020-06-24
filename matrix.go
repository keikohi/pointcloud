package pointcloud

import (
	"math"

	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/spatial/r3"
)

func Translate(line Line, points []r3.Vec) {

	if line.udir.X < 1.0e-8 && line.udir.Y < 1.0e-8 {
		return
	}
	ycos := line.udir.Z
	ysin := -math.Sqrt(line.udir.X*line.udir.X + line.udir.Y*line.udir.Y)

	zNorm := math.Sqrt(line.udir.X*line.udir.X + line.udir.Y*line.udir.Y)
	zcos := line.udir.X / zNorm
	zsin := -line.udir.Y / zNorm

	rotateY := []float64{
		ycos, 0.0, ysin, 0.0,
		0.0, 1.0, 0.0, 0.0,
		-ysin, 0.0, ycos, 0.0,
		0.0, 0.0, 0.0, 1.0}

	rotateZ := []float64{
		zcos, -zsin, 0.0, 0.0,
		zsin, zcos, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0}

	transform := []float64{
		1.0, 0.0, 0.0, -line.p.X,
		0.0, 1.0, 0.0, -line.p.Y,
		0.0, 0.0, 1.0, -line.p.Z,
		0.0, 0.0, 0.0, 1.0}

	matY := mat.NewDense(4, 4, rotateY)
	matZ := mat.NewDense(4, 4, rotateZ)
	matT := mat.NewDense(4, 4, transform)

	A := mat.NewDense(4, 4, nil)
	A.Product(matY, matZ, matT)

	for i := 0; i < len(points); i++ {
		x := []float64{points[i].X, points[i].Y, points[i].Z, 1.0}
		X := mat.NewDense(4, 1, x)
		AX := mat.NewDense(4, 1, nil)
		AX.Product(A, X)
		points[i].X = AX.At(0, 0)
		points[i].Y = AX.At(1, 0)
		points[i].Z = AX.At(2, 0)
	}
}
