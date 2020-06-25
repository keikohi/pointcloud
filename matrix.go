package pointcloud

import (
	"math"

	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/spatial/r3"
)

type Matrix struct {
	A *mat.Dense
}

func NewPlaneMatrix(plane Plane) (Matrix, bool) {
	return calcTransformMat(plane.p, r3.Unit(plane.dir))
}

func NewLineMatrix(line Line) (Matrix, bool) {
	return calcTransformMat(line.p, r3.Unit(line.udir))
}

func calcTransformMat(p r3.Vec, udir r3.Vec) (Matrix, bool) {
	if udir.X < 1.0e-8 && udir.Y < 1.0e-8 {
		return Matrix{}, false
	}
	ycos := udir.Z
	ysin := -math.Sqrt(udir.X*udir.X + udir.Y*udir.Y)

	zNorm := math.Sqrt(udir.X*udir.X + udir.Y*udir.Y)
	zcos := udir.X / zNorm
	zsin := -udir.Y / zNorm

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
		1.0, 0.0, 0.0, -p.X,
		0.0, 1.0, 0.0, -p.Y,
		0.0, 0.0, 1.0, -p.Z,
		0.0, 0.0, 0.0, 1.0}

	matY := mat.NewDense(4, 4, rotateY)
	matZ := mat.NewDense(4, 4, rotateZ)
	matT := mat.NewDense(4, 4, transform)

	A := mat.NewDense(4, 4, nil)
	A.Product(matY, matZ, matT)
	return Matrix{A: A}, true
}

func (matrix Matrix) Translate(points []r3.Vec) {
	for i := 0; i < len(points); i++ {
		x := []float64{points[i].X, points[i].Y, points[i].Z, 1.0}
		X := mat.NewDense(4, 1, x)
		AX := mat.NewDense(4, 1, nil)
		AX.Product(matrix.A, X)
		points[i].X = AX.At(0, 0)
		points[i].Y = AX.At(1, 0)
		points[i].Z = AX.At(2, 0)
	}
}

func (matrix Matrix) TranslateInverse(points []r3.Vec) error {
	for i := 0; i < len(points); i++ {
		x := []float64{points[i].X, points[i].Y, points[i].Z, 1.0}
		X := mat.NewDense(4, 1, x)
		AX := mat.NewDense(4, 1, nil)
		err := matrix.A.Inverse(matrix.A)
		if err != nil {
			return err
		}
		AX.Product(matrix.A, X)
		points[i].X = AX.At(0, 0)
		points[i].Y = AX.At(1, 0)
		points[i].Z = AX.At(2, 0)
	}
	return matrix.A.Inverse(matrix.A)
}
