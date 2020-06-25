package pointcloud

// func TestMatrix(t *testing.T) {

// 	line := Line{p: r3.Vec{X: 0.0, Y: 0.0, Z: 0.0}, udir: r3.Vec{X: 1.0, Y: 0.0, Z: 0.0}}
// 	var points []r3.Vec
// 	points = append(points, r3.Vec{X: 1.0, Y: 0.0, Z: 0.0})

// 	Translate(line, points)
// 	fmt.Fprintf(os.Stderr, "%+v \n", points)
// }

// func TestMatrix2(t *testing.T) {

// 	line := Line{p: r3.Vec{X: 0.0, Y: 0.0, Z: 0.0}, udir: r3.Vec{X: 0.0, Y: 1.0, Z: 0.0}}
// 	var points []r3.Vec
// 	points = append(points, r3.Vec{X: 0.0, Y: 3.0, Z: 0.0})

// 	Translate(line, points)
// 	fmt.Fprintf(os.Stderr, "%+v \n", points)
// }

// func TestMatrixTransform(t *testing.T) {

// 	line := Line{p: r3.Vec{X: 0.0, Y: 0.0, Z: 1.0}, udir: r3.Vec{X: 1.0 / math.Sqrt(2), Y: 1.0 / math.Sqrt(2), Z: 0.0}}
// 	var points []r3.Vec
// 	points = append(points, r3.Vec{X: 1.0 / math.Sqrt(2), Y: 1.0 / math.Sqrt(2), Z: 1})

// 	Translate(line, points)
// 	fmt.Fprintf(os.Stderr, "%+v \n", points)
// }

// func TestMatrixTransform2(t *testing.T) {

// 	line := Line{p: r3.Vec{X: 1.0, Y: 1.0, Z: 1.0}, udir: r3.Vec{X: 0.0, Y: 1.0, Z: 0.0}}
// 	var points []r3.Vec
// 	points = append(points, r3.Vec{X: 1.0, Y: 1.0, Z: 2.0})

// 	Translate(line, points)
// 	if !(points[0].X == -1 && points[0].Y == 0.0 && points[0].Z == 0.0) {
// 		fmt.Fprintf(os.Stderr, "%+v \n", points)
// 		t.Fail()
// 	}
// }
