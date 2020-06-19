package loader

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"

	"gonum.org/v1/gonum/spatial/r3"
)

type CsvLoader struct {
}

func (cl CsvLoader) Load(filepath string) ([]r3.Vec, int) {
	fp, _ := os.Open(filepath)
	defer fp.Close()
	scanner := bufio.NewScanner(fp)
	lineNum := 0
	points := make([]r3.Vec, 0, 10e4)
	for scanner.Scan() {
		entry := scanner.Text()
		lineNum++
		fields := strings.Split(entry, ",")
		points = append(points, cl.toVec(fields))
	}
	fmt.Printf("load %d points \n", lineNum)
	return points, lineNum
}

func (cl *CsvLoader) toVec(fields []string) r3.Vec {
	x, _ := strconv.ParseFloat(fields[0], 64)
	y, _ := strconv.ParseFloat(fields[1], 64)
	z, _ := strconv.ParseFloat(fields[2], 64)
	return r3.Vec{X: x, Y: y, Z: z}
}
