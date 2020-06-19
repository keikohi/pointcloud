package pointcloud

import (
	"fmt"
	"os"

	"gonum.org/v1/gonum/spatial/r3"
)

const LargeValue float64 = 1.0e8
const MinValue float64 = 1.0e8

func zMinMax(obj []r3.Vec) (r3.Vec, r3.Vec) {
	mini := 0
	maxi := 0
	maxv := obj[0]
	minv := obj[0]
	for i, p := range obj {
		if p.Z < minv.Z {
			minv = p
			mini = i
		}
		if p.Z > maxv.Z {
			maxv = p
			maxi = i
		}
	}
	fmt.Fprintf(os.Stderr, "Z min: %+v, max: %+v", obj[mini], obj[maxi])
	return obj[mini], obj[maxi]
}

func sectionalPoints(obj []r3.Vec, interval, zmin, zmax float64) ([][]r3.Vec, []float64) {
	var zlist []float64
	zi := zmin
	sections := make([][]r3.Vec, 0, 5000)
	for zi <= zmax {
		zlist = append(zlist, zi)
		section := make([]r3.Vec, 0, 5000)
		for _, p := range obj {
			if p.Z >= zi && p.Z <= zi+interval {
				section = append(section, p)
			}
		}
		sections = append(sections, section)
		zi += interval
	}
	return sections, zlist
}
