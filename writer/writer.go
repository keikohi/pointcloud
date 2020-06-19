package writer

import (
	"encoding/csv"
	"os"
	"strconv"

	"gonum.org/v1/gonum/spatial/r3"
)

type CsvWriter struct {
	filepath string
	points   []r3.Vec
}

func NewCsvWriter(filepath string, points []r3.Vec) CsvWriter {
	return CsvWriter{filepath: filepath, points: points}
}

func (cw *CsvWriter) Write() error {
	fp, err := os.Create(cw.filepath)
	if err != nil {
		return err
	}
	defer fp.Close()

	writer := csv.NewWriter(fp)
	for _, p := range cw.points {
		x := strconv.FormatFloat(p.X, 'f', 6, 64)
		y := strconv.FormatFloat(p.Y, 'f', 6, 64)
		z := strconv.FormatFloat(p.Z, 'f', 6, 64)
		writer.Write([]string{x, y, z, "0", "0", "0"})
	}
	writer.Flush()
	return nil
}
