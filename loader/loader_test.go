package loader

import "testing"

func TestLoader(t *testing.T) {

	filepath := "D:\\dev\\pc\\tower\\csv\\tower_000004.csv"
	loader := CsvLoader{}
	loader.Load(filepath)

}
