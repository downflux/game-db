package hyperrectangle

import (
	"testing"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

func TestN(t *testing.T) {
	r := *hyperrectangle.New(vnd.V{0, 0}, vnd.V{10, 10})
	type config struct {
		name string
		v    vector.V
		want vector.V
	}

	configs := []config{
		{
			name: "North",
			v:    vector.V{5, 20},
			want: vector.V{0, 1},
		},
		{
			name: "South",
			v:    vector.V{5, -10},
			want: vector.V{0, -1},
		},
		{
			name: "East",
			v:    vector.V{20, 5},
			want: vector.V{1, 0},
		},
		{
			name: "West",
			v:    vector.V{-10, 5},
			want: vector.V{-1, 0},
		},

		{
			name: "Corner/NE",
			v:    vector.V{10, 10},
			want: vector.Unit(vector.V{1, 1}),
		},
		{
			name: "Corner/SE",
			v:    vector.V{10, 0},
			want: vector.Unit(vector.V{1, -1}),
		},
		{
			name: "Corner/SW",
			v:    vector.V{0, 0},
			want: vector.Unit(vector.V{-1, -1}),
		},
		{
			name: "Corner/NW",
			v:    vector.V{0, 10},
			want: vector.Unit(vector.V{-1, 1}),
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := N(r, c.v); !vector.Within(got, c.want) {
				t.Errorf("N() = %v, want = %v", got, c.want)
			}
		})
	}
}
