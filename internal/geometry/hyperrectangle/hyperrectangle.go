package hyperrectangle

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

type Side uint64

const (
	SideN = 1 << iota
	SideE
	SideS
	SideW

	CornerNE = SideN | SideE
	CornerSE = SideS | SideE
	CornerSW = SideS | SideW
	CornerNW = SideN | SideW
)

func N(r hyperrectangle.R, v vector.V) vector.V {
	vx, vy := v.X(), v.Y()

	xmin, xmax := r.Min().X(vnd.AXIS_X), r.Max().X(vnd.AXIS_X)
	ymin, ymax := r.Min().X(vnd.AXIS_Y), r.Max().X(vnd.AXIS_Y)

	var domain Side
	if dnorth := vy - ymax; dnorth >= 0 {
		domain |= SideN
	}
	if dsouth := ymin - vy; dsouth >= 0 {
		domain |= SideS
	}
	if deast := vx - xmax; deast >= 0 {
		domain |= SideE
	}
	if dwest := xmin - vx; dwest >= 0 {
		domain |= SideW
	}

	n := vector.M{0, 0}
	n.Copy(v)

	switch domain {
	case SideN:
		return vector.V{0, 1}
	case SideE:
		return vector.V{1, 0}
	case SideS:
		return vector.V{0, -1}
	case SideW:
		return vector.V{-1, 0}
	case CornerNE:
		n.Sub(vector.V{xmax, ymax})
		if epsilon.Within(vector.Magnitude(n.V()), 0) {
			n.Copy(vector.V{1, 1})
		}
	case CornerSE:
		n.Sub(vector.V{xmax, ymin})
		if epsilon.Within(vector.Magnitude(n.V()), 0) {
			n.Copy(vector.V{1, -1})
		}
	case CornerSW:
		n.Sub(vector.V{xmin, ymin})
		if epsilon.Within(vector.Magnitude(n.V()), 0) {
			n.Copy(vector.V{-1, -1})
		}
	case CornerNW:
		n.Sub(vector.V{xmin, ymax})
		if epsilon.Within(vector.Magnitude(n.V()), 0) {
			n.Copy(vector.V{-1, 1})
		}
	default:
		panic(fmt.Sprintf("invalid domain: %v", domain))
	}

	n.Unit()
	return n.V()
}
