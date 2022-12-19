package feature

import (
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

type F hyperrectangle.R

func New(min vector.V, max vector.V) *F { return (*F)(hyperrectangle.New(vnd.V(min), vnd.V(max))) }

func (f F) AABB() hyperrectangle.R { return hyperrectangle.R(f) }
