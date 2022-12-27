package feature

import (
	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-collider/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

type O struct {
	Min vector.V
	Max vector.V

	Mask mask.M
}

type F struct {
	id   id.ID
	aabb hyperrectangle.R
	mask mask.M
}

func (f *F) ID() id.ID              { return f.id }
func (f *F) AABB() hyperrectangle.R { return f.aabb }
func (f *F) Mask() mask.M           { return f.mask }

// SetID is only called by internal libraries. This function must not be invoked
// by external users.
func SetID(f *F, x id.ID) { f.id = x }

// New is only called by internal libraries.
func New(o O) *F {
	f := &F{
		aabb: *hyperrectangle.New(
			vnd.V(make([]float64, 2)),
			vnd.V(make([]float64, 2)),
		),
	}
	f.aabb.M().Min().Copy(vnd.V(o.Min))
	f.aabb.M().Max().Copy(vnd.V(o.Max))
	return f
}
