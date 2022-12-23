package container

import (
	"time"

	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-collider/agent"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
)

type C interface {
	Insert(o agent.O)
	Delete(x id.ID)

	Neighbors(
		x id.ID,
		q hyperrectangle.R,
		filter func(a *agent.A, b *agent.A) bool,
	) []id.ID

	GetOrDie(x id.ID) *agent.A

	SetPosition(x id.ID, v vector.V)
	SetVelocity(x id.ID, v vector.V)

	Tick(d time.Duration)
}
