package interfaces

import (
	"github.com/downflux/data/agent/size"
	"github.com/downflux/data/id"
	"github.com/downflux/data/shared/layer"
	"github.com/downflux/go-geometry/2d/hyperrectangle"
	"github.com/downflux/go-geometry/2d/vector"
)

// Ephemeral describes an object which may be inserted or removed.
type Ephemeral interface {
	IsInserted() bool
	IsRemoved() bool
}

type FeatureCollider interface {
	ID() id.ID
	AABB() hyperrectangle.R

	Layer() layer.F
}

type ProjectileCollider interface {
	ID() id.ID

	TargetVelocity(v vector.V)

	SetPosition(v vector.V)
	SetHeading(v vector.V)
	SetVelocity(v vector.V)
}

type AgentCollider interface {
	ID() id.ID
	AABB() hyperrectangle.R

	Position(v vector.V)
	TargetVelocity(v vector.V)

	SetPosition(v vector.V)
	SetHeading(v vector.V)
	SetVelocity(v vector.V)

	Layer() layer.F
	Size() size.F
}
