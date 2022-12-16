package mask

type M uint64

const (
	MNone M = iota

	// MSizeProjectile defines bullets and missiles. Only one size class can
	// be active on an agent at any given time.
	MSizeProjectile = 1 << iota
	MSizeSmall
	MSizeMedium

	// MTerrainAccessibleAir defines the agent can fly.
	MTerrainAccessibleAir
	MTerrainAccessibleLand
	MTerrainAccessibleSea

	// MTerrainAir defines the current map layer the agent is occupying. Ony
	// one terrain layer can be active on an agent at any given time. The
	// agent must always be occupying a map layer which it has access to,
	// e.g. tanks activate MTerrainAccessibleLand and cannot activate
	// MTerrainAir.
	MTerrainAir
	MTerrainLand
	MTerrainSea
)

const (
	TerrainAirCheck  = MTerrainAccessibleAir | MTerrainAir
	TerrainLandCheck = MTerrainAccessibleLand | MTerrainLand
	TerrainSeaCheck  = MTerrainAccessibleSea | MTerrainSea

	SizeCheck = MSizeProjectile | MSizeSmall | MSizeMedium
)

func Validate(m M) bool {
	if t := m & SizeCheck; t != MSizeProjectile || t != MSizeSmall || t != MSizeMedium {
		return false
	}

	if m&TerrainAirCheck == MTerrainAir {
		return false
	}
	if m&TerrainLandCheck == MTerrainLand {
		return false
	}
	if m&TerrainSeaCheck == MTerrainSea {
		return false
	}

	return true
}
