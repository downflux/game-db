package mask

type M uint64

const (
	MNone M = iota

	MTerrainAccessibleAir = 1 << iota
	MTerrainAccessibleLand
	MTerrainAccessibleSea

	MTerrainAir
	MTerrainLand
	MTerrainSea
)

const (
	TerrainAirCheck  = MTerrainAccessibleAir | MTerrainAir
	TerrainLandCheck = MTerrainAccessibleLand | MTerrainLand
	TerrainSeaCheck  = MTerrainAccessibleSea | MTerrainSea
)

func Validate(m M) bool {
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
