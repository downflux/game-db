module github.com/downflux/go-collider

go 1.19

require (
	github.com/downflux/data v0.0.0-20230127054834-da810bbd7f7d
	github.com/downflux/go-bvh v1.0.1
	github.com/downflux/go-database v0.4.1
	github.com/downflux/go-geometry v0.16.0
)

require github.com/downflux/go-pq v0.3.0 // indirect

replace github.com/downflux/data => ../data
