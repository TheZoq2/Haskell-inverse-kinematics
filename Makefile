run:
	make build
	make exec

build:
	stack build

exec:
	stack exec inversekinematics-exe
