#include "unknowngridnav.hpp"
#include "../search/unknownlsslrtastar2.hpp"
#include "../search/unknownfhatlrtastar.hpp"
#include <cstdio>
#include <cerrno>

int main(int argc, const char *argv[]) {
	dfheader(stdout);

	FILE *lvl = stdin;
	const char *lvlpath = "";
	for (int i = 0; i < argc; i++) {
		if (i < argc - 1 && strcmp(argv[i], "-lvl") == 0)
			lvlpath = argv[++i];
	}

	if (lvlpath[0] != '\0') {
		lvl = fopen(lvlpath, "r");
		if (!lvl)
			fatalx(errno, "Failed to open %s for reading", lvlpath);
	}

	UnknownGridMap map(lvl);

	unsigned int x0, y0, xg, yg;
	if (fscanf(lvl, " %u %u %u %u", &x0, &y0, &xg, &yg) != 4)
		fatal("Failed to read start and end locations");

	if (lvlpath[0] != '\0') {
		dfpair(stdout, "level", "%s", lvlpath);
		fclose(lvl);
 	}

	dfpair(stdout, "start x", "%u", x0);
	dfpair(stdout, "start y", "%u", y0);
	dfpair(stdout, "goal x", "%u", xg);
	dfpair(stdout, "goal y", "%u", yg);

	if (map.blkd(map.index(x0+1, y0+1)))
		fatal("The start location is blocked");
	if (map.blkd(map.index(xg+1, yg+1)))
		fatal("The goal location is blocked");

	UnknownGridNav d(&map, x0, y0, xg, yg);

	SearchAlgorithm<UnknownGridNav> *search = NULL;
	for (int i = 0; i < argc; i++) {
		if(strcmp(argv[i], "lsslrtastar2") == 0)
			search = new UnknownLsslrtastar2<UnknownGridNav>(argc, argv);
		else if(strcmp(argv[i], "fhatlrtastar") == 0)
			search = new UnknownFhatlrtastar<UnknownGridNav>(argc, argv);
	}

	if(!search) {
		fatal("unsupported or missing search algorithm");
	}

	GridNav::State s0 = d.initialstate();
	search->search(d, s0);
	dfpair(stdout, "solution cost", "%g",  d.pathcost(search->res.path, search->res.ops, (lvlpath[0] != '\0')));
	search->output(stdout);

	dffooter(stdout);

	return 0;
}