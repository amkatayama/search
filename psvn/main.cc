// © 2013 the Search Authors under the MIT license. See AUTHORS for the list of authors.

#include "psvn.hpp"
#include "../search/main.hpp"
#include <cstdio>

int main(int argc, const char *argv[]) {
	dfheader(stdout);

	for (int i = 0; i < argc; i++) {
		// read in parameters
	}
	
	PSVN d(stdin);
	search<PSVN>(d, argc, argv);
	dffooter(stdout);
	return 0;
}
