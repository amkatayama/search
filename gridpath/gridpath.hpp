#include "gridmap.hpp"
#include <cstdio>
#include <cmath>
#include <cstdlib>

class GridPath {
public:

	enum { UnitCost = false };

	typedef float Cost;
	static const float InfCost = -1.0;

	typedef int Oper;	// Index into the ops arrays.
	enum { Nop = -1 };

	GridPath(GridMap*, unsigned int, unsigned int,
		unsigned int, unsigned int);

	class State {
		friend class GridPath;
		unsigned int loc;
		int nops;
		Oper ops[8];
	public:
		State(void) : nops(-1) { }
	};

	class PackedState {
		friend class GridPath;
		unsigned int loc;
	public:
		unsigned long hash(void) { return loc; }

		bool eq(const PackedState &other) const {
			return other.loc == loc;
		}
	};

	struct Undo {
		Undo(State &, Oper) { }
	};

	State initialstate(void);

	Cost h(State &s) {
		return octiledist(s.loc, finish);
	}

	bool isgoal(State &s) {
		return s.loc == finish;
	}

	unsigned int nops(State &s) {
		if (s.nops < 0) {
			s.nops = 0;

			bool upok = map->terrainok(s.loc, map->up(s.loc));
			if (upok)
				s.ops[s.nops++] = map->up(s.loc);

			bool downok = map->terrainok(s.loc, map->down(s.loc));
			if (downok)
				s.ops[s.nops++] = map->down(s.loc);

			bool leftok = map->terrainok(s.loc, map->left(s.loc));
			if (leftok)
				s.ops[s.nops++] = map->left(s.loc);

			bool rightok= map->terrainok(s.loc, map->right(s.loc));
			if (rightok)
				s.ops[s.nops++] = map->right(s.loc);

			const unsigned int upleft = map->up(map->left(s.loc));
			if (upok && leftok && map->terrainok(s.loc, upleft))
				s.ops[s.nops++] = upleft;
 
			const unsigned int downleft = map->down(map->left(s.loc));
			if (downok && leftok && map->terrainok(s.loc, downleft))
				s.ops[s.nops++] = downleft;

			const unsigned int upright = map->up(map->right(s.loc));
			if (upok && rightok && map->terrainok(s.loc, upright))
				s.ops[s.nops++] = upright;

			const unsigned int downright = map->down(map->right(s.loc));
			if (downok && rightok && map->terrainok(s.loc, downright))
				s.ops[s.nops++] = downright;
		}
		return s.nops;
	}

	Oper nthop(State &s, unsigned int n) {
		return s.ops[n];
	}

	Oper revop(State &s, Oper op) {
		return s.loc;
	}

	Cost opcost(State &s, Oper op) {
		if (map->x(s.loc) == map->x(op) || map->y(s.loc) == map->y(op))
			return 1;
		return sqrtf(2.0);
	}

	void undo(State &s, Undo &u) { }

	State &apply(State &buf, State &s, Oper op) {
		buf.loc = op;
		return buf;
	}

	void pack(PackedState &dst, State &src) {
		dst.loc = src.loc;
	}

	State &unpack(State &buf, PackedState &pkd) {
		buf.loc = pkd.loc;
		return buf;
	}

	void dumpstate(FILE *out, State &s) {
		fprintf(out, "%u, %u\n", map->x(s.loc), map->y(s.loc));
	}

	unsigned int width(void) { return map->width(); }

	unsigned int height(void) { return map->height(); }

private:

	float octiledist(unsigned int l0, unsigned int l1) {
		unsigned int dx = abs(map->x(l0) - map->x(l1));
		unsigned int dy = abs(map->y(l0) - map->y(l1));
		unsigned int diag = dx < dy ? dx : dy;
		unsigned int straight = dx < dy ? dy : dx;
		return (straight - diag) + sqrtf(2.0) * diag;
	}

	unsigned int start, finish;
	GridMap *map;
};