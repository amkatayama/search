// Here is the class template for creating a new domain.
// Search algorithms are templatized on domains that
// must look like this.

#include "../utils/utils.hpp"
#include <cstdio>

// #pragma GCC diagnostic ignored "-Wuninitialized"


struct PSVN {

	// Probably double, float, or int.  Must be convertable
	// to a double, must be constructable from an int.
	// Cost(-1) is reserved for infinite cost.  See gridnav
	// for an example of a kind of sophistocated cost type.
	// 
	// Some searches will use a special (faster)
	// closed list if the costs are ints.
	typedef int Cost;

	// The type of an operator which can be
	// applied to a state.  This is usually just an
	// integer but it may be some more complex
	// class.  Searches assume that operator==
	// is defined on the Oper class.

	// int because we are pointing to array of operators 
	typedef int Oper;  
	static const Oper Nop = -1;

	// Pattern Database
	

    // PSVN constructor reads three things for three different files
	// 1. initial state 
	// 2. abstraction rule 
	// 3. pattern database 
	PSVN (FILE* in) {
		char line[ ] = "";
		fscanf(in, "%[^\n]", line);
		read_state(line, &init_state); 

		fprintf(stdout, "Initial state: ");
		print_state(stdout, &init_state);
		fprintf(stdout, "\n");

		state_t fwd = init_state;

		std::vector<int> rules;

		ruleid_iterator_t fwd_iter;
		int fwd_ruleid;
		init_fwd_iter( &fwd_iter, &init_state);
		fwd_ruleid = next_ruleid( &fwd_iter );
		// while ( (fwd_ruleid = next_ruleid( &fwd_iter )) >= 0 ) {
		// 	rules.push_back( fwd_ruleid );
		// }

		apply_fwd_rule(fwd_ruleid, &fwd, &init_state);

		fprintf(stdout, "state after fwdrule %d applied: ", fwd_ruleid);
		print_state(stdout, &init_state);
		fprintf(stdout, "\n");
	
		
		state_t bwd = init_state;
		apply_bwd_rule(fwd_ruleid, &bwd, &init_state);

		fprintf(stdout, "state after bwdrule %d applied: ", fwd_ruleid);
		print_state(stdout, &init_state);
		fprintf(stdout, "\n");

		// print intial state 
		// do some operator to it
		// generate edge with the operator 
		// print state
		// call edge destructor (delete?)
		// print state

		abst = read_abstraction_from_file( ABSTFILE );
		if ( abst == NULL ){
        	fatal("could not read the abstraction file");
   		}

		FILE* pdb_file = fopen( PDBFILE , "r");
		pdb = read_state_map( pdb_file );
		if ( pdb == NULL ){
        	fatal("could not read the abstraction file");
   		}
		fclose(pdb_file);
	}

	struct State {
		state_t psvn_state;

		bool eq(const PSVN*, const State &other) const {
			return compare_states(&psvn_state, &other.psvn_state) == 0;
		}

		unsigned long hash(const PSVN*) const {
			return hash_state(&psvn_state);
		}
	};

	// typedef state_t State ;


	// Memory-intensive algs such as A* which store
	// PackedStates instead of States.  Each time operations	
	// are needed, the state is unpacked and operated
	// upon.
	//
	// If your state is as packed as it will get then you
	// can simply 'typedef State PackedState'
	typedef State PackedState;	

	// struct PackedState {
	// 	…

	// 	// Functions for putting a packed state
	// 	// into a hash table.
	// 	// bool operator==(const PackedState &) const {
	// 	// 	return false;
	// 	// }
	// };

	// Get the initial state.
	State initialstate(void) const {
		State s;
        // initializing psvnstate and abstracting it 
		s.psvn_state = init_state;

		return s;
	}

	// Get the heuristic.
	Cost h(const State &s) const {
		// read pdb from pdb_file, store in file pointer
		State abst_state;
		// create an abstract state corresponding to the state pointer using abstraction pointed by abst pointer
		abstract_state( abst, &s.psvn_state, &abst_state.psvn_state );

		// fprintf(stdout, "Current state: ");
		// print_state(stdout, &s.psvn_state);
		// fprintf(stdout, "\n");
		
		// get distance of abst_state from pdb 
		int *h;
    	h = state_map_get( pdb, &abst_state.psvn_state );

		return *h;
	}

	// Get a distance estimate.
	Cost d(const State &s) const {
		return h(s);
	}

	// Is the given state a goal state?
	bool isgoal(const State &s) const {
		return is_goal( &s.psvn_state );
	}

	// Operators implements an vector of the applicable
	// operators for a given state.
	struct Operators {

		// both forward and backward operators
		std::vector<int> rules;

		Operators(const PSVN&, const State &s) {
			// the direction of iterator is fixed when initialized
			ruleid_iterator_t fwd_iter;
			// ruleid_iterator_t bwd_iter;		
			int fwd_ruleid;
			// int bwd_ruleid;
			init_fwd_iter( &fwd_iter, &s.psvn_state );
			// init_bwd_iter( &bwd_iter, &s.psvn_state );
			while ( (fwd_ruleid = next_ruleid( &fwd_iter )) >= 0 ) {
				rules.push_back( fwd_ruleid );
			}
		}

		// size returns the number of applicable operators.
		unsigned int size() const {
			return rules.size();
		}

		// operator[] returns a specific operator.
		Oper operator[] (unsigned int i) const { 
			return rules[i];
		}
	};

	// edge in pancake or other domains?
	// this is where operation actually happen
	struct Edge {
		Cost cost;
		Oper revop;
		Cost revcost;

		// The state field may or may not be a reference.
		// The reference variant is used in domains that
		// do in-place modification and the non-reference
		// variant is used in domains that do out-of-place
		// modification.
		// State state;
		State &state;

		// Applys the operator to thet state.  Some domains
		// may modify the input state in this constructor.
		// Because of this, a search algorithm may not
		// use the state passed to this constructor until
		// after the Edge's destructor has been called!
		Edge(const PSVN &d, State &s, Oper op) : revop(op), state(s) { 
		// Edge(const PSVN &d, State &s, Oper op) : revop(op) { 

			// state.psvn_state = s.psvn_state;

			// fprintf(stdout, "Before forward rule: ");
			// print_state(stdout, &state.psvn_state);
			// fprintf(stdout, "\n");
			state_t temp = state.psvn_state;
			apply_fwd_rule( op, &temp, &state.psvn_state );

			// fprintf(stdout, "After forward rule: ");
			// print_state(stdout, &state.psvn_state);
			// fprintf(stdout, "\n");

			cost = get_fwd_rule_cost( op );
		}

		// The destructor is expected to undo any changes
		// that it may have made to the input state in
		// the constructor.  If a domain uses out-of-place
		// modification then the destructor may not be
		// required.
		~Edge(void) {
			state_t temp = state.psvn_state;
			apply_bwd_rule( revop, &temp, &state.psvn_state );
			revcost = get_bwd_rule_cost( revop );
		}
	};

	// Pack the state into the destination packed state.
	// If PackedState is the same type as State then this
	// should at least copy.
	void pack(PackedState &dst, State &src) const {
		dst = src;
	}

	// Unpack the state and return a reference to the
	// resulting unpacked state.  If PackedState and
	// State are the same type then the packed state
	// can just be immediately returned and used
	// so that there is no need to copy.
	State &unpack(State &buf, PackedState &pkd) const {
		return pkd;
	}

	// Print the state.
	void dumpstate(FILE *out, const State &s) const {
		print_state( out, &s.psvn_state );
	}

	Cost pathcost(const std::vector<State>&, const std::vector<Oper>&);

	private:
		state_t init_state;
		state_map_t* pdb;
		abstraction_t* abst;  // abstraction rule 
};
