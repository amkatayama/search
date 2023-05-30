psvn: psvn/psvn2c\

PSVNOBJS=\
    psvn/psvn.o

PSVNOPT = --no_state_map --no_abstraction --no_backwards_moves --history_len=0

OPT = -g -Wall -O3

psvn/psvn2c:
	$(CXX) $(OPT) psvn/psvn2c.cpp psvn/psvn_base.cpp -o $@

psvn/%.c: psvn/%.psvn psvn/psvn2c
	@cp psvn/psvn2c*.c .
	@psvn/psvn2c  $(PSVNOPT) --name=$(*F)  < $< > psvn/psvn-domain.c
	@cp psvn/psvn-domain.c $@
	@rm psvn2c*.c

include $(PSVNOBJS:.o=.d)
CLEAN+=$(PSVNOBJS)

EVERYTHING+=psvn

IGNOREFLAGS = -Wno-unused-function -Wno-unused-variable

psvn/%.o:
	@echo $@
	@$(CXX) -include psvn/psvn-domain.c $(IGNOREFLAGS) -c $(CXXFLAGS) -o $@ $<

psvn/%_solver: psvn/%.c\
	psvn/psvn.o\
	psvn/main.o\
	utils/utils.a\
	structs/structs.a\
	search/search.a
	@echo $@
	@$(CXX) $^ $(LDFLAGS) $(IGNOREFLAGS) -o $@
	@rm psvn/psvn-domain.c
	@rm $<
	@rm psvn/psvn.o
	@rm psvn/main.o

include psvn/main.d
CLEAN+=psvn/main.o psvn/psvn2c psvn/*_solver