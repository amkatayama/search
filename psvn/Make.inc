psvn: psvn/psvn2c\ 

PSVNOBJS=\
    psvn/psvn.o

PSVNOPT = --fwd_history_len=3 --bwd_history_len=0  --backwards_moves --state_map
OPT = -g -Wall -O3

psvn/psvn2c:
	$(CXX) $(OPT) psvn/psvn2c.cpp psvn/psvn_base.cpp -o $@

# generate c code from the abstracted psvn 
# when I generate c code from abstracted psvn initial heuristics becomes 0
# when abstracted psvn file is used the first state becomes the goal state 
psvn/$(DOMAIN).c: psvn/$(DOMAIN).psvn psvn/psvn2c
	@cp psvn/psvn2c*.c .
	@psvn/psvn2c  $(PSVNOPT) --name=$(*F)  < $< > $@
	@rm psvn2c*.c

psvn/abst_$(DOMAIN).c: psvn/abst_$(DOMAIN).psvn psvn/psvn2c
	@cp psvn/psvn2c*.c .
	@psvn/psvn2c  $(PSVNOPT) --name=$(*F)  < $< > $@
	@rm psvn2c*.c

include $(PSVNOBJS:.o=.d)
CLEAN+=$(PSVNOBJS)

EVERYTHING+=psvn

IGNOREFLAGS = -Wno-unused-function -Wno-unused-variable

psvn/abstractor: psvn/abstractor.cpp psvn/psvn_base.cpp psvn/psvn_base.hpp
	$(CXX) $(OPT) psvn/abstractor.cpp psvn/psvn_base.cpp $(IGNOREFLAGS) -o $@
#	rm -f ./psvn.cpp ./psvn.hpp

psvn/abst_$(DOMAIN).dist: psvn/abst_$(DOMAIN).c psvn/dist.cpp psvn/priority_queue.hpp
	$(CXX) $(OPT) psvn/dist.cpp -include $< $(IGNOREFLAGS) -o $@
#	rm -f psvn/priority_queue.hpp

# executes build sequentially 
psvn/$(DOMAIN)_domain: 
	@echo $@ 
	@make psvn/abstractor
	@psvn/abstractor psvn/$(DOMAIN).psvn psvn/abst_$(DOMAIN) < psvn/abst_$(DOMAIN).txt
# 	prefix for dist has to be the same as the prefix given in the second argument of abstractor.cpp
	@make psvn/abst_$(DOMAIN).dist
	@psvn/abst_$(DOMAIN).dist psvn/abst_$(DOMAIN).state_map
	@$(MAKE) -s $(@:_domain=_solver)

psvn/%.o: 
	@echo $@
	@$(CXX) -include psvn/$(DOMAIN).c -DABSTFILE=\"psvn/abst_$(DOMAIN).abst\" -DPDBFILE=\"psvn/abst_$(DOMAIN).state_map\" $(IGNOREFLAGS) -c $(CXXFLAGS) -o $@ $<

psvn/%_solver: psvn/$(DOMAIN).c\
	psvn/psvn.o\
	psvn/main.o\
	utils/utils.a\
	structs/structs.a\
	search/search.a
	@echo $@
	@$(CXX) $^ $(LDFLAGS) $(IGNOREFLAGS) -o $@
	@rm psvn/$(DOMAIN).c
	@rm psvn/abst_$(DOMAIN).c
	@rm psvn/psvn.o
	@rm psvn/main.o
	@rm psvn/abst_$(DOMAIN).dist

include psvn/main.d
CLEAN+=psvn/main.o psvn/psvn2c psvn/*_solver psvn/abstractor 