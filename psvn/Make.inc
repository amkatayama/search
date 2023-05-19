psvn:\
	psvn/psvn_solver\

PSVNOBJS=\
    psvn/psvn.o

include $(PSVNOBJS:.o=.d)
CLEAN+=$(PSVNOBJS)

EVERYTHING+=psvn

psvn/psvn_solver:\
	$(PSVNOBJS)\
	psvn/main.o\
	utils/utils.a\
	structs/structs.a\
	search/search.a
	@echo $@
	@$(CXX) $^ $(LDFLAGS) -o $@

include psvn/main.d
CLEAN+=psvn/psvn_solver psvn/main.o