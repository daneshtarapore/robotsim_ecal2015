INCLUDES = -I. -I/usr/X11R6/include/ -std=c++0x

#SUBDIRS = smovableobjects fitnessfunctions sensors renders     \
#	  controllers actuators arenas populations experiments \
#	  collisions commoninterface emitters

bin_PROGRAMS  = bioinstsim2
bioinstsim2_SOURCES = main.cpp simobject.cpp agent.cpp simulator.cpp arena.cpp bioinstsim.cpp random.cpp circulararena.cpp rectangulararena.cpp boundlessarena.cpp common.cpp openglrender.cpp render.cpp arguments.cpp experiment.cpp populationanalyzer.cpp robotagent.cpp behavior.cpp aggregatebehavior.cpp testexperiment.cpp dispersebehavior.cpp flockbehavior.cpp homingbehavior.cpp randomwalkbehavior.cpp featurevector.cpp

bioinstsim2_LDADD   = -lGL -lGLU -lX11  -L/usr/X11R6/lib -L/usr/lib 

EXTRA_DIST = main.h simobject.h agent.h simulator.h arena.h bioinstsim.h random.h circulararena.h rectangulararena.h boundlessarena.h common.h openglrender.h render.h arguments.h experiment.h populationanalyzer.h robotagent.h behavior.h aggregatebehavior.h testexperiment.h dispersebehavior.h flockbehavior.h homingbehavior.h randomwalkbehavior.h featurevector.h

