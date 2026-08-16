#
# compiler
#
CC        = clang++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3 -DNDEBUG
CXXFLAGS 		= -Wall -Wextra -std=c++11 

#
# scots 
#
SCOTSROOT 		= /home/cublab/Workspace-Linux/unitree_go2_dev/scots
SCOTSINC		= -I$(SCOTSROOT)/utils -I$(SCOTSROOT)/bdd 

#
# cudd 
#
CUDDPATH		=  $(SCOTSROOT)/cudd-3.0.0
CUDDINC 		= -I$(CUDDPATH)
CUDDLIBS		= -lcudd 
CUDDLPATH   		= -L$(CUDDPATH)/lib

#
# pybind11 / python
#
PYTHONINC = -I/home/cublab/.local/lib/python3.8/site-packages/pybind11/include \
            $(shell python3-config --includes)


PYTHONLIBS = $(shell python3-config --ldflags --embed)

TARGET = SymbolicSetPython

all: $(TARGET).so

%.o:%.cc
	$(CC) -fPIC -c $(CXXFLAGS) $(CUDDINC) $(SCOTSINC) $(PYTHONINC) $< -o $@
	

$(TARGET).so: $(TARGET).o
	$(CC) -shared -fPIC $(CXXFLAGS) -o $(TARGET).so $(TARGET).o $(CUDDLPATH) $(CUDDLIBS) $(PYTHONLIBS)


clean:
	rm  ./$(TARGET).so  ./$(TARGET).o
