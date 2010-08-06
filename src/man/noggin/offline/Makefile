C++ = g++
C++-FLAGS = -Wall -O3 -DNDEBUG
RM = rm -f
INCLUDE = -I ../../include/ -I ../../vision/ -I ./../ \
	-I ./ -I /sw/include/ -I ../../

VPATH = ../../include : ../../vision : ../../noggin :

OBJS = $(INCLUDE_OBJS) \
       $(VISION_OBJS) \
       $(NOGGIN_OBJS) \
       $(OFFLINE_OBJS)

OFFLINE_OBJS = fakerIO.o \
	fakerIterators.o \

INCLUDE_OBJS = NBMath.o \
	NBMatrixMath.o \

NOGGIN_OBJS = Observation.o \
	MCL.o \
	BallEKF.o \
	MMLocEKF.o \
	LocEKF.o

VISION_OBJS = Utility.o \
	ConcreteLandmark.o \
	ConcreteCorner.o \
	ConcreteCross.o \
	ConcreteFieldObject.o \
	ConcreteLine.o \
	VisualDetection.o \
	VisualFieldObject.o \
	VisualCorner.o \
	VisualCross.o \
	VisualLine.o \
	VisualBall.o

FAKER_OBJS = faker.o
OBS_TO_LOC_OBJS = obsToLoc.o
NOISE_OBJS = noiseVaccuracy.o
ROBOT_LOG_OBJS = convertRobotLog.o

EXECS = faker.o \
	faker \
	navToObs.o \
	obsToLoc.o \
	navToObs \
	obsToLoc \
	noiseVaccuracy \
	convertRobotLog

NAV_TO_OBS_SRCS = navToObs.cpp \
	../NogginStructs.h

OBS_TO_LOC_SRCS = obsToLoc.cpp \
	../NogginStructs.h

FAKER_SRCS = LocLogFaker.cpp \
	../NogginStructs.h

NOISE_SRCS = noiseVaccuracy.cpp

ROBOT_LOG_SRCS = convertRobotLog.cpp

LDLIBS = $(OBJS)
LDFLAGS = $(LDLIBS)

all : convertRobotLog faker

# The faker tool
convertRobotLog : $(ROBOT_LOG_OBJS) $(OBJS) convertRobotLog.o
	$(C++) $(C++-FLAGS) $(INCLUDE) $(LDFLAGS) convertRobotLog.o -DNO_ZLIB -o $@

noiseVaccuracy : $(NOISE_SRCS) $(OBJS) noiseVaccuracy.o
	$(C++) $(C++-FLAGS) $(INCLUDE) $(LDFLAGS) noiseVaccuracy.o -DNO_ZLIB -o $@

faker : $(FAKER_SRCS) $(OBJS) $(FAKER_OBJS)
	$(C++) $(C++-FLAGS) $(INCLUDE) $(LDFLAGS) faker.o -DNO_ZLIB -o $@

obsToLoc : $(OBS_TO_LOC_OBJS) $(OBJS)
	$(C++) $(C++-FLAGS) $(INCLUDE) $(LDFLAGS) obsToLoc.o -DNO_ZLIB -o $@

navToObs : $(NAV_TO_OBS_SRCS) $(OBJS) navToObs.o
	$(C++) $(C++-FLAGS) $(INCLUDE) $(LDFLAGS) navToObs.o -DNO_ZLIB -o $@

$(OBJS): %.o : %.cpp %.h
	$(C++) $(C++-FLAGS) $(INCLUDE) -c $< -o $@

$(FAKER_OBJS): $(FAKER_SRCS)
	$(C++) $(C++-FLAGS) $(INCLUDE) -c $< -o $@

$(OBS_TO_LOC_OBJS) : $(OBS_TO_LOC_SRCS)
	$(C++) $(C++-FLAGS) $(INCLUDE) -c $< -o $@

$(NOISE_OBJS) : $(NOISE_SRCS)
	$(C++) $(C++-FLAGS) $(INCLUDE) -c $< -o $@

$(ROBOT_LOG_OBJS) : $(ROBOT_LOG_SRCS)
	$(C++) $(C++-FLAGS) $(INCLUDE) -c $< -o $@

.PHONY : clean

clean :
	$(RM) $(OBJS) $(EXECS)
