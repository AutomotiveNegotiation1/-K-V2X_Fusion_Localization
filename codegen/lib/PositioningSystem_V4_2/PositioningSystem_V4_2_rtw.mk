###########################################################################
## Makefile generated for component 'PositioningSystem_V4_2'. 
## 
## Makefile     : PositioningSystem_V4_2_rtw.mk
## Generated on : Fri Jun 21 16:19:56 2024
## Final product: ./PositioningSystem_V4_2.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPILER_COMMAND_FILE   Compiler command listing model reference header paths
# CMD_FILE                Command file
# MODELLIB                Static library target

PRODUCT_NAME              = PositioningSystem_V4_2
MAKEFILE                  = PositioningSystem_V4_2_rtw.mk
MATLAB_ROOT               = C:/PROGRA~1/MATLAB/R2023b
MATLAB_BIN                = C:/PROGRA~1/MATLAB/R2023b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = D:/MATLAB/Positioning_Alg_20240601
TGT_FCN_LIB               = ISO_C
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = ../../..
COMPILER_COMMAND_FILE     = PositioningSystem_V4_2_rtw_comp.rsp
CMD_FILE                  = PositioningSystem_V4_2_rtw.rsp
C_STANDARD_OPTS           = -fwrapv
CPP_STANDARD_OPTS         = -fwrapv
MODELLIB                  = PositioningSystem_V4_2.lib

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          MinGW64 | gmake (64-bit Windows)
# Supported Version(s):    8.x
# ToolchainInfo Version:   2023b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS
# MINGW_ROOT
# MINGW_C_STANDARD_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS            = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align
WARN_FLAGS_MAX        = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS        = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align
CPP_WARN_FLAGS_MAX    = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow
MW_EXTERNLIB_DIR      = $(MATLAB_ROOT)/extern/lib/win64/mingw64
SHELL                 = %SystemRoot%/system32/cmd.exe

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lws2_32

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC_PATH = $(MINGW_ROOT)
CC = "$(CC_PATH)/gcc"

# Linker: GNU Linker
LD_PATH = $(MINGW_ROOT)
LD = "$(LD_PATH)/g++"

# C++ Compiler: GNU C++ Compiler
CPP_PATH = $(MINGW_ROOT)
CPP = "$(CPP_PATH)/g++"

# C++ Linker: GNU C++ Linker
CPP_LD_PATH = $(MINGW_ROOT)
CPP_LD = "$(CPP_LD_PATH)/g++"

# Archiver: GNU Archiver
AR_PATH = $(MINGW_ROOT)
AR = "$(AR_PATH)/ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = $(MINGW_ROOT)
MAKE = "$(MAKE_PATH)/mingw32-make.exe"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @del
ECHO                = @echo
MV                  = @move
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(MINGW_C_STANDARD_OPTS) -m64 \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -m64 \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPP_LDFLAGS          =  -static -m64
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,--no-undefined \
                         -Wl,--out-implib,$(notdir $(basename $(PRODUCT))).lib
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              =  -static -m64
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,--no-undefined \
                       -Wl,--out-implib,$(notdir $(basename $(PRODUCT))).lib



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./PositioningSystem_V4_2.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = 

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__USE_MINGW_ANSI_STDIO=1
DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=PositioningSystem_V4_2

DEFINES = $(DEFINES_) $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_data.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/rt_nonfinite.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/rtGetNaN.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/rtGetInf.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_initialize.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_terminate.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/interp1.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mrdivide_helper.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/quat2eul.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/EKF_UWB_SLAM_4.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mod.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/UWBPosition_V4_1.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/UWBpos_V2_3.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/inv.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mldivide.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/abs.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/UWBMultiTagPos_V3_1.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/find.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mean.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/exp.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/nullAssignment.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/sort.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/sortIdx.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/div.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_emxutil.c $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_rtwutil.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = PositioningSystem_V4_2_data.obj rt_nonfinite.obj rtGetNaN.obj rtGetInf.obj PositioningSystem_V4_2_initialize.obj PositioningSystem_V4_2_terminate.obj PositioningSystem_V4_2.obj interp1.obj mrdivide_helper.obj quat2eul.obj EKF_UWB_SLAM_4.obj mod.obj UWBPosition_V4_1.obj UWBpos_V2_3.obj inv.obj mldivide.obj abs.obj UWBMultiTagPos_V3_1.obj find.obj mean.obj exp.obj nullAssignment.obj sort.obj sortIdx.obj div.obj PositioningSystem_V4_2_emxutil.obj PositioningSystem_V4_2_rtwutil.obj

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES) @$(COMPILER_COMMAND_FILE)

CPPFLAGS += $(CPPFLAGS_BASIC)

#---------------------
# MEX C++ Compiler
#---------------------

MEX_CPP_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CPPFLAGS += $(MEX_CPP_Compiler_BASIC)

#-----------------
# MEX Compiler
#-----------------

MEX_Compiler_BASIC =  @$(COMPILER_COMMAND_FILE)

MEX_CFLAGS += $(MEX_Compiler_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################


MINGW_C_STANDARD_OPTS = $(C_STANDARD_OPTS)


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) @$(CMD_FILE)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.obj : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : %.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(RELATIVE_PATH_TO_ANCHOR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.obj : $(START_DIR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


PositioningSystem_V4_2_data.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_data.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rt_nonfinite.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetNaN.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/rtGetNaN.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetInf.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/rtGetInf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


PositioningSystem_V4_2_initialize.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_initialize.c
	$(CC) $(CFLAGS) -o "$@" "$<"


PositioningSystem_V4_2_terminate.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_terminate.c
	$(CC) $(CFLAGS) -o "$@" "$<"


PositioningSystem_V4_2.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2.c
	$(CC) $(CFLAGS) -o "$@" "$<"


interp1.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/interp1.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mrdivide_helper.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mrdivide_helper.c
	$(CC) $(CFLAGS) -o "$@" "$<"


quat2eul.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/quat2eul.c
	$(CC) $(CFLAGS) -o "$@" "$<"


EKF_UWB_SLAM_4.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/EKF_UWB_SLAM_4.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mod.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mod.c
	$(CC) $(CFLAGS) -o "$@" "$<"


UWBPosition_V4_1.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/UWBPosition_V4_1.c
	$(CC) $(CFLAGS) -o "$@" "$<"


UWBpos_V2_3.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/UWBpos_V2_3.c
	$(CC) $(CFLAGS) -o "$@" "$<"


inv.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/inv.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mldivide.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mldivide.c
	$(CC) $(CFLAGS) -o "$@" "$<"


abs.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/abs.c
	$(CC) $(CFLAGS) -o "$@" "$<"


UWBMultiTagPos_V3_1.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/UWBMultiTagPos_V3_1.c
	$(CC) $(CFLAGS) -o "$@" "$<"


find.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/find.c
	$(CC) $(CFLAGS) -o "$@" "$<"


mean.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/mean.c
	$(CC) $(CFLAGS) -o "$@" "$<"


exp.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/exp.c
	$(CC) $(CFLAGS) -o "$@" "$<"


nullAssignment.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/nullAssignment.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sort.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/sort.c
	$(CC) $(CFLAGS) -o "$@" "$<"


sortIdx.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/sortIdx.c
	$(CC) $(CFLAGS) -o "$@" "$<"


div.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/div.c
	$(CC) $(CFLAGS) -o "$@" "$<"


PositioningSystem_V4_2_emxutil.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_emxutil.c
	$(CC) $(CFLAGS) -o "$@" "$<"


PositioningSystem_V4_2_rtwutil.obj : $(START_DIR)/codegen/lib/PositioningSystem_V4_2/PositioningSystem_V4_2_rtwutil.c
	$(CC) $(CFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(COMPILER_COMMAND_FILE) $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(subst /,\,$(PRODUCT))
	$(RM) $(subst /,\,$(ALL_OBJS))
	$(ECHO) "### Deleted all derived files."


