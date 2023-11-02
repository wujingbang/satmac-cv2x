#
# OMNeT++/OMNEST Makefile for satmac
#
# This file was generated with the command:
#  opp_makemake -f --deep -O out -KINET_3_6_6_PROJ=/home/cxy/OpenCV2X/inet-3.6.6 -KVEINS_INET3_PROJ=/home/cxy/OpenCV2X/veins-5.0/subprojects/veins_inet3 -KVEINS_5_0_PROJ=/home/cxy/OpenCV2X/veins-5.0 -DINET_IMPORT -DVEINS_INET_IMPORT -DVEINS_IMPORT -I. -I$$\(INET_3_6_6_PROJ\)/src -I$$\(VEINS_INET3_PROJ\)/src -I$$\(VEINS_5_0_PROJ\)/src -Isrc -Lsrc/common/geohash/proj/64bit/lib -L$$\(INET_3_6_6_PROJ\)/src -L$$\(VEINS_INET3_PROJ\)/src -L$$\(VEINS_5_0_PROJ\)/src -lINET$$\(D\) -lveins_inet$$\(D\) -lveins$$\(D\) -d src -Xsrc
#

# Name of target to be created (-o option)
TARGET = satmac$(D)$(EXE_SUFFIX)
TARGET_DIR = .

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(QTENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)
#USERIF_LIBS = $(QTENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = -I. -I$(INET_3_6_6_PROJ)/src -I$(VEINS_INET3_PROJ)/src -I$(VEINS_5_0_PROJ)/src -Isrc

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = $(LDFLAG_LIBPATH)src/common/geohash/proj/64bit/lib $(LDFLAG_LIBPATH)$(INET_3_6_6_PROJ)/src $(LDFLAG_LIBPATH)$(VEINS_INET3_PROJ)/src $(LDFLAG_LIBPATH)$(VEINS_5_0_PROJ)/src  -lINET$(D) -lveins_inet$(D) -lveins$(D)

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS =

# Message files
MSGFILES =

# SM files
SMFILES =

# Other makefile variables (-K)
INET_3_6_6_PROJ=/home/cxy/OpenCV2X/inet-3.6.6
VEINS_INET3_PROJ=/home/cxy/OpenCV2X/veins-5.0/subprojects/veins_inet3
VEINS_5_0_PROJ=/home/cxy/OpenCV2X/veins-5.0

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIBS = $(OPPMAIN_LIB) $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)
ifneq ($(TOOLCHAIN_NAME),clangc2)
LIBS += -Wl,-rpath,$(abspath src/common/geohash/proj/64bit/lib) -Wl,-rpath,$(abspath $(INET_3_6_6_PROJ)/src) -Wl,-rpath,$(abspath $(VEINS_INET3_PROJ)/src) -Wl,-rpath,$(abspath $(VEINS_5_0_PROJ)/src)
endif

COPTS = $(CFLAGS) $(IMPORT_DEFINES) -DINET_IMPORT -DVEINS_INET_IMPORT -DVEINS_IMPORT $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $(TARGET_DIR)/$(TARGET)

$(TARGET_DIR)/% :: $O/%
	@mkdir -p $(TARGET_DIR)
	$(Q)$(LN) $< $@
ifeq ($(TOOLCHAIN_NAME),clangc2)
	$(Q)-$(LN) $(<:%.dll=%.lib) $(@:%.dll=%.lib)
endif

$O/$(TARGET): $(OBJS) submakedirs $(wildcard $(EXTRA_OBJS)) Makefile $(CONFIGFILE)
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET) $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

submakedirs:  src_dir

.PHONY: all clean cleanall depend msgheaders smheaders  src
src: src_dir

src_dir:
	cd src && $(MAKE) all

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE) | msgheaders smheaders
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc -MD -MP -MF $O/$(basename $<)_m.h.d $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)
	$(Q)cd src && $(MAKE) msgheaders

smheaders: $(SMFILES:.sm=_sm.h)
	$(Q)cd src && $(MAKE) smheaders

clean:
	$(qecho) Cleaning $(TARGET)
	$(Q)-rm -rf $O
	$(Q)-rm -f $(TARGET_DIR)/$(TARGET)
	$(Q)-rm -f $(TARGET_DIR)/$(TARGET:%.dll=%.lib)
	$(Q)-rm -f $(call opp_rwildcard, . , *_m.cc *_m.h *_sm.cc *_sm.h)
	-$(Q)cd src && $(MAKE) clean

cleanall:
	$(Q)$(MAKE) -s clean MODE=release
	$(Q)$(MAKE) -s clean MODE=debug
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

# include all dependencies
-include $(OBJS:%=%.d) $(MSGFILES:%.msg=$O/%_m.h.d)