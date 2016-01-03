#
# OMNeT++/OMNEST Makefile for libveins
#
# This file was generated with the command:
#  opp_makemake -f --deep --make-so -O out -I../../inet/src/linklayer/ieee80211/radio -I../../inet/src/networklayer/routing/aodv -I../../inet/src/networklayer/common -I../../inet/src/networklayer/icmpv6 -I../../inet/src -I../../inet/src/world/obstacles -I../../inet/src/networklayer/xmipv6 -I../../inet/src/networklayer/contract -I../../inet/src/networklayer/autorouting/ipv4 -I../../inet/src/util -I../../inet/src/linklayer/common -I../../inet/src/transport/contract -I../../inet/src/status -I../../inet/src/linklayer/radio/propagation -I../../inet/src/linklayer/ieee80211/radio/errormodel -I../../inet/src/linklayer/radio -I../../inet/src/util/headerserializers/tcp -I../../inet/src/networklayer/ipv4 -I../../inet/src/mobility/contract -I../../inet/src/util/headerserializers/ipv4 -I../../inet/src/base -I../../inet/src/util/headerserializers -I../../inet/src/world/radio -I../../inet/src/linklayer/ieee80211/mac -I../../inet/src/networklayer/ipv6 -I../../inet/src/transport/sctp -I../../inet/src/util/headerserializers/udp -I../../inet/src/networklayer/ipv6tunneling -I../../inet/src/applications/pingapp -I../../inet/src/battery/models -I../../inet/src/util/headerserializers/ipv6 -I../../inet/src/util/headerserializers/sctp -I../../inet/src/linklayer/contract -I../../inet/src/networklayer/arp -I../../inet/src/transport/tcp_common -I../../inet/src/transport/udp -I../../inet/src/world/traci -I../../inet/src/networklayer/routing/gpsr -L../../inet/out/$$\(CONFIGNAME\)/src -L../../lte/out/$$\(CONFIGNAME\)/src -linet -llte -DINET_IMPORT -KINET_PROJ=../../inet -KLTE_PROJ=../../lte
#

# Name of target to be created (-o option)
TARGET = libveins$(SHARED_LIB_SUFFIX)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I../../inet/src/linklayer/ieee80211/radio \
    -I../../inet/src/networklayer/routing/aodv \
    -I../../inet/src/networklayer/common \
    -I../../inet/src/networklayer/icmpv6 \
    -I../../inet/src \
    -I../../inet/src/world/obstacles \
    -I../../inet/src/networklayer/xmipv6 \
    -I../../inet/src/networklayer/contract \
    -I../../inet/src/networklayer/autorouting/ipv4 \
    -I../../inet/src/util \
    -I../../inet/src/linklayer/common \
    -I../../inet/src/transport/contract \
    -I../../inet/src/status \
    -I../../inet/src/linklayer/radio/propagation \
    -I../../inet/src/linklayer/ieee80211/radio/errormodel \
    -I../../inet/src/linklayer/radio \
    -I../../inet/src/util/headerserializers/tcp \
    -I../../inet/src/networklayer/ipv4 \
    -I../../inet/src/mobility/contract \
    -I../../inet/src/util/headerserializers/ipv4 \
    -I../../inet/src/base \
    -I../../inet/src/util/headerserializers \
    -I../../inet/src/world/radio \
    -I../../inet/src/linklayer/ieee80211/mac \
    -I../../inet/src/networklayer/ipv6 \
    -I../../inet/src/transport/sctp \
    -I../../inet/src/util/headerserializers/udp \
    -I../../inet/src/networklayer/ipv6tunneling \
    -I../../inet/src/applications/pingapp \
    -I../../inet/src/battery/models \
    -I../../inet/src/util/headerserializers/ipv6 \
    -I../../inet/src/util/headerserializers/sctp \
    -I../../inet/src/linklayer/contract \
    -I../../inet/src/networklayer/arp \
    -I../../inet/src/transport/tcp_common \
    -I../../inet/src/transport/udp \
    -I../../inet/src/world/traci \
    -I../../inet/src/networklayer/routing/gpsr \
    -I. \
    -Ibase \
    -Ibase/connectionManager \
    -Ibase/messages \
    -Ibase/modules \
    -Ibase/phyLayer \
    -Ibase/utils \
    -Iinet \
    -Imodules \
    -Imodules/analogueModel \
    -Imodules/application \
    -Imodules/application/ieee80211p \
    -Imodules/application/traci \
    -Imodules/mac \
    -Imodules/mac/ieee80211p \
    -Imodules/messages \
    -Imodules/mobility \
    -Imodules/mobility/traci \
    -Imodules/nic \
    -Imodules/obstacle \
    -Imodules/phy \
    -Imodules/routing \
    -Imodules/utility \
    -Imodules/world \
    -Imodules/world/annotations \
    -Inodes \
    -Iscripts

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L../../inet/out/$(CONFIGNAME)/src -L../../lte/out/$(CONFIGNAME)/src  -linet -llte
LIBS += -Wl,-rpath,`abspath ../../inet/out/$(CONFIGNAME)/src` -Wl,-rpath,`abspath ../../lte/out/$(CONFIGNAME)/src`

# Output directory
PROJECT_OUTPUT_DIR = ../out
PROJECTRELATIVE_PATH = src
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS = \
    $O/base/connectionManager/ConnectionManager.o \
    $O/base/connectionManager/BaseConnectionManager.o \
    $O/base/connectionManager/NicEntryDirect.o \
    $O/base/connectionManager/NicEntryDebug.o \
    $O/base/connectionManager/ChannelAccess.o \
    $O/base/modules/BatteryAccess.o \
    $O/base/modules/IMobility.o \
    $O/base/modules/BaseBattery.o \
    $O/base/modules/BaseWorldUtility.o \
    $O/base/modules/BaseMacLayer.o \
    $O/base/modules/BaseLayer.o \
    $O/base/modules/BaseApplLayer.o \
    $O/base/modules/BaseMobility.o \
    $O/base/modules/BaseModule.o \
    $O/base/phyLayer/BaseDecider.o \
    $O/base/phyLayer/MappingBase.o \
    $O/base/phyLayer/ChannelInfo.o \
    $O/base/phyLayer/MappingUtils.o \
    $O/base/phyLayer/Signal.o \
    $O/base/phyLayer/ChannelState.o \
    $O/base/phyLayer/BasePhyLayer.o \
    $O/base/phyLayer/Decider.o \
    $O/base/phyLayer/PhyUtils.o \
    $O/base/utils/asserts.o \
    $O/base/utils/winsupport.o \
    $O/base/utils/NetwToMacControlInfo.o \
    $O/base/utils/SimpleAddress.o \
    $O/inet/Coord.o \
    $O/modules/analogueModel/SimpleObstacleShadowing.o \
    $O/modules/analogueModel/BreakpointPathlossModel.o \
    $O/modules/analogueModel/TwoRayInterferenceModel.o \
    $O/modules/analogueModel/PERModel.o \
    $O/modules/analogueModel/JakesFading.o \
    $O/modules/analogueModel/SimplePathlossModel.o \
    $O/modules/analogueModel/LogNormalShadowing.o \
    $O/modules/application/ieee80211p/BaseWaveApplLayer.o \
    $O/modules/application/traci/TraCIDemoRSU11p.o \
    $O/modules/application/traci/TraCITestApp.o \
    $O/modules/application/traci/TraCIDemo11p.o \
    $O/modules/mac/ieee80211p/Mac1609_4.o \
    $O/modules/mobility/traci/TraCIColor.o \
    $O/modules/mobility/traci/TraCIMobility.o \
    $O/modules/mobility/traci/TraCIScenarioManagerLaunchd.o \
    $O/modules/mobility/traci/TraCIScenarioManagerInet.o \
    $O/modules/mobility/traci/TraCIBuffer.o \
    $O/modules/mobility/traci/TraCIScenarioManager.o \
    $O/modules/mobility/traci/TraCICommandInterface.o \
    $O/modules/mobility/traci/TraCIConnection.o \
    $O/modules/obstacle/Obstacle.o \
    $O/modules/obstacle/ObstacleControl.o \
    $O/modules/phy/SNRThresholdDecider.o \
    $O/modules/phy/PhyLayer80211p.o \
    $O/modules/phy/Decider80211p.o \
    $O/modules/routing/GPSR.o \
    $O/modules/world/annotations/AnnotationManager.o \
    $O/modules/world/annotations/AnnotationDummy.o \
    $O/base/messages/MacPkt_m.o \
    $O/base/messages/BorderMsg_m.o \
    $O/base/messages/AirFrame_m.o \
    $O/base/messages/ChannelSenseRequest_m.o \
    $O/modules/messages/Mac80211Pkt_m.o \
    $O/modules/messages/WaveShortMessage_m.o \
    $O/modules/messages/AirFrame11p_m.o \
    $O/modules/routing/GPSR_m.o

# Message files
MSGFILES = \
    base/messages/MacPkt.msg \
    base/messages/BorderMsg.msg \
    base/messages/AirFrame.msg \
    base/messages/ChannelSenseRequest.msg \
    modules/messages/Mac80211Pkt.msg \
    modules/messages/WaveShortMessage.msg \
    modules/messages/AirFrame11p.msg \
    modules/routing/GPSR.msg

# Other makefile variables (-K)
INET_PROJ=../../inet
LTE_PROJ=../../lte

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

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
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppenvir$D $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS) -DINET_IMPORT $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating shared library: $@
	$(Q)$(SHLIB_LD) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS) $(LDFLAGS)
	$(Q)$(SHLIB_POSTPROCESS) $O/$(TARGET)

.PHONY: all clean cleanall depend msgheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f veins veins.exe libveins.so libveins.a libveins.dll libveins.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h
	$(Q)-rm -f base/*_m.cc base/*_m.h
	$(Q)-rm -f base/connectionManager/*_m.cc base/connectionManager/*_m.h
	$(Q)-rm -f base/messages/*_m.cc base/messages/*_m.h
	$(Q)-rm -f base/modules/*_m.cc base/modules/*_m.h
	$(Q)-rm -f base/phyLayer/*_m.cc base/phyLayer/*_m.h
	$(Q)-rm -f base/utils/*_m.cc base/utils/*_m.h
	$(Q)-rm -f inet/*_m.cc inet/*_m.h
	$(Q)-rm -f modules/*_m.cc modules/*_m.h
	$(Q)-rm -f modules/analogueModel/*_m.cc modules/analogueModel/*_m.h
	$(Q)-rm -f modules/application/*_m.cc modules/application/*_m.h
	$(Q)-rm -f modules/application/ieee80211p/*_m.cc modules/application/ieee80211p/*_m.h
	$(Q)-rm -f modules/application/traci/*_m.cc modules/application/traci/*_m.h
	$(Q)-rm -f modules/mac/*_m.cc modules/mac/*_m.h
	$(Q)-rm -f modules/mac/ieee80211p/*_m.cc modules/mac/ieee80211p/*_m.h
	$(Q)-rm -f modules/messages/*_m.cc modules/messages/*_m.h
	$(Q)-rm -f modules/mobility/*_m.cc modules/mobility/*_m.h
	$(Q)-rm -f modules/mobility/traci/*_m.cc modules/mobility/traci/*_m.h
	$(Q)-rm -f modules/nic/*_m.cc modules/nic/*_m.h
	$(Q)-rm -f modules/obstacle/*_m.cc modules/obstacle/*_m.h
	$(Q)-rm -f modules/phy/*_m.cc modules/phy/*_m.h
	$(Q)-rm -f modules/routing/*_m.cc modules/routing/*_m.h
	$(Q)-rm -f modules/utility/*_m.cc modules/utility/*_m.h
	$(Q)-rm -f modules/world/*_m.cc modules/world/*_m.h
	$(Q)-rm -f modules/world/annotations/*_m.cc modules/world/annotations/*_m.h
	$(Q)-rm -f nodes/*_m.cc nodes/*_m.h
	$(Q)-rm -f scripts/*_m.cc scripts/*_m.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc base/*.cc base/connectionManager/*.cc base/messages/*.cc base/modules/*.cc base/phyLayer/*.cc base/utils/*.cc inet/*.cc modules/*.cc modules/analogueModel/*.cc modules/application/*.cc modules/application/ieee80211p/*.cc modules/application/traci/*.cc modules/mac/*.cc modules/mac/ieee80211p/*.cc modules/messages/*.cc modules/mobility/*.cc modules/mobility/traci/*.cc modules/nic/*.cc modules/obstacle/*.cc modules/phy/*.cc modules/routing/*.cc modules/utility/*.cc modules/world/*.cc modules/world/annotations/*.cc nodes/*.cc scripts/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/base/connectionManager/BaseConnectionManager.o: base/connectionManager/BaseConnectionManager.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/connectionManager/NicEntryDebug.h \
	base/connectionManager/NicEntryDirect.h \
	base/modules/BaseWorldUtility.h \
	base/utils/FindModule.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/connectionManager/ChannelAccess.o: base/connectionManager/ChannelAccess.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/ChannelAccess.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/connectionManager/ConnectionManager.o: base/connectionManager/ConnectionManager.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/ConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseWorldUtility.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/connectionManager/NicEntryDebug.o: base/connectionManager/NicEntryDebug.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/ChannelAccess.h \
	base/connectionManager/NicEntry.h \
	base/connectionManager/NicEntryDebug.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/connectionManager/NicEntryDirect.o: base/connectionManager/NicEntryDirect.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/ChannelAccess.h \
	base/connectionManager/NicEntry.h \
	base/connectionManager/NicEntryDirect.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/messages/AirFrame_m.o: base/messages/AirFrame_m.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/linklayer/radio/AirFrame_m.h \
	$(INET_PROJ)/src/linklayer/radio/ModulationType.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/messages/BorderMsg_m.o: base/messages/BorderMsg_m.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/BorderMsg_m.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/messages/ChannelSenseRequest_m.o: base/messages/ChannelSenseRequest_m.cc \
	base/messages/ChannelSenseRequest_m.h \
	base/phyLayer/ChannelState.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/messages/MacPkt_m.o: base/messages/MacPkt_m.cc \
	base/messages/MacPkt_m.h \
	base/utils/MiXiMDefs.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h
$O/base/modules/BaseApplLayer.o: base/modules/BaseApplLayer.cc \
	base/modules/BaseApplLayer.h \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseModule.h \
	base/modules/BatteryAccess.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/PassedMessage.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h
$O/base/modules/BaseBattery.o: base/modules/BaseBattery.cc \
	base/modules/BaseBattery.h \
	base/modules/BaseModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/modules/BaseLayer.o: base/modules/BaseLayer.cc \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseModule.h \
	base/modules/BatteryAccess.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/PassedMessage.h \
	base/utils/miximkerneldefs.h
$O/base/modules/BaseMacLayer.o: base/modules/BaseMacLayer.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/networklayer/common/IInterfaceTable.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceEntry.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceTableAccess.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceToken.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPvXAddress.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/ChannelAccess.h \
	base/messages/MacPkt_m.h \
	base/modules/AddressingInterface.h \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseMacLayer.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/MacToPhyControlInfo.h \
	base/phyLayer/MacToPhyInterface.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/PhyUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MacToNetwControlInfo.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/NetwToMacControlInfo.h \
	base/utils/PassedMessage.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/modules/BaseMobility.o: base/modules/BaseMobility.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/BorderMsg_m.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/modules/BaseModule.o: base/modules/BaseModule.cc \
	base/modules/BaseModule.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/modules/BaseWorldUtility.o: base/modules/BaseWorldUtility.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseWorldUtility.h \
	base/utils/FindModule.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/modules/BatteryAccess.o: base/modules/BatteryAccess.cc \
	base/modules/BaseBattery.h \
	base/modules/BaseModule.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/modules/IMobility.o: base/modules/IMobility.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/modules/IMobility.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/phyLayer/BaseDecider.o: base/phyLayer/BaseDecider.cc \
	base/messages/AirFrame_m.h \
	base/messages/ChannelSenseRequest_m.h \
	base/phyLayer/BaseDecider.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Decider.h \
	base/phyLayer/DeciderToPhyInterface.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/phyLayer/BasePhyLayer.o: base/phyLayer/BasePhyLayer.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/ChannelAccess.h \
	base/connectionManager/NicEntry.h \
	base/messages/AirFrame_m.h \
	base/messages/ChannelSenseRequest_m.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/BasePhyLayer.h \
	base/phyLayer/ChannelInfo.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Decider.h \
	base/phyLayer/DeciderToPhyInterface.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/MacToPhyControlInfo.h \
	base/phyLayer/MacToPhyInterface.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/PhyToMacControlInfo.h \
	base/phyLayer/PhyUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/phyLayer/ChannelInfo.o: base/phyLayer/ChannelInfo.cc \
	base/messages/AirFrame_m.h \
	base/phyLayer/ChannelInfo.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/phyLayer/ChannelState.o: base/phyLayer/ChannelState.cc \
	base/phyLayer/ChannelState.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/phyLayer/Decider.o: base/phyLayer/Decider.cc \
	base/messages/ChannelSenseRequest_m.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Decider.h \
	base/phyLayer/DeciderToPhyInterface.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/phyLayer/MappingBase.o: base/phyLayer/MappingBase.cc \
	base/phyLayer/Interpolation.h \
	base/phyLayer/MappingBase.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/phyLayer/MappingUtils.o: base/phyLayer/MappingUtils.cc \
	base/phyLayer/Interpolation.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/phyLayer/PhyUtils.o: base/phyLayer/PhyUtils.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/PhyUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/base/phyLayer/Signal.o: base/phyLayer/Signal.cc \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h
$O/base/utils/NetwToMacControlInfo.o: base/utils/NetwToMacControlInfo.cc \
	base/utils/MiXiMDefs.h \
	base/utils/NetwToMacControlInfo.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h
$O/base/utils/SimpleAddress.o: base/utils/SimpleAddress.cc \
	base/utils/MiXiMDefs.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h
$O/base/utils/asserts.o: base/utils/asserts.cc \
	base/utils/asserts.h
$O/base/utils/winsupport.o: base/utils/winsupport.cc \
	base/utils/winsupport.h
$O/inet/Coord.o: inet/Coord.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/modules/analogueModel/BreakpointPathlossModel.o: modules/analogueModel/BreakpointPathlossModel.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/BreakpointPathlossModel.h
$O/modules/analogueModel/JakesFading.o: modules/analogueModel/JakesFading.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/ChannelAccess.h \
	base/messages/AirFrame_m.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/JakesFading.h
$O/modules/analogueModel/LogNormalShadowing.o: modules/analogueModel/LogNormalShadowing.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/LogNormalShadowing.h
$O/modules/analogueModel/PERModel.o: modules/analogueModel/PERModel.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/PERModel.h
$O/modules/analogueModel/SimpleObstacleShadowing.o: modules/analogueModel/SimpleObstacleShadowing.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/modules/BaseWorldUtility.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/FindModule.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/SimpleObstacleShadowing.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/analogueModel/SimplePathlossModel.o: modules/analogueModel/SimplePathlossModel.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/modules/BaseWorldUtility.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/SimplePathlossModel.h
$O/modules/analogueModel/TwoRayInterferenceModel.o: modules/analogueModel/TwoRayInterferenceModel.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/modules/BaseWorldUtility.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/TwoRayInterferenceModel.h
$O/modules/application/ieee80211p/BaseWaveApplLayer.o: modules/application/ieee80211p/BaseWaveApplLayer.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/networklayer/arp/ARPPacket_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPProtocolId_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/ChannelAccess.h \
	base/modules/BaseApplLayer.h \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/NetwToMacControlInfo.h \
	base/utils/PassedMessage.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/application/ieee80211p/BaseWaveApplLayer.h \
	modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	modules/messages/WaveShortMessage_m.h \
	modules/utility/Consts80211p.h
$O/modules/application/traci/TraCIDemo11p.o: modules/application/traci/TraCIDemo11p.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/networklayer/arp/ARPPacket_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPProtocolId_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/ChannelAccess.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseApplLayer.h \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/NetwToMacControlInfo.h \
	base/utils/PassedMessage.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/application/ieee80211p/BaseWaveApplLayer.h \
	modules/application/traci/TraCIDemo11p.h \
	modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	modules/messages/WaveShortMessage_m.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIColor.h \
	modules/mobility/traci/TraCICommandInterface.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCICoord.h \
	modules/mobility/traci/TraCIMobility.h \
	modules/mobility/traci/TraCIScenarioManager.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/utility/Consts80211p.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/application/traci/TraCIDemoRSU11p.o: modules/application/traci/TraCIDemoRSU11p.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/networklayer/arp/ARPPacket_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPProtocolId_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/ChannelAccess.h \
	base/modules/BaseApplLayer.h \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/NetwToMacControlInfo.h \
	base/utils/PassedMessage.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/application/ieee80211p/BaseWaveApplLayer.h \
	modules/application/traci/TraCIDemoRSU11p.h \
	modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	modules/messages/WaveShortMessage_m.h \
	modules/utility/Consts80211p.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/application/traci/TraCITestApp.o: modules/application/traci/TraCITestApp.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseApplLayer.h \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/PassedMessage.h \
	base/utils/SimpleAddress.h \
	base/utils/asserts.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/application/traci/TraCITestApp.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIColor.h \
	modules/mobility/traci/TraCICommandInterface.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCICoord.h \
	modules/mobility/traci/TraCIMobility.h \
	modules/mobility/traci/TraCIScenarioManager.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/mac/ieee80211p/Mac1609_4.o: modules/mac/ieee80211p/Mac1609_4.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/INotifiable.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NotificationBoard.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/contract/Ieee802Ctrl_m.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/networklayer/arp/ARP.h \
	$(INET_PROJ)/src/networklayer/arp/ARPPacket_m.h \
	$(INET_PROJ)/src/networklayer/arp/IARPCache.h \
	$(INET_PROJ)/src/networklayer/common/IInterfaceTable.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceEntry.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceTableAccess.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceToken.h \
	$(INET_PROJ)/src/networklayer/contract/IPProtocolId_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPvXAddress.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/util/FWMath.h \
	$(INET_PROJ)/src/util/opp_utils.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/ChannelAccess.h \
	base/connectionManager/NicEntry.h \
	base/messages/AirFrame_m.h \
	base/messages/ChannelSenseRequest_m.h \
	base/messages/MacPkt_m.h \
	base/modules/BaseBattery.h \
	base/modules/BaseLayer.h \
	base/modules/BaseMacLayer.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/BaseDecider.h \
	base/phyLayer/BasePhyLayer.h \
	base/phyLayer/ChannelInfo.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Decider.h \
	base/phyLayer/DeciderToPhyInterface.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/MacToPhyControlInfo.h \
	base/phyLayer/MacToPhyInterface.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/PhyUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/NetwToMacControlInfo.h \
	base/utils/PassedMessage.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/JakesFading.h \
	modules/analogueModel/LogNormalShadowing.h \
	modules/analogueModel/SimplePathlossModel.h \
	modules/mac/ieee80211p/Mac1609_4.h \
	modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
	modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	modules/messages/Mac80211Pkt_m.h \
	modules/messages/WaveShortMessage_m.h \
	modules/phy/Decider80211p.h \
	modules/phy/Decider80211pToPhy80211pInterface.h \
	modules/phy/PhyLayer80211p.h \
	modules/phy/SNRThresholdDecider.h \
	modules/utility/Consts80211p.h
$O/modules/messages/AirFrame11p_m.o: modules/messages/AirFrame11p_m.cc \
	base/messages/AirFrame_m.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	modules/messages/AirFrame11p_m.h
$O/modules/messages/Mac80211Pkt_m.o: modules/messages/Mac80211Pkt_m.cc \
	base/messages/MacPkt_m.h \
	base/utils/MiXiMDefs.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h \
	modules/messages/Mac80211Pkt_m.h
$O/modules/messages/WaveShortMessage_m.o: modules/messages/WaveShortMessage_m.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/networklayer/arp/ARPPacket_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPProtocolId_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/messages/WaveShortMessage_m.h
$O/modules/mobility/traci/TraCIBuffer.o: modules/mobility/traci/TraCIBuffer.cc \
	$(INET_PROJ)/src/world/traci/TraCIConstants.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIConstants.h \
	modules/mobility/traci/TraCICoord.h
$O/modules/mobility/traci/TraCIColor.o: modules/mobility/traci/TraCIColor.cc \
	modules/mobility/traci/TraCIColor.h
$O/modules/mobility/traci/TraCICommandInterface.o: modules/mobility/traci/TraCICommandInterface.cc \
	$(INET_PROJ)/src/world/traci/TraCIConstants.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIColor.h \
	modules/mobility/traci/TraCICommandInterface.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCIConstants.h \
	modules/mobility/traci/TraCICoord.h
$O/modules/mobility/traci/TraCIConnection.o: modules/mobility/traci/TraCIConnection.cc \
	$(INET_PROJ)/src/world/traci/TraCIConstants.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCIConstants.h
$O/modules/mobility/traci/TraCIMobility.o: modules/mobility/traci/TraCIMobility.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIColor.h \
	modules/mobility/traci/TraCICommandInterface.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCICoord.h \
	modules/mobility/traci/TraCIMobility.h \
	modules/mobility/traci/TraCIScenarioManager.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/mobility/traci/TraCIScenarioManager.o: modules/mobility/traci/TraCIScenarioManager.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIColor.h \
	modules/mobility/traci/TraCICommandInterface.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCIConstants.h \
	modules/mobility/traci/TraCICoord.h \
	modules/mobility/traci/TraCIMobility.h \
	modules/mobility/traci/TraCIScenarioManager.h \
	modules/mobility/traci/TraCIScenarioManagerInet.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/mobility/traci/TraCIScenarioManagerInet.o: modules/mobility/traci/TraCIScenarioManagerInet.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h
$O/modules/mobility/traci/TraCIScenarioManagerLaunchd.o: modules/mobility/traci/TraCIScenarioManagerLaunchd.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseWorldUtility.h \
	base/utils/FindModule.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIColor.h \
	modules/mobility/traci/TraCICommandInterface.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCIConstants.h \
	modules/mobility/traci/TraCICoord.h \
	modules/mobility/traci/TraCIScenarioManager.h \
	modules/mobility/traci/TraCIScenarioManagerLaunchd.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/obstacle/Obstacle.o: modules/obstacle/Obstacle.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/utils/FindModule.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/obstacle/Obstacle.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/obstacle/ObstacleControl.o: modules/obstacle/ObstacleControl.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/utils/FindModule.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/phy/Decider80211p.o: modules/phy/Decider80211p.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/messages/AirFrame_m.h \
	base/messages/ChannelSenseRequest_m.h \
	base/messages/MacPkt_m.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/BaseDecider.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Decider.h \
	base/phyLayer/DeciderToPhyInterface.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/MacToPhyInterface.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/PhyUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/SimpleAddress.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
	modules/messages/AirFrame11p_m.h \
	modules/messages/Mac80211Pkt_m.h \
	modules/phy/Decider80211p.h \
	modules/phy/Decider80211pToPhy80211pInterface.h \
	modules/phy/DeciderResult80211.h \
	modules/utility/Consts80211p.h
$O/modules/phy/PhyLayer80211p.o: modules/phy/PhyLayer80211p.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/ChannelAccess.h \
	base/connectionManager/NicEntry.h \
	base/messages/AirFrame_m.h \
	base/messages/ChannelSenseRequest_m.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/phyLayer/AnalogueModel.h \
	base/phyLayer/BaseDecider.h \
	base/phyLayer/BasePhyLayer.h \
	base/phyLayer/ChannelInfo.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Decider.h \
	base/phyLayer/DeciderToPhyInterface.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/MacToPhyControlInfo.h \
	base/phyLayer/MacToPhyInterface.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/PhyUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/FindModule.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/analogueModel/BreakpointPathlossModel.h \
	modules/analogueModel/JakesFading.h \
	modules/analogueModel/LogNormalShadowing.h \
	modules/analogueModel/PERModel.h \
	modules/analogueModel/SimpleObstacleShadowing.h \
	modules/analogueModel/SimplePathlossModel.h \
	modules/analogueModel/TwoRayInterferenceModel.h \
	modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
	modules/messages/AirFrame11p_m.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/phy/Decider80211p.h \
	modules/phy/Decider80211pToPhy80211pInterface.h \
	modules/phy/PhyLayer80211p.h \
	modules/phy/SNRThresholdDecider.h \
	modules/utility/Consts80211p.h \
	modules/world/annotations/AnnotationManager.h
$O/modules/phy/SNRThresholdDecider.o: modules/phy/SNRThresholdDecider.cc \
	base/messages/AirFrame_m.h \
	base/messages/ChannelSenseRequest_m.h \
	base/phyLayer/BaseDecider.h \
	base/phyLayer/ChannelState.h \
	base/phyLayer/Decider.h \
	base/phyLayer/DeciderToPhyInterface.h \
	base/phyLayer/Interpolation.h \
	base/phyLayer/Mapping.h \
	base/phyLayer/MappingBase.h \
	base/phyLayer/MappingUtils.h \
	base/phyLayer/Signal_.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	modules/phy/SNRThresholdDecider.h
$O/modules/routing/GPSR.o: modules/routing/GPSR.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/ILifecycle.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/base/INotifiable.h \
	$(INET_PROJ)/src/base/LifecycleOperation.h \
	$(INET_PROJ)/src/base/ModuleAccess.h \
	$(INET_PROJ)/src/base/NodeOperations.h \
	$(INET_PROJ)/src/base/NotificationBoard.h \
	$(INET_PROJ)/src/base/NotifierConsts.h \
	$(INET_PROJ)/src/linklayer/contract/MACAddress.h \
	$(INET_PROJ)/src/mobility/contract/IMobility.h \
	$(INET_PROJ)/src/networklayer/common/IInterfaceTable.h \
	$(INET_PROJ)/src/networklayer/common/INetfilter.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceEntry.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceTableAccess.h \
	$(INET_PROJ)/src/networklayer/common/InterfaceToken.h \
	$(INET_PROJ)/src/networklayer/contract/IPProtocolId_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPSocket.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4ControlInfo.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4ControlInfo_m.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPvXAddress.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Datagram_m.h \
	$(INET_PROJ)/src/networklayer/ipv4/IPv4Route.h \
	$(INET_PROJ)/src/networklayer/ipv4/IRoutingTable.h \
	$(INET_PROJ)/src/networklayer/routing/gpsr/GPSR.h \
	$(INET_PROJ)/src/networklayer/routing/gpsr/GPSRDefs.h \
	$(INET_PROJ)/src/networklayer/routing/gpsr/GPSR_m.h \
	$(INET_PROJ)/src/networklayer/routing/gpsr/PositionTable.h \
	$(INET_PROJ)/src/status/NodeStatus.h \
	$(INET_PROJ)/src/transport/udp/UDPPacket.h \
	$(INET_PROJ)/src/transport/udp/UDPPacket_m.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/modules/BaseBattery.h \
	base/modules/BaseMobility.h \
	base/modules/BaseModule.h \
	base/modules/BaseWorldUtility.h \
	base/modules/BatteryAccess.h \
	base/modules/IMobility.h \
	base/utils/HostState.h \
	base/utils/MiXiMDefs.h \
	base/utils/Move.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/routing/GPSR.h \
	modules/routing/GPSR_m.h
$O/modules/routing/GPSR_m.o: modules/routing/GPSR_m.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/networklayer/contract/IPv4Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPv6Address.h \
	$(INET_PROJ)/src/networklayer/contract/IPvXAddress.h \
	$(INET_PROJ)/src/networklayer/routing/gpsr/GPSRDefs.h \
	$(INET_PROJ)/src/networklayer/routing/gpsr/GPSR_m.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/routing/GPSR_m.h
$O/modules/world/annotations/AnnotationDummy.o: modules/world/annotations/AnnotationDummy.cc \
	modules/world/annotations/AnnotationDummy.h
$O/modules/world/annotations/AnnotationManager.o: modules/world/annotations/AnnotationManager.cc \
	$(INET_PROJ)/src/base/Compat.h \
	$(INET_PROJ)/src/base/Coord.h \
	$(INET_PROJ)/src/base/INETDefs.h \
	$(INET_PROJ)/src/util/FWMath.h \
	base/connectionManager/BaseConnectionManager.h \
	base/connectionManager/NicEntry.h \
	base/modules/BaseWorldUtility.h \
	base/utils/FindModule.h \
	base/utils/MiXiMDefs.h \
	base/utils/miximkerneldefs.h \
	inet/Coord.h \
	inet/FWMath.h \
	modules/mobility/traci/TraCIBuffer.h \
	modules/mobility/traci/TraCIColor.h \
	modules/mobility/traci/TraCICommandInterface.h \
	modules/mobility/traci/TraCIConnection.h \
	modules/mobility/traci/TraCICoord.h \
	modules/mobility/traci/TraCIScenarioManager.h \
	modules/obstacle/Obstacle.h \
	modules/obstacle/ObstacleControl.h \
	modules/world/annotations/AnnotationManager.h

