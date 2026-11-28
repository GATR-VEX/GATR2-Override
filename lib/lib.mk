# lib.mk
# Libraries from lib/ for a PROS project, built into $(BINDIR)/lib/.
#   spectagatr  live viewer
#   locagatr    robot pose. Needs GATR2_BRAIN_LIBS := locagatr
#   navigatr    routes. Needs locagatr
# Include it on both sides of common.mk:
#
#   GATR2_LIB_ROOT ?= ../lib
#   GATR2_LIBS := spectagatr locagatr navigatr
#   include $(GATR2_LIB_ROOT)/lib.mk
#   -include ./common.mk
#   include $(GATR2_LIB_ROOT)/lib.mk

ifndef GATR2_LIB_SETUP
GATR2_LIB_SETUP := 1

# lib/ as a path relative to the PROS project. Drive letter paths break the
# include flags in the toolchain shell.
GATR2_LIB_ROOT ?= ../lib

# Libraries to build.
GATR2_LIBS ?= spectagatr

# Sources per library.
GATR2_LIB_SRC_spectagatr := \
	$(GATR2_LIB_ROOT)/spectaGATR/brain/src/publisher.cpp \
	$(GATR2_LIB_ROOT)/spectaGATR/brain/pros/viewer.cpp \
	$(GATR2_LIB_ROOT)/spectaGATR/protocol/src/fields.cpp \
	$(GATR2_LIB_ROOT)/spectaGATR/protocol/src/path_line.cpp \
	$(GATR2_LIB_ROOT)/spectaGATR/protocol/src/pose_line.cpp \
	$(GATR2_LIB_ROOT)/spectaGATR/protocol/src/robot_line.cpp \
	$(GATR2_LIB_ROOT)/spectaGATR/protocol/src/units.cpp

GATR2_LIB_INC_spectagatr := \
	$(GATR2_LIB_ROOT)/spectaGATR/brain/include \
	$(GATR2_LIB_ROOT)/spectaGATR/protocol/include

GATR2_LIB_SRC_locagatr := \
	$(GATR2_LIB_ROOT)/locaGATR/src/types.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/pipeline.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/report.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/sources/control_board_source.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/processors/tracking_wheel_processor.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/processors/wheel_pair_processor.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/processors/drive_motor_processor.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/estimators/odometry_estimator.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/log/log_row.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/src/log/log_name.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/pros/localizer.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/pros/log/run_log.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/pros/sources/vex_imu_source.cpp \
	$(GATR2_LIB_ROOT)/locaGATR/pros/sources/motor_encoder_source.cpp

GATR2_LIB_INC_locagatr := \
	$(GATR2_LIB_ROOT)/locaGATR/include \
	$(GATR2_LIB_ROOT)/gatr2/include

GATR2_LIB_SRC_navigatr := \
	$(GATR2_LIB_ROOT)/naviGATR/src/route.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/src/path.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/src/profiles/profile.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/src/pipeline.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/src/profiles/trapezoid_profile.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/src/followers/holonomic_follower.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/src/drives/mecanum_drive.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/src/driver/heading_hold.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/pros/navigator.cpp \
	$(GATR2_LIB_ROOT)/naviGATR/pros/drives/mecanum_drive.cpp

GATR2_LIB_INC_navigatr := \
	$(GATR2_LIB_ROOT)/naviGATR/include \
	$(GATR2_LIB_ROOT)/gatr2/include

GATR2_LIB_SRC := $(foreach lib,$(GATR2_LIBS),$(GATR2_LIB_SRC_$(lib)))
GATR2_LIB_OBJ := $(patsubst $(GATR2_LIB_ROOT)/%,$(BINDIR)/lib/%.o,$(GATR2_LIB_SRC))

EXTRA_INCDIR += $(foreach lib,$(GATR2_LIBS),$(GATR2_LIB_INC_$(lib)))
ELF_DEPS += $(GATR2_LIB_OBJ)

else

GATR2_LIB_WARNFLAGS ?= -Wall -Wextra

# Own compile rule: the common.mk one assumes sources under src/.
define gatr2_lib_src_rule
$(patsubst $(GATR2_LIB_ROOT)/%,$(BINDIR)/lib/%.o,$1): $1
	$(VV)mkdir -p $$(dir $$@)
	$$(call test_output_2,Compiled $$< ,$(CXX) -c $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) $(GATR2_LIB_WARNFLAGS) -MMD -MP -MF $$(@:.o=.d) -o $$@ $$<,$(OK_STRING))
endef
$(foreach src,$(GATR2_LIB_SRC),$(eval $(call gatr2_lib_src_rule,$(src))))

-include $(GATR2_LIB_OBJ:.o=.d)

endif
