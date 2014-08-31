#
# User makefile.
# Edit this file to change compiler options and related stuff.
#

# Programmer interface configuration, see http://dev.bertos.org/wiki/ProgrammerInterface for help
mobilinkd-tnc1_PROGRAMMER_TYPE = none
mobilinkd-tnc1_PROGRAMMER_PORT = none

# Files included by the user.
mobilinkd-tnc1_USER_CSRC = \
	$(mobilinkd-tnc1_SRC_PATH)/main.c \
	$(mobilinkd-tnc1_SRC_PATH)/hc-05.c \
	$(mobilinkd-tnc1_SRC_PATH)/afsk_dcd.c \
	$(mobilinkd-tnc1_SRC_PATH)/battery.c \
	$(mobilinkd-tnc1_SRC_PATH)/power.c \
	$(mobilinkd-tnc1_SRC_PATH)/mobilinkd_error.c \
	$(mobilinkd-tnc1_SRC_PATH)/mobilinkd_eeprom.c \
	$(mobilinkd-tnc1_HW_PATH)/hw/hw_afsk.c \
	#

# Files included by the user.
mobilinkd-tnc1_USER_PCSRC = \
	#

# Files included by the user.
mobilinkd-tnc1_USER_CPPASRC = \
	#

# Files included by the user.
mobilinkd-tnc1_USER_CXXSRC = \
	#

# Files included by the user.
mobilinkd-tnc1_USER_ASRC = \
	#

# Flags included by the user.
mobilinkd-tnc1_USER_LDFLAGS = \
	#

# Flags included by the user.
mobilinkd-tnc1_USER_CPPAFLAGS = \
	#

# Flags included by the user.
mobilinkd-tnc1_USER_CPPFLAGS = \
	-fno-strict-aliasing \
	-fwrapv \
	#
