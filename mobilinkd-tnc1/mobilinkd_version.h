// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD_VERSION_H_
#define MOBILINKD_VERSION_H_

#include <cpu/pgm.h>

#include "buildrev.h"

#define VERS_MAJOR 1
#define VERS_MINOR 3
#define VERS_PATCH 0

#define VERS_SEP .

#define VERSION_CAT_NX(MAJOR, MINOR, PATCH, BUILD) MAJOR ## . ## MINOR \
    ## . ## PATCH ## . ## BUILD

#define VERSION_CAT(MAJOR, MINOR, PATCH, BUILD) VERSION_CAT_NX(MAJOR, MINOR, PATCH, BUILD)

#define STRINGIFY(X) #X
#define TO_STRING(x) STRINGIFY(x)

#define MOBILINKD_VERSION_STR TO_STRING(VERSION_CAT(VERS_MAJOR, VERS_MINOR, VERS_PATCH, VERS_BUILD))

extern const char firmware_version[] PROGMEM;
extern const char hardware_version[] PROGMEM;

#endif // MOBILINKD_VERSION_H_
