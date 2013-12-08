// Copyright 2013 Mobilinkd LLC <info@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD_UTIL_H_
#define MOBILINKD_UTIL_H_

#include <avr/pgmspace.h>
#include <io/kfile.h>

/**
 * Write a PROGMEM string to kfile \a fd.
 * \return 0 if OK, EOF in case of error.
 */
INLINE int kfile_print_P(struct KFile *fd, const char *s)
{
    while (pgm_read_byte(s))
    {
        if (kfile_putc(pgm_read_byte(s++), fd) == EOF)
            return EOF;
    }
    return 0;
}

#endif // MOBILINKD_UTIL_H_
