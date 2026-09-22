/*
 * Minimal config.h for compiling vendored libmodbus directly (no autotools).
 *
 * Deliberately does NOT define HAVE_STRUCT_TERMIOS2, so modbus-rtu.c uses the
 * standard termios baud table (1200..115200) instead of the <asm/termbits.h>
 * custom-baud path. Safer on older BeagleBone kernels. Re-enable by defining
 * HAVE_STRUCT_TERMIOS2 if custom baud rates are ever needed.
 *
 * HAVE_STRLCPY intentionally undefined -> libmodbus uses its own strlcpy.
 * HAVE_DECL_TIOCSRS485 / HAVE_DECL_TIOCM_RTS undefined -> the RS-485 kernel
 * ioctl path is skipped (USB-RS485 adapters handle half-duplex in hardware).
 */
#ifndef LIBMODBUS_VENDOR_CONFIG_H
#define LIBMODBUS_VENDOR_CONFIG_H

#endif /* LIBMODBUS_VENDOR_CONFIG_H */
