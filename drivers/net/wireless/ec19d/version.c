#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>

/* NR_DRIVER_VERSION is defined by the makefile */

#undef VERSION
#ifdef NR_DRIVER_VERSION
#define VERSION NR_DRIVER_VERSION " " __DATE__ " " __TIME__
#else
#define VERSION __DATE__ " " __TIME__
#endif

MODULE_VERSION(VERSION);
