#include "x86_64_extremenetworks_8730_32d_int.h"

#if X86_64_EXTREMENETWORKS_8730_32D_CONFIG_INCLUDE_DEBUG == 1

#include <unistd.h>

static char help__[] =
    "Usage: debug [options]\n"
    "    -c CPLD Versions\n"
    "    -h Help\n"
    ;

int
x86_64_extremenetworks_8730_32d_debug_main(int argc, char* argv[])
{
    int c = 0;
    int help = 0;
    int rv = 0;

    while( (c = getopt(argc, argv, "ch")) != -1) {
        switch(c)
            {
            case 'c': c = 1; break;
            case 'h': help = 1; rv = 0; break;
            default: help = 1; rv = 1; break;
            }

    }

    if(help || argc == 1) {
        printf("%s", help__);
        return rv;
    }

    if(c) {
        printf("Not implemented.\n");
    }


    return 0;
}

#endif


