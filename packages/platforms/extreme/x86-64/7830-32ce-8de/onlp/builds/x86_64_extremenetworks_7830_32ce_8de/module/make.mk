###############################################################################
#
# 
#
###############################################################################
THIS_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
x86_64_extremenetworks_7830_32ce_8de_INCLUDES := -I $(THIS_DIR)inc
x86_64_extremenetworks_7830_32ce_8de_INTERNAL_INCLUDES := -I $(THIS_DIR)src
x86_64_extremenetworks_7830_32ce_8de_DEPENDMODULE_ENTRIES := init:x86_64_extremenetworks_7830_32ce_8de ucli:x86_64_extremenetworks_7830_32ce_8de

