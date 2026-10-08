#
# "main" pseudo-component makefile.
#
# Compile the application and its Modbus/UI modules.

COMPONENT_SRCDIRS := . modbus_src
COMPONENT_ADD_INCLUDEDIRS := . modbus_src

CFLAGS+= -DLV_CONF_INCLUDE_SIMPLE
