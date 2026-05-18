1. Add Skypuff application

Add to conf_general.h:

#define APP_CUSTOM_TO_USE                       "skypuff/app_skypuff.c"

Don't forget to set CUSTOM_APPLICATION in the vesc_tool.



2. Too big firmware

Firmware becomes too big with lispBM included.
Add USE_LISPBM=0 when make.

$ make str500 USE_LISPBM=0


