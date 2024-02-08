idf_component_register(SRCS "app_main.c" "ds18b20.c" "display.c" "max7219.c" "cJSON.c" "temperatures.c"
                    "flashmem.c" "factoryreset.c" "ntcreader.c" "apwebserver/server.c" INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
