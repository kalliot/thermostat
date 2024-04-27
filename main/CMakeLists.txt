idf_component_register(SRCS "app_main.c" "ds18b20.c" "display.c" "max7219.c" "cJSON.c" "temperatures.c" "pidcontroller.c"
                    "flashmem.c" "heater.c" "factoryreset.c" "ntcreader.c" "apwebserver/server.c" "ota/ota.c" INCLUDE_DIRS "."
	            EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
