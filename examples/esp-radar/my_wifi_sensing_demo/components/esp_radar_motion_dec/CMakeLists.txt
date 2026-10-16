# Project-local component named esp_radar_motion_dec.
# ESP-IDF gives components/ higher priority than managed_components, so this
# replaces the closed-source prebuilt espressif/esp_radar_motion_dec library.
# Sources stay in csi_pca_fingerprint_algo; this file only wires them in.

get_filename_component(MOTION_DEC_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../csi_pca_fingerprint_algo" ABSOLUTE)

if(NOT EXISTS "${MOTION_DEC_ROOT}/src/esp_radar_motion_dec.c")
    message(FATAL_ERROR "Open-source esp_radar_motion_dec sources not found: ${MOTION_DEC_ROOT}/src/esp_radar_motion_dec.c")
endif()

idf_component_register(SRCS "${MOTION_DEC_ROOT}/src/esp_radar_motion_dec.c"
                       INCLUDE_DIRS "${MOTION_DEC_ROOT}/include"
                       REQUIRES esp_common)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
