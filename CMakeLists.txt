# incunest_afe4490 as an ESP-IDF component.
#
# Point EXTRA_COMPONENT_DIRS at this directory (or add it as a git submodule under components/)
# and REQUIRE incunest_afe4490 from your main component. No vendored copy, no patches: the same
# sources PlatformIO/Arduino builds compile here, with incunest_afe4490_hal_idf.cpp providing the
# platform calls over driver/gpio and driver/spi_master.
#
# The application still owns the SPI bus (spi_bus_initialize) — see incunest_afe4490_hal.h.
# Override -DINCUNEST_AFE4490_HAL_SPI_HOST=SPI3_HOST if the AFE is not on SPI2.
#
# PlatformIO ignores this file; it compiles every .cpp in the directory and the per-file guards
# leave the non-matching HAL implementations empty.

idf_component_register(
    SRCS "incunest_afe4490.cpp"
         "incunest_afe4490_hal_idf.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_spi esp_timer
)

# Same warning policy as library.json's build.flags: the library compiles clean, and stays so.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
