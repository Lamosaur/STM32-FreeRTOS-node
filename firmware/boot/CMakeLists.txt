# ---------------------------------------------------------------------------
# Boot core — the target-independent pieces of Phase 2.
#
# Plain C on purpose: the bootloader is the smallest program in this project, and
# nothing in it needs a C++ runtime. The same library compiles into the host test
# suite (where gtest drives it from C++) and, later, into the bootloader image —
# that shared compilation is FR-0.6's habit applied to Phase 2.
# ---------------------------------------------------------------------------
add_library(boot_core STATIC
  src/crc.c
  src/metadata.c
  src/decision.c)

target_include_directories(boot_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(boot_core PRIVATE node_options_strict)
target_compile_features(boot_core PUBLIC c_std_11)