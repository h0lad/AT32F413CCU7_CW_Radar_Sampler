# SPDX-License-Identifier: AGPL-3.0-or-later
include(CMakeParseArguments)

function(at32_add_module module_name)
  set(options)
  set(one_value_args)
  set(multi_value_args SOURCES INCLUDE_DIRECTORIES COMPILE_DEFINITIONS)
  cmake_parse_arguments(MODULE "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT MODULE_SOURCES)
    message(FATAL_ERROR "at32_add_module(${module_name}) requires at least one source file")
  endif()

  add_executable(${module_name})

  target_sources(${module_name} PRIVATE
    ${MODULE_SOURCES}
    $<TARGET_OBJECTS:at32_runtime>
  )

  target_link_libraries(${module_name} PRIVATE at32_vendor)

  target_include_directories(${module_name} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    ${MODULE_INCLUDE_DIRECTORIES}
  )

  target_compile_definitions(${module_name} PRIVATE ${MODULE_COMPILE_DEFINITIONS})

  set(module_output_dir "${CMAKE_BINARY_DIR}/artifacts/${module_name}")
  file(MAKE_DIRECTORY "${module_output_dir}")

  set_target_properties(${module_name} PROPERTIES
    OUTPUT_NAME "${module_name}"
    SUFFIX ".elf"
    RUNTIME_OUTPUT_DIRECTORY "${module_output_dir}"
  )

  target_link_options(${module_name} PRIVATE
    "-T${AT32_LINKER_SCRIPT}"
    "LINKER:--gc-sections"
    "LINKER:--print-memory-usage"
    "LINKER:-Map=${module_output_dir}/${module_name}.map"
  )

  add_custom_command(TARGET ${module_name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${module_name}> "${module_output_dir}/${module_name}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${module_name}> "${module_output_dir}/${module_name}.bin"
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${module_name}>
    VERBATIM
  )
endfunction()
