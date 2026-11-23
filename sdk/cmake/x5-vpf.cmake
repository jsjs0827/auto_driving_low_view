# The X5 media VPF and an unrelated VPF library can coexist in one sysroot.
# CMAKE_PREFIX_PATH is searched before HINTS, so generic `NAMES vpf` is unsafe.
function(perception_find_x5_vpf)
  if(NOT IS_DIRECTORY "${HORIZON_ROOT}")
    message(FATAL_ERROR "HORIZON_ROOT must name the matching X5 target prefix")
  endif()

  # Use a separate, freshly searched variable: an old VPF_LIBRARY cache entry
  # must not bypass the restricted search. Prefer the real versioned payload
  # over aliases, which may be flattened text files in a Windows sysroot copy.
  unset(_perception_x5_vpf_candidate CACHE)
  unset(_perception_x5_vpf_candidate)
  find_library(_perception_x5_vpf_candidate
    NAMES libvpf.so.1.0.0 libvpf.so.1
    PATHS "${HORIZON_ROOT}/hobot/lib" "${HORIZON_ROOT}/lib"
    NAMES_PER_DIR NO_DEFAULT_PATH REQUIRED)

  if(CMAKE_READELF)
    set(_vpf_readelf "${CMAKE_READELF}")
  else()
    find_program(_vpf_readelf NAMES aarch64-linux-gnu-readelf readelf REQUIRED)
  endif()
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env "LC_ALL=C" "${_vpf_readelf}"
      --file-header --dynamic --dyn-syms --wide "${_perception_x5_vpf_candidate}"
    RESULT_VARIABLE _vpf_readelf_result
    OUTPUT_VARIABLE _vpf_elf
    ERROR_VARIABLE _vpf_readelf_error)
  if(NOT _vpf_readelf_result STREQUAL "0")
    message(FATAL_ERROR
      "Cannot inspect X5 VPF library ${_perception_x5_vpf_candidate}: ${_vpf_readelf_error}")
  endif()
  if(NOT _vpf_elf MATCHES "Class:[ \t]+ELF64" OR
     NOT _vpf_elf MATCHES "Data:[^\n]+little endian" OR
     NOT _vpf_elf MATCHES "Type:[ \t]+DYN" OR
     NOT _vpf_elf MATCHES "Machine:[ \t]+AArch64" OR
     NOT _vpf_elf MATCHES "\\(SONAME\\)[^\n]+\\[libvpf\\.so\\.1\\]")
    message(FATAL_ERROR
      "X5 VPF must be a little-endian AArch64 ELF64 shared library with SONAME libvpf.so.1: ${_perception_x5_vpf_candidate}")
  endif()

  set(_vpf_required_symbols
    hbn_vflow_stop hbn_vflow_destroy hbn_vflow_create hbn_vflow_add_vnode hbn_vflow_start
    hbn_vnode_close hbn_vnode_sendframe hbn_vnode_getframe hbn_vnode_releaseframe
    hbn_vnode_open hbn_vnode_set_attr hbn_vnode_set_ichn_attr
    hbn_vnode_set_ochn_attr hbn_vnode_set_ochn_buf_attr)
  foreach(_vpf_symbol IN LISTS _vpf_required_symbols)
    # --dyn-syms excludes .symtab. A numeric section index excludes UND imports;
    # require an externally visible function, not just a matching string.
    if(NOT _vpf_elf MATCHES
       "[ \t]FUNC[ \t]+(GLOBAL|WEAK)[ \t]+(DEFAULT|PROTECTED)[ \t]+[0-9]+[ \t]+${_vpf_symbol}([@ \t\r\n]|$)")
      message(FATAL_ERROR
        "X5 VPF is missing defined dynamic function ${_vpf_symbol}: ${_perception_x5_vpf_candidate}")
    endif()
  endforeach()
  set(VPF_LIBRARY "${_perception_x5_vpf_candidate}" CACHE FILEPATH
    "Validated X5 media VPF library (SONAME libvpf.so.1)" FORCE)
  set(VPF_LIBRARY "${_perception_x5_vpf_candidate}" PARENT_SCOPE)
  unset(_perception_x5_vpf_candidate CACHE)
  message(STATUS "X5 media VPF: ${VPF_LIBRARY} (libvpf.so.1; 14 required hbn exports)")
endfunction()
