#!/usr/bin/env bash
# Source after defining ROOT, X5_SYSROOT and PSDK_ROOT. No host library fallback.
target_fail() { echo "target_ros_env: $*" >&2; return 2; }
target_arm() {
  [[ -f $1 ]] && LC_ALL=C readelf -h "$1" 2>/dev/null | grep -q 'Machine:.*AArch64'
}
target_setting() {
  local value=$2
  if [[ $value == *$'\n'* || $value == *$'\r'* || $value == *'[=['* || $value == *']=]'* ]]; then
    target_fail 'Unsupported CMake path characters'; return 2
  fi
  printf 'set(%s [=[%s]=] CACHE STRING "Checked target path" FORCE)\n' "$1" "$value" >> "$target_cache"
}
command -v readelf >/dev/null || { target_fail 'readelf is required'; return 2; }
target_arm "$PSDK_ROOT/lib/libperception_sdk.so" || { target_fail 'PSDK_ROOT must contain an AArch64 SDK'; return 2; }
[[ -n ${TARGET_ROS_PREFIX:-} ]] || { target_fail 'Set TARGET_ROS_PREFIX to complete real ARM ROS prefix(es), separated by semicolons'; return 2; }
IFS=';' read -r -a target_roots <<< "$TARGET_ROS_PREFIX"
target_roots+=("$X5_SYSROOT/usr_x5" "$X5_SYSROOT/usr" "$X5_SYSROOT/opt/ros/humble" "$X5_SYSROOT/opt/tros/humble")
target_prefixes=()
for target_root in "${target_roots[@]}"; do
  [[ ! -d $target_root ]] || target_prefixes+=("$(cd -- "$target_root" && pwd)")
done
mkdir -p "$ROOT/validation"
target_cache="$ROOT/validation/target_ros.cmake"
: > "$target_cache"
for target_name in rclcpp sensor_msgs std_msgs ament_cmake; do
  target_found=
  case "$target_name" in
    rclcpp) target_library=librclcpp.so;;
    sensor_msgs) target_library=libsensor_msgs__rosidl_typesupport_cpp.so;;
    std_msgs) target_library=libstd_msgs__rosidl_typesupport_cpp.so;;
    ament_cmake) target_library=;;
  esac
  for target_root in "${target_prefixes[@]}"; do
    target_config="$target_root/share/$target_name/cmake/${target_name}Config.cmake"
    [[ -f $target_config ]] || continue
    if [[ -n $target_library ]] && ! target_arm "$target_root/lib/$target_library"; then continue; fi
    target_setting "${target_name}_DIR" "$(dirname -- "$target_config")"
    target_found=$target_root; break
  done
  [[ -n $target_found ]] || { target_fail "Missing real target $target_name package"; return 2; }
done
for target_kind in rosidl_typesupport_c rosidl_typesupport_cpp; do
  target_found=
  for target_root in "${target_prefixes[@]}"; do
    target_arm "$target_root/lib/lib${target_kind}.so" || continue
    for target_marker in "$target_root/share/ament_index/resource_index/$target_kind/"*; do
      [[ -f $target_marker ]] || continue
      if target_arm "$target_root/lib/lib$(basename -- "$target_marker").so"; then target_found=1; fi
    done
  done
  [[ -n $target_found ]] || { target_fail "Missing ARM $target_kind implementation and ament resource index"; return 2; }
done
target_system=${TARGET_SYSTEM_PREFIX:-$X5_SYSROOT/usr_x5}
target_archlib="$target_system/lib/aarch64-linux-gnu"
for target_library in libcrypto.so.3 libssl.so.3; do
  target_arm "$target_archlib/$target_library" || { target_fail "Missing ARM $target_archlib/$target_library"; return 2; }
done
[[ -f $target_system/include/openssl/ssl.h ]] && grep -Eq '^[[:space:]]*#[[:space:]]*define[[:space:]]+OPENSSL_VERSION_MAJOR[[:space:]]+3([[:space:]]|$)' "$target_system/include/openssl/opensslv.h" ||
  { target_fail 'Target OpenSSL development headers must match OpenSSL 3'; return 2; }
for target_name in spdlog fmt; do
  target_dir="$target_archlib/cmake/$target_name"
  [[ -d $target_dir ]] || { target_fail "Missing target CMake package: $target_dir"; return 2; }
  target_found=
  for target_library in "$target_archlib/lib$target_name.so"*; do
    if target_arm "$target_library"; then target_found=1; break; fi
  done
  [[ -n $target_found ]] || { target_fail "Missing ARM $target_name shared library"; return 2; }
  target_setting "${target_name}_DIR" "$target_dir"
done
target_cmake_prefix="$PSDK_ROOT;$target_system"
target_find_roots="$X5_SYSROOT;$PSDK_ROOT"
target_links="$PSDK_ROOT/lib;$target_archlib;$target_system/lib;$X5_SYSROOT/usr_x5/lib/hbbpu;$X5_SYSROOT/usr_x5/hobot/lib;$X5_SYSROOT/usr/lib/aarch64-linux-gnu;$X5_SYSROOT/lib/aarch64-linux-gnu"
AMENT_PREFIX_PATH=
for target_root in "${target_prefixes[@]}"; do
  target_cmake_prefix+=";$target_root"; target_find_roots+=";$target_root"; target_links+=";$target_root/lib"
  AMENT_PREFIX_PATH+="${AMENT_PREFIX_PATH:+:}$target_root"
done
export AMENT_PREFIX_PATH
target_setting CMAKE_PREFIX_PATH "$target_cmake_prefix"
target_setting R36_DEMO_EXTRA_ROOTS "$target_find_roots"
target_setting PSDK_LINK_DEPENDENCY_DIRS "$target_links"
target_setting CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu
target_setting CMAKE_LIBRARY_PATH "$target_archlib;$target_system/lib"
target_setting CMAKE_INCLUDE_PATH "$target_system/include/aarch64-linux-gnu;$target_system/include"
target_setting R36_DEMO_SYSTEM_INCLUDE_DIRS "$target_system/include/aarch64-linux-gnu;$target_system/include"
target_setting OPENSSL_ROOT_DIR "$target_system"
target_setting OPENSSL_INCLUDE_DIR "$target_system/include"
target_setting OPENSSL_CRYPTO_LIBRARY "$target_archlib/libcrypto.so.3"
target_setting OPENSSL_SSL_LIBRARY "$target_archlib/libssl.so.3"
printf 'set(ENV{AMENT_PREFIX_PATH} [=[%s]=])\n' "$AMENT_PREFIX_PATH" >> "$target_cache"
printf 'ARM SDK/ROS/ament indexes and OpenSSL/logging libraries checked. Cache: %s\n' "$target_cache"
