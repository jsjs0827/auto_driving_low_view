#!/usr/bin/env bash
# Source inside the build container. Read-only dependency discovery; no build.
# Do not change shell options here: failure/errexit policy belongs to the caller.
_matrix_env_fail() {
  printf 'prepare_inference_matrix_env.sh: %s\n' "$*" >&2
  return 2
}
_matrix_env_arm() {
  local header
  [[ -f $1 ]] || return 1
  header=$(LC_ALL=C readelf -h "$1" 2>/dev/null) || return 1
  [[ $header =~ Machine:[[:space:]]*AArch64([[:space:]]|$) ]]
}
_matrix_env_main() {
  local name value compiler machine opencv pcl horizon_library= cfg prefix physical previous duplicate found
  local selected_ros= selected_eigen= explicit_arm=0
  local -a ros_prefixes=() candidates=() scan_roots=() search_prefixes=() eigen_candidates=()
  for name in X5_SYSROOT TARGET_SYSTEM_PREFIX STEREO_HORIZON_ROOT; do
    value=${!name:-}
    [[ -n $value && $value == /* && -d $value ]] || {
      _matrix_env_fail "$name must name an existing absolute directory: ${value:-<unset>}"; return 2;
    }
  done
  [[ -d $X5_SYSROOT/usr/include ]] || {
    _matrix_env_fail "Missing sysroot usr/include (check any historical usr symlink): $X5_SYSROOT/usr/include"; return 2;
  }
  for name in readelf find; do
    command -v "$name" >/dev/null 2>&1 || {
      _matrix_env_fail "Required container command is missing: $name"; return 2;
    }
  done
  compiler=${X5_CXX_COMPILER:-/usr/bin/aarch64-linux-gnu-g++}
  command -v "$compiler" >/dev/null 2>&1 || {
    _matrix_env_fail "Missing AArch64 compiler command: $compiler"; return 2;
  }
  machine=$("$compiler" -dumpmachine 2>/dev/null) || {
    _matrix_env_fail "Compiler failed -dumpmachine: $compiler"; return 2;
  }
  [[ $machine == aarch64*-linux-* ]] || {
    _matrix_env_fail "Compiler must target AArch64 Linux; got: $machine"; return 2;
  }
  [[ -f $STEREO_HORIZON_ROOT/include/dnn/hb_dnn.h ]] || {
    _matrix_env_fail "Missing Horizon header: $STEREO_HORIZON_ROOT/include/dnn/hb_dnn.h"; return 2;
  }
  for value in "$STEREO_HORIZON_ROOT/lib/libdnn.so" "$STEREO_HORIZON_ROOT/lib/hbbpu/libdnn.so"; do
    if [[ -f $value ]]; then horizon_library=$value; break; fi
  done
  [[ -n $horizon_library ]] || {
    _matrix_env_fail "Missing Horizon libdnn.so under $STEREO_HORIZON_ROOT/lib or lib/hbbpu"; return 2;
  }
  opencv=${TARGET_OPENCV_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/opencv4}
  pcl=${TARGET_PCL_DIR:-$TARGET_SYSTEM_PREFIX/lib/aarch64-linux-gnu/cmake/pcl}
  [[ $opencv == /* && -f $opencv/OpenCVConfig.cmake ]] || {
    _matrix_env_fail "Missing target OpenCV config: $opencv/OpenCVConfig.cmake"; return 2;
  }
  [[ $pcl == /* && -f $pcl/PCLConfig.cmake ]] || {
    _matrix_env_fail "Missing target PCL config: $pcl/PCLConfig.cmake"; return 2;
  }

  if [[ -n ${TARGET_ROS_PREFIX:-} ]]; then
    value=$TARGET_ROS_PREFIX
    [[ $value != *$'\n'* && $value != *$'\r'* && $value != \;* && $value != *\; && $value != *\;\;* ]] || {
      _matrix_env_fail 'TARGET_ROS_PREFIX must be one semicolon-separated line without empty entries.'; return 2;
    }
    IFS=';' read -r -a candidates <<< "$value"
    for prefix in "${candidates[@]}"; do
      [[ $prefix == /* && -d $prefix ]] || {
        _matrix_env_fail "Invalid explicit TARGET_ROS_PREFIX directory: $prefix"; return 2;
      }
      physical=$(cd -P -- "$prefix" && pwd) || return 2
      duplicate=0
      for previous in "${ros_prefixes[@]}"; do [[ $physical != "$previous" ]] || duplicate=1; done
      if ((duplicate == 0)); then ros_prefixes+=("$physical"); fi
      if [[ -f $prefix/share/rclcpp/cmake/rclcppConfig.cmake ]]; then
        _matrix_env_arm "$prefix/lib/librclcpp.so" || {
          _matrix_env_fail "Explicit ROS prefix has no valid AArch64 librclcpp.so: $prefix"; return 2;
        }
        explicit_arm=1
      fi
    done
    ((explicit_arm)) || {
      _matrix_env_fail 'Explicit TARGET_ROS_PREFIX must include an AArch64 rclcpp prefix (config and lib/librclcpp.so).'; return 2;
    }
  else
    candidates=()
    # Restrict discovery to the supplied sysroot and these known ROS locations.
    for prefix in "$X5_SYSROOT" /opt/ros/humble /opt/tros/humble /app/humble; do
      [[ ! -d $prefix ]] || scan_roots+=("$prefix")
    done
    for prefix in "${scan_roots[@]}"; do
      while IFS= read -r -d '' cfg; do
        value=${cfg%/share/rclcpp/cmake/rclcppConfig.cmake}
        _matrix_env_arm "$value/lib/librclcpp.so" || continue
        physical=$(cd -P -- "$value" && pwd) || continue
        duplicate=0
        for previous in "${candidates[@]}"; do [[ $physical != "$previous" ]] || duplicate=1; done
        if ((duplicate == 0)); then candidates+=("$physical"); fi
      done < <(find -L "$prefix" -type f -path '*/share/rclcpp/cmake/rclcppConfig.cmake' -print0 2>/dev/null)
    done
    if ((${#candidates[@]} == 0)); then
      _matrix_env_fail 'No AArch64 ROS2 rclcpp prefix found in X5_SYSROOT or /opt/ros/humble, /opt/tros/humble, /app/humble. Set TARGET_ROS_PREFIX to complete matching ARM ROS2 development prefix(es); x86 ROS is not usable.'
      return 2
    fi
    if ((${#candidates[@]} != 1)); then
      _matrix_env_fail 'Multiple physical AArch64 rclcpp prefixes found; set TARGET_ROS_PREFIX explicitly:'
      printf '  %s\n' "${candidates[@]}" >&2
      return 2
    fi
    ros_prefixes=("${candidates[0]}")
  fi
  search_prefixes=("${ros_prefixes[@]}" "$TARGET_SYSTEM_PREFIX" "$X5_SYSROOT/usr_x5" "$X5_SYSROOT/usr"
                   "$X5_SYSROOT/opt/ros/humble" "$X5_SYSROOT/opt/tros/humble" "$X5_SYSROOT/app/humble")
  for name in nav_msgs sensor_msgs std_msgs ament_cmake; do
    found=0
    for prefix in "${search_prefixes[@]}"; do
      if [[ -f $prefix/share/$name/cmake/${name}Config.cmake ]]; then found=1; break; fi
    done
    ((found)) || {
      _matrix_env_fail "Missing target $name CMake package in selected ROS prefixes/sysroot roots. Add its real ARM installation to TARGET_ROS_PREFIX."; return 2;
    }
  done

  if [[ -n ${TARGET_EIGEN_DIR:-} ]]; then
    selected_eigen=$TARGET_EIGEN_DIR
    [[ $selected_eigen == /* && -f $selected_eigen/Eigen3Config.cmake ]] || {
      _matrix_env_fail "Invalid TARGET_EIGEN_DIR: $selected_eigen/Eigen3Config.cmake"; return 2;
    }
  elif [[ -f $TARGET_SYSTEM_PREFIX/share/eigen3/cmake/Eigen3Config.cmake ]]; then
    selected_eigen=$TARGET_SYSTEM_PREFIX/share/eigen3/cmake
  else
    while IFS= read -r -d '' cfg; do
      physical=$(cd -P -- "${cfg%/*}" && pwd) || continue
      duplicate=0
      for previous in "${eigen_candidates[@]}"; do [[ $physical != "$previous" ]] || duplicate=1; done
      if ((duplicate == 0)); then eigen_candidates+=("$physical"); fi
    done < <(find -L "$X5_SYSROOT" -type f -name Eigen3Config.cmake -print0 2>/dev/null)
    if ((${#eigen_candidates[@]} == 0)); then
      _matrix_env_fail 'Eigen3Config.cmake was not found in the sysroot. Set TARGET_EIGEN_DIR to its existing CMake directory.'
      return 2
    fi
    if ((${#eigen_candidates[@]} != 1)); then
      _matrix_env_fail 'Multiple physical Eigen CMake directories found; set TARGET_EIGEN_DIR explicitly:'
      printf '  %s\n' "${eigen_candidates[@]}" >&2
      return 2
    fi
    selected_eigen=${eigen_candidates[0]}
  fi
  selected_eigen=$(cd -P -- "$selected_eigen" && pwd) || return 2
  selected_ros=$(IFS=';'; printf '%s' "${ros_prefixes[*]}")
  export X5_SYSROOT TARGET_SYSTEM_PREFIX STEREO_HORIZON_ROOT
  export X5_CXX_COMPILER=$compiler TARGET_OPENCV_DIR=$opencv TARGET_PCL_DIR=$pcl
  export TARGET_ROS_PREFIX=$selected_ros TARGET_EIGEN_DIR=$selected_eigen
  printf 'Dependency precheck passed (no build or sysroot changes).\nTARGET_ROS_PREFIX=%s\nTARGET_EIGEN_DIR=%s\n' \
    "$TARGET_ROS_PREFIX" "$TARGET_EIGEN_DIR"
  printf 'The build still checks complete ARM ROS/typesupport/OpenSSL/logging dependencies.\n'
}

if _matrix_env_main; then
  unset -f _matrix_env_main _matrix_env_fail _matrix_env_arm
else
  unset -f _matrix_env_main _matrix_env_fail _matrix_env_arm
  if [[ ${BASH_SOURCE[0]} != "$0" ]]; then return 2; fi
  exit 2
fi
