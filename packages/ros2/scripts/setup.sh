#!/bin/bash

_build_sh_completion() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    
    if [[ -z "${ACT_ROS_WS}" ]]; then
        return 0
    fi
    
    local src_dir="${ACT_ROS_WS}/src"
    
    if [[ -d "${src_dir}" ]]; then
        local packages=$(find "${src_dir}" -maxdepth 2 -name "package.xml" \
            -exec dirname {} \; 2>/dev/null | xargs -n1 basename | sort -u)
        COMPREPLY=($(compgen -W "${packages}" -- "${cur}"))
    fi
}

complete -F _build_sh_completion build.sh
complete -F _build_sh_completion ./scripts/build.sh
complete -F _build_sh_completion ${ACT_ROS_WS}/scripts/build.sh
for ((i=0; i<=2; i+=2)); do
  v4l2-ctl -d "/dev/video${i}" --set-ctrl=auto_exposure=1
  v4l2-ctl -d "/dev/video${i}" --set-ctrl=exposure_time_absolute=10
done
