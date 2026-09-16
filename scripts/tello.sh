#!/usr/bin/env bash
# Menu front end for the flight workflow.
#
# Every option here composes one of the command lines documented in the
# README and then SHOWS IT before running it. That is deliberate: the point is
# to stop you retyping twenty-character flag soup, not to hide what the tools
# are doing - when something misbehaves you still need to know the exact
# command to reason about, quote in a bug report, or run by hand.
set -uo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BUILD_DIR=${BUILD_DIR:-build}
CONFIG_FILE=${TELLO_LAUNCHER_CONFIG:-.tello-launcher.conf}

bold=$'\e[1m'; dim=$'\e[2m'; red=$'\e[31m'; green=$'\e[32m'
yellow=$'\e[33m'; cyan=$'\e[36m'; reset=$'\e[0m'

# ---------------------------------------------------------------- settings --
# Defaults are the values that have actually flown well, not library defaults.
declare -A S=(
    [ip]=192.168.10.1
    [camera]=config/tello_camera.yaml
    [max_speed]=100
    [cruise]=40
    [corner_limit]=5
    [corner_brake]=4.0
    [corner_min]=0.2
    [centering]=1.0
    [spacing]=0.8
    [skip]=5
    [limit]=12
    [repeat]=1
    [board_cols]=8
    [board_rows]=10
    [square]=0.025
)

load_settings() {
    [[ -f $CONFIG_FILE ]] || return 0
    while IFS='=' read -r key value; do
        [[ -z ${key// } || ${key:0:1} == '#' ]] && continue
        [[ -v S[$key] ]] && S[$key]=$value
    done < "$CONFIG_FILE"
}

save_settings() {
    {
        echo "# Written by scripts/tello.sh; safe to edit or delete."
        for key in "${!S[@]}"; do echo "$key=${S[$key]}"; done
    } > "$CONFIG_FILE"
}

# ------------------------------------------------------------------ helpers --
die() { printf '%s\n' "${red}$*${reset}" >&2; return 1; }

need_binary() {
    local path=$BUILD_DIR/$1
    if [[ ! -x $path ]]; then
        printf '%s\n' "${red}找不到 $path${reset}"
        printf '%s\n' "請先建置：${bold}cmake -S . -B $BUILD_DIR -DBUILD_SLAM=ON -DBUILD_TOOLS=ON && cmake --build $BUILD_DIR -j${reset}"
        return 1
    fi
    printf '%s' "$path"
}

ask() {  # ask <prompt> <settings-key>
    local prompt=$1 key=$2 answer
    read -r -p "$prompt [${cyan}${S[$key]}${reset}]: " answer
    [[ -n $answer ]] && S[$key]=$answer
    return 0
}

ask_path() {  # ask_path <prompt> <default> -> echoes the answer
    local prompt=$1 default=$2 answer
    read -r -p "$prompt [${cyan}${default}${reset}]: " answer
    printf '%s' "${answer:-$default}"
}

confirm() {
    local answer
    read -r -p "${bold}執行？${reset} [Y/n] " answer
    [[ -z $answer || $answer =~ ^[Yy] ]]
}

drone_reachable() {
    ping -c1 -W1 "${S[ip]}" >/dev/null 2>&1
}

require_drone() {
    if drone_reachable; then
        printf '%s\n' "${green}無人機 ${S[ip]} 有回應${reset}"
        return 0
    fi
    printf '%s\n' "${yellow}ping 不到 ${S[ip]}${reset} — 確認已連上 Tello 的 WiFi。"
    local answer
    read -r -p "仍要繼續嗎？ [y/N] " answer
    [[ $answer =~ ^[Yy] ]]
}

# Shows the command, then runs it with the terminal untouched so the app's own
# output and its OpenCV/Pangolin windows behave exactly as they would by hand.
run_command() {
    printf '\n%s\n' "${dim}────────────────────────────────────────${reset}"
    printf '%s\n' "${bold}$*${reset}"
    printf '%s\n\n' "${dim}────────────────────────────────────────${reset}"
    confirm || { printf '%s\n' "已取消。"; return 0; }
    save_settings
    "$@"
    local status=$?
    printf '\n'
    if [[ $status -eq 0 ]]; then
        printf '%s\n' "${green}完成（結束碼 0）${reset}"
    else
        printf '%s\n' "${red}結束碼 $status${reset}"
    fi
    read -r -p "按 Enter 回到選單…" _
}

# ------------------------------------------------------- annotated pickers --
map_note() {  # map_note <map.osa> -> "344 keyframes, 尺度未校正"
    local meta=${1%.osa}.meta.yaml
    [[ -f $meta ]] || { printf '%s' "${yellow}無 metadata${reset}"; return; }
    local kf scale calibrated
    kf=$(grep -m1 '^keyframes:' "$meta" | awk '{print $2}')
    scale=$(grep -m1 '^scale_metres_per_unit:' "$meta" | awk '{print $2}')
    calibrated=$(grep -m1 '^scale_calibrated:' "$meta" | awk '{print $2}')
    if [[ $calibrated == 1 ]]; then
        printf '%s' "${kf:-?} 關鍵幀, 尺度 ${scale} m/unit"
    else
        printf '%s' "${kf:-?} 關鍵幀, ${yellow}尺度未校正${reset}"
    fi
}

mission_note() {  # mission_note <mission.yaml>
    local count map
    count=$(grep -c '^      label:' "$1" 2>/dev/null || echo 0)
    map=$(grep -m1 '^map_path:' "$1" | sed 's/^map_path: *//; s/"//g')
    printf '%s' "${count} 航點 → $(basename "${map:-?}")"
}

recording_note() {
    local mb
    mb=$(du -m "$1" 2>/dev/null | cut -f1)
    printf '%s' "${mb} MB, $(date -r "$1" '+%m-%d %H:%M')"
}

# pick <glob> <noter-fn> <what>; echoes the chosen path, empty if none/cancel
pick() {
    local glob=$1 noter=$2 what=$3
    local -a items=()
    local file
    for file in $glob; do [[ -e $file ]] && items+=("$file"); done
    if [[ ${#items[@]} -eq 0 ]]; then
        printf '%s\n' "${yellow}找不到任何${what}${reset}" >&2
        return 1
    fi

    printf '\n%s\n' "${bold}選擇${what}：${reset}" >&2
    local i
    for i in "${!items[@]}"; do
        printf '  %2d) %-46s %s\n' "$((i+1))" "${items[$i]}" "$($noter "${items[$i]}")" >&2
    done
    printf '   0) 取消\n' >&2

    local choice
    read -r -p "> " choice
    [[ $choice =~ ^[0-9]+$ ]] || return 1
    [[ $choice -eq 0 || $choice -gt ${#items[@]} ]] && return 1
    printf '%s' "${items[$((choice-1))]}"
}

# ------------------------------------------------------------------- tasks --
task_record() {
    local bin; bin=$(need_binary apps/video_view/video_view) || return 0
    require_drone || return 0
    ask "無人機 IP" ip
    local name; name=$(ask_path "錄影檔名" "recordings/route_$(date +%m%d_%H%M).h264")
    mkdir -p "$(dirname "$name")"
    run_command "$bin" --ip "${S[ip]}" --record "$name"
}

task_build_map() {
    local bin; bin=$(need_binary apps/map_builder/map_builder) || return 0
    local video; video=$(pick 'recordings/*.h264' recording_note "錄影") || return 0
    local base; base=$(basename "$video" .h264)
    local out; out=$(ask_path "輸出地圖" "maps/${base}.osa")
    run_command "$bin" --video "$video" --camera "${S[camera]}" --out "$out"
}

task_calibrate_scale() {
    local bin; bin=$(need_binary apps/tools/calibrate_scale/calibrate_scale) || return 0
    local map; map=$(pick 'maps/*.osa' map_note "地圖") || return 0
    require_drone || return 0
    local distance; distance=$(ask_path "兩個標記之間的實際距離（公尺）" "2.0")
    run_command "$bin" --map "$map" --drone --ip "${S[ip]}" \
        --camera "${S[camera]}" --distance "$distance" --no-viewer
}

task_relocalize() {
    local bin; bin=$(need_binary apps/relocalize/relocalize) || return 0
    local map; map=$(pick 'maps/*.osa' map_note "地圖") || return 0

    printf '\n%s\n' "${bold}用什麼來源驗證？${reset}"
    printf '  1) 錄影（離線，不需無人機）\n  2) 即時串流（需要無人機）\n  0) 取消\n'
    local choice; read -r -p "> " choice
    case $choice in
        1)  local video; video=$(pick 'recordings/*.h264' recording_note "錄影") || return 0
            run_command "$bin" --map "$map" --video "$video" \
                --camera "${S[camera]}" --no-viewer --no-preview ;;
        2)  require_drone || return 0
            run_command "$bin" --map "$map" --drone --ip "${S[ip]}" \
                --camera "${S[camera]}" --no-viewer ;;
        *)  return 0 ;;
    esac
}

task_make_mission() {
    local bin; bin=$(need_binary apps/tools/trajectory_to_mission/trajectory_to_mission) || return 0
    local map; map=$(pick 'maps/*.osa' map_note "地圖") || return 0

    local meta=${map%.osa}.meta.yaml
    if [[ -f $meta ]] && [[ $(grep -m1 '^scale_calibrated:' "$meta" | awk '{print $2}') != 1 ]]; then
        printf '%s\n' "${yellow}這張地圖的尺度還沒校正，產出的航線距離不是公尺，autonomous_mission 會拒飛。${reset}"
    fi

    ask "航點間距（公尺）" spacing
    ask "略過開頭幾個航點" skip
    ask "最多保留幾個航點（0 = 全部）" limit
    local base; base=$(basename "$map" .osa)
    local out; out=$(ask_path "輸出航線" "config/mission_${base}.yaml")

    local -a cmd=("$bin" --map "$map" --out "$out"
                  --spacing "${S[spacing]}" --skip "${S[skip]}")
    [[ ${S[limit]} -gt 0 ]] && cmd+=(--limit "${S[limit]}")
    run_command "${cmd[@]}"
}

task_fly() {
    local bin; bin=$(need_binary apps/autonomous_mission/autonomous_mission) || return 0
    local map; map=$(pick 'maps/*.osa' map_note "地圖") || return 0
    local mission; mission=$(pick 'config/mission*.yaml' mission_note "航線") || return 0

    printf '\n%s\n' "${bold}怎麼飛？${reset}"
    printf '  1) 真機飛行\n  2) 用錄影 dry run（不送指令）\n  0) 取消\n'
    local mode; read -r -p "> " mode
    [[ $mode == 1 || $mode == 2 ]] || return 0

    printf '\n%s\n' "${bold}飛行參數${reset} ${dim}（Enter 沿用上次的值）${reset}"
    ask "  巡航舵量 cruise（速度旋鈕）" cruise
    ask "  舵量上限 max-speed" max_speed
    ask "  轉角預視限制（度）" corner_limit
    ask "  轉角提前煞車（秒）" corner_brake
    ask "  轉角最低巡航比例" corner_min
    ask "  路徑向心權重" centering
    ask "  重複趟數" repeat

    local -a cmd=("$bin" --map "$map" --mission "$mission" --camera "${S[camera]}"
                  --max-speed "${S[max_speed]}" --cruise "${S[cruise]}"
                  --corner-limit "${S[corner_limit]}" --corner-brake "${S[corner_brake]}"
                  --corner-min "${S[corner_min]}" --centering "${S[centering]}")
    [[ ${S[repeat]} -gt 1 ]] && cmd+=(--repeat "${S[repeat]}" --ping-pong)

    if [[ $mode == 1 ]]; then
        require_drone || return 0
        cmd+=(--ip "${S[ip]}" --record "recordings/flight_$(date +%m%d_%H%M).h264" --no-viewer)
        printf '\n%s\n' "${yellow}真機飛行：預覽視窗要有鍵盤焦點才能人工接管。${reset}"
        printf '%s\n' "${yellow}space=接管  m=交還  Esc=降落  x=緊急停止${reset}"
    else
        local video; video=$(pick 'recordings/*.h264' recording_note "錄影") || return 0
        cmd+=(--video "$video" --realtime --dry-run --no-viewer --no-preview)
    fi
    run_command "${cmd[@]}"
}

task_manual() {
    local bin; bin=$(need_binary apps/manual_control_gui/manual_control_gui) || return 0
    require_drone || return 0
    run_command "$bin" --ip "${S[ip]}"
}

task_calibrate_camera() {
    local bin; bin=$(need_binary apps/tools/calibrate_camera/calibrate_camera) || return 0
    local video; video=$(pick 'recordings/*.h264' recording_note "棋盤格錄影") || return 0
    printf '%s\n' "${dim}--cols/--rows 數的是「內角點」，比方格數少一。11x9 方格的板子是 8 x 10。${reset}"
    ask "內角點（橫向）" board_cols
    ask "內角點（縱向）" board_rows
    ask "方格邊長（公尺）" square
    local out; out=$(ask_path "輸出設定檔" "${S[camera]}")
    run_command "$bin" --video "$video" --cols "${S[board_cols]}" --rows "${S[board_rows]}" \
        --square "${S[square]}" --views 30 --out "$out"
}

task_inventory() {
    printf '\n%s\n' "${bold}地圖${reset}"
    local f
    for f in maps/*.osa; do [[ -e $f ]] && printf '  %-46s %s\n' "$f" "$(map_note "$f")"; done
    printf '\n%s\n' "${bold}錄影${reset}"
    for f in recordings/*.h264; do [[ -e $f ]] && printf '  %-46s %s\n' "$f" "$(recording_note "$f")"; done
    printf '\n%s\n' "${bold}航線${reset}"
    for f in config/mission*.yaml; do [[ -e $f ]] && printf '  %-46s %s\n' "$f" "$(mission_note "$f")"; done
    printf '\n%s %s\n' "${bold}相機設定${reset}" "${S[camera]}"
    printf '\n'
    read -r -p "按 Enter 回到選單…" _
}

# -------------------------------------------------------------------- main --
main_menu() {
    while true; do
        clear
        printf '%s\n' "${bold}Tello EDU + ORB-SLAM3${reset}  ${dim}(${BUILD_DIR}, 無人機 ${S[ip]})${reset}"
        printf '%s\n\n' "${dim}────────────────────────────────────────${reset}"
        printf '  %s\n' "${bold}建圖流程${reset}"
        printf '   1) 錄製影片           %s\n' "${dim}video_view --record${reset}"
        printf '   2) 從影片建立地圖     %s\n' "${dim}map_builder${reset}"
        printf '   3) 校正地圖尺度       %s\n' "${dim}calibrate_scale（需無人機）${reset}"
        printf '   4) 驗證定位           %s\n' "${dim}relocalize${reset}"
        printf '   5) 產生航線           %s\n' "${dim}trajectory_to_mission${reset}"
        printf '\n  %s\n' "${bold}飛行${reset}"
        printf '   6) 自主飛行 / dry run %s\n' "${dim}autonomous_mission${reset}"
        printf '   7) 手動遙控           %s\n' "${dim}manual_control_gui${reset}"
        printf '\n  %s\n' "${bold}其他${reset}"
        printf '   8) 相機校正           %s\n' "${dim}calibrate_camera${reset}"
        printf '   9) 檢視現有素材\n'
        printf '   0) 離開\n\n'
        local choice
        read -r -p "> " choice
        case $choice in
            1) task_record ;;
            2) task_build_map ;;
            3) task_calibrate_scale ;;
            4) task_relocalize ;;
            5) task_make_mission ;;
            6) task_fly ;;
            7) task_manual ;;
            8) task_calibrate_camera ;;
            9) task_inventory ;;
            0) save_settings; printf '\n'; exit 0 ;;
            *) ;;
        esac
    done
}

load_settings
main_menu
