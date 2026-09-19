#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "${workspace_dir}"

status=0
fail() {
    printf '[FAIL] %s\n' "$1" >&2
    status=1
}

scan() {
    local label="$1"
    local pattern="$2"
    local output
    if output="$(rg -n --pcre2 --hidden \
        --glob '!.git/**' --glob '!build/**' --glob '!install/**' --glob '!log/**' \
        --glob '!runtime/**' --glob '!scripts/security_scan.sh' --glob '!SECURITY.md' \
        --glob '!*.pyc' "${pattern}" . 2>/dev/null)"; then
        fail "${label}"
        printf '%s\n' "${output}" >&2
    fi
}

scan '发现私钥、常见 token 前缀或硬编码用户目录' \
    '(BEGIN [A-Z ]*PRIVATE KEY|(?<![A-Za-z0-9])(sk-[A-Za-z0-9_-]{12,}|AIza[0-9A-Za-z_-]{20,}|gh[pousr]_[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16})(?![A-Za-z0-9])|/home/[A-Za-z0-9._-]+/)'
scan '发现 RFC1918 私网地址；硬件地址必须移入本机配置' \
    '(?<![0-9])(?:10\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}|172\.(?:1[6-9]|2[0-9]|3[01])\.[0-9]{1,3}\.[0-9]{1,3}|192\.168\.[0-9]{1,3}\.[0-9]{1,3})(?![0-9])'

provider_code_paths=(
    llmdecision
    llmModule/Adapters
    llmModule/Module1
    llmModule/Module4/route_planning
    llmModule/Module4/module4_ros_interface
    groundingdinoVLM/src/groundingdino_vlm
    groundingdinoVLM/src/piper_probe
    groundingdinoVLM/src/obstacle_traversal/obstacle_traversal
)
if output="$(rg -n --pcre2 --glob '*.py' --glob '!**/test/**' \
    '(?:type\(exc\).__name__|(?:LOGGER|self\.get_logger\(\))\.exception\(|str\(exc\)|\{exc\}|(?:LOGGER|self\.get_logger\(\))\.[a-z_]+\([^\n]*\bexc\b(?!\.(?:code|stage)))' \
    "${provider_code_paths[@]}" 2>/dev/null)"; then
    fail 'provider/ROS 边界存在未脱敏的异常类型、异常正文或 traceback 输出'
    printf '%s\n' "${output}" >&2
fi

if output="$(rg -n --pcre2 --glob '*.py' --glob '!**/test/**' \
    'raw_response_(?:file|path)|raw_vlm_response_file|write_text\([^\n]*\braw\b' \
    --glob '!**/test/**' --glob '!**/tests/**' \
    llmdecision llmModule/Module4 groundingdinoVLM/src 2>/dev/null)"; then
    fail '运行代码可能持久化 provider 原始响应或内部路径'
    printf '%s\n' "${output}" >&2
fi

if output="$(rg -n --pcre2 --hidden \
    --glob '*.{yaml,yml,json,toml,ini,cfg,conf,env}' \
    '(?i)^\s*["'"']?(?:api[_-]?key|access[_-]?key|secret|password|passwd|token)["'"']?\s*[:=]\s*["'"']?(?!\s*(?:$|null\b|none\b|false\b|true\b|<[^>]+>|\$\{|redacted\b|placeholder\b|example\b))[A-Za-z0-9+/=_-]{6,}' \
    . 2>/dev/null)"; then
    fail '配置文件中疑似存在明文认证值；只允许环境变量名或无效占位符'
    printf '%s\n' "${output}" >&2
fi

for generated in build install log acceptance_results migration_backup .codex .agents; do
    [[ ! -e "${generated}" ]] || fail "存在不应发布的生成目录：${generated}"
done

if find . -mindepth 1 -type d \( -name .git -o -name .hg -o -name .svn \) \
    -print -quit | rg -q .; then
    fail '存在嵌套版本库元数据，可能泄露历史、远端地址或本机配置'
fi

if find . -type l -print -quit | rg -q .; then
    fail '存在符号链接；公开发行件不得通过链接引用工作区外文件'
    find . -type l -printf '%p -> %l\n' >&2
fi

if find . -type d \( -name __pycache__ -o -name .pytest_cache -o -name '*.egg-info' \
    -o -name .cache -o -name .venv -o -name .deps \) -print -quit | rg -q .; then
    fail '存在缓存、虚拟环境或生成元数据目录'
    find . -type d \( -name __pycache__ -o -name .pytest_cache -o -name '*.egg-info' \
        -o -name .cache -o -name .venv -o -name .deps \) -print >&2
fi

if find . -type f \( -name '*.log' -o -name '*.bag' -o -name '*.db3' -o -name '*.mcap' \
    -o -name '*.pcap' -o -name '*.jsonl' -o -name '*.pkl' -o -name '*.pickle' \
    -o -name '*.npy' -o -name '*.npz' -o -name '*.sqlite' -o -name '*.sqlite3' \) \
    -print -quit | rg -q .; then
    fail '存在日志、rosbag、抓包或不透明序列化数据产物'
fi

allowed_checkpoint='./src/neupan_ros2/src/neupan_ros2/config/robots/scout/models/dune_model_5000.pth'
while IFS= read -r model; do
    [[ "${model}" == "${allowed_checkpoint}" ]] || fail "存在未列入许可清单的模型权重：${model}"
done < <(find . -type f \( -name '*.pt' -o -name '*.pth' -o -name '*.onnx' \
    -o -name '*.engine' -o -name '*.safetensors' \) -print | sort)

expected_sha='66c55fedae14bc4684f0dee9c37fe5d0e5a86abfce74ddf99af2663e85edb8de'
if [[ ! -f "${allowed_checkpoint}" ]]; then
    fail "缺少公开演示所需 Scout checkpoint：${allowed_checkpoint}"
elif [[ "$(sha256sum "${allowed_checkpoint}" | awk '{print $1}')" != "${expected_sha}" ]]; then
    fail "Scout checkpoint 校验值与 THIRD_PARTY_NOTICES.md 不一致"
fi

if [[ -d llmdecision/examples/images ]]; then
    fail 'examples/images 可能包含未授权场景数据；公开发行中不得存在'
fi

if (( status == 0 )); then
    printf '[PASS] 未发现已知凭据、私网地址、明文认证值、运行数据、生成物、外部链接或未许可权重。\n'
fi
exit "${status}"
