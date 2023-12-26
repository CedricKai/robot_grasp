#!/bin/bash
set -x        # 在执行每条命令前先打印出来，用于调试。
set -e        # 在任何命令返回非零退出状态时立即退出脚本，这意味着如果任何命令失败，脚本就会停止。

DIR="$( cd "$( dirname -- "$0" )" && pwd )"            # 获取当前脚本所在的目录
export PYTHONUNBUFFERED="True"                         # 输出不会被缓冲，可以实时看到输出结果

DATE=${1-"test"}

LOG="output/${RUN_NAME}/log.txt"        # 令定义了一个变量LOG，它的值是日志文件的路径
exec &> >(tee -a "$LOG")                # 将标准输出和标准错误输出重定向到日志文件
echo Logging output to "$LOG"           # 命令在控制台打印日志文件的路径

python -m core.grasp --date ${DATE}
