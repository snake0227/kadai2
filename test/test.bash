#!/bin/bash
# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Identifier: BSD-3-Clause

ng(){
    echo "${1}行目が違う"
    res=1
}

res=0

# --- ファイルの場所を自動探索 ---
# カレントディレクトリ配下から monitor.py を探す
SCRIPT_PATH=$(find . -name "monitor.py" | head -n 1)

if [ -z "$SCRIPT_PATH" ]; then
    echo "エラー: monitor.py が見つかりません"
    exit 1
fi

echo "Found script at: $SCRIPT_PATH"
# monitor.py が入っているディレクトリを PYTHONPATH に追加
export PYTHONPATH="$(dirname $SCRIPT_PATH):$PYTHONPATH"

# 環境設定
source /opt/ros/humble/setup.bash

# --- Test 1: 正常系 ---
echo "Test 1: Running monitor.py"

# バックグラウンド起動
python3 "$SCRIPT_PATH" /tmp > /tmp/system_test.log 2>&1 &
PID=$!

# トピックが出るまで待機
echo "Waiting for /cpu_usage topic..."
COUNT=0
while ! ros2 topic list | grep -q '/cpu_usage'; do
    sleep 1
    COUNT=$((COUNT+1))
    if [ $COUNT -ge 20 ]; then
        echo "Timeout: Topic not found"
        echo "--- Python Runtime Log ---"
        cat /tmp/system_test.log
        ng "$LINENO"
        break
    fi
done

# データ受信確認
if [ "$res" = 0 ]; then
    # トピック名が /cpu_usage でない可能性に備え、念のためリストを表示
    ros2 topic list
    timeout 10s ros2 topic echo /cpu_usage std_msgs/msg/Float32 --once || ng "$LINENO"
fi

kill $PID 2>/dev/null

# --- Test 2 & 3: 異常系 ---
echo "Test 2: Invalid path check"
python3 "$SCRIPT_PATH" /non_existent_999 > /dev/null 2>&1
[ "$?" = 1 ] || ng "$LINENO"

echo "Test 3: No argument check"
python3 "$SCRIPT_PATH" > /dev/null 2>&1
[ "$?" = 1 ] || ng "$LINENO"

[ "$res" = 0 ] && echo "ALL TESTS PASSED"
exit $res
