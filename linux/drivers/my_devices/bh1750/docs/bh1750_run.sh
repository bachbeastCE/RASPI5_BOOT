#!/bin/bash

DEVICE="/dev/bh1750"
sudo chmod a+r /dev/bh1750

if [ ! -e "$DEVICE" ]; then
    echo "Thiết bị $DEVICE không tồn tại."
    exit 1
fi

while true; do
    RAW=$(cat "$DEVICE" 2>/dev/null)
    
    # Kiểm tra nếu dữ liệu là số
    if [[ "$RAW" =~ ^[0-9]+$ ]]; then
        LUX=$(echo "scale=2; $RAW / 1.2" | bc)
        echo "Lux = $LUX"
    else
        echo "Không nhận được dữ liệu hợp lệ: $RAW"
    fi

    sleep 1
done