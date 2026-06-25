# 1. locate the bag to augment
  SRC_BAG=bags/rosbag2_2026_06_24-12_00_25
  NEW_BAG=bags/rosbag2_2026_06_24-12_00_25_decompressed
  echo "$SRC_BAG -> $NEW_BAG"

  # 2. start the decompressor (compressed -> raw)
  ros2 run image_transport republish \
    --ros-args \
    -p in_transport:=compressed \
    -p out_transport:=raw \
    -r in/compressed:=/camera/image_raw/compressed \
    -r out:=/camera/image_raw &
  REPUB_PID=$!
  sleep 2   # give discovery time to connect before playback starts

  # 3. record everything plus the new decompressed topic
  ros2 bag record --storage mcap -o "$NEW_BAG" -a &
  RECORD_PID=$!
  sleep 2

  # 4. play the source bag through once
  ros2 bag play "$SRC_BAG" --clock --rate 1.0

  # 5. tear down
  kill $RECORD_PID; wait $RECORD_PID 2>/dev/null
  kill $REPUB_PID; wait $REPUB_PID 2>/dev/null
