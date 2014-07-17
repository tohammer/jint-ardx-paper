#!/bin/bash

for i in 4 100 1000 10000 100000 1000000 10000000 100000000; do
  echo "uint8[$i] data" > msg/packet_${i}.msg;
  echo "uint8[$i] data" > msg/packet_${i}_init.msg;
done

