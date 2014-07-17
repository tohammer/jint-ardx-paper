#!/bin/bash

for i in `ls msg_gen/cpp/include/normal/packet_*.h | grep -v init`; do
  echo $i;
  sed -i 's#    data.assign(0);#//    data.assign(0);#' $i;
  sed -i 's#  : data()#//  : data()#' $i;
done
