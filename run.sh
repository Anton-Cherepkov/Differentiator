#!/usr/bin/env bash

mkdir graphs &>/dev/null

g++ main.cpp -o differentiator
./differentiator

cd graphs
for f in *.dot
do
    dot -Tpng $f -o "${f::-4}".png
done
