#!/usr/bin/env bash
set -e
for cfg in ../config/*.yaml; do
  xml=../mujoco/$(basename "${cfg%.*}.xml")
  python yaml2mjcf.py "$cfg" "$xml"
done
echo "All YAML → XML conversions done."
