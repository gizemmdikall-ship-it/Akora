#!/bin/bash
# Re-launches the Blender render after a crash; film.py skips frames that already exist.
cd "$(dirname "$0")/../.."
N=${1:-1332}
for attempt in $(seq 1 60); do
  have=$(ls output/film/frames 2>/dev/null | wc -l)
  [ "$have" -ge "$N" ] && break
  echo "== attempt $attempt, frames $have" >> output/film/render.log
  python3 anim/blender/film.py render 0 "$N" >> output/film/render.log 2>&1
done
echo "== supervisor done" >> output/film/render.log
