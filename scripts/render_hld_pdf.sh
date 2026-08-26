#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

docker run --rm -v "$ROOT:/data" -w /data minlag/mermaid-cli \
  -i docs/hld/class-overview.mmd -o docs/hld/class-overview.png -b transparent

docker run --rm -v "$ROOT:/data" -w /data minlag/mermaid-cli \
  -i docs/hld/seq-comparative-cell.mmd -o docs/hld/seq-comparative-cell.png -b transparent

docker run --rm -v "$ROOT:/data" -w /data minlag/mermaid-cli \
  -i docs/hld/seq-drone-step.mmd -o docs/hld/seq-drone-step.png -b transparent

# Strip mermaid fences for pandoc (images already embedded via markdown image links).
# Pandoc reads docs/HLD.md; resource path includes docs/ so hld/*.png resolve.
docker run --rm -v "$ROOT:/data" -w /data/docs pandoc/latex \
  HLD.md -o /data/HLD.pdf --resource-path=. -f markdown -t pdf --pdf-engine=xelatex
