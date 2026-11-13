#!/bin/bash
# Rebuilds the BioBuzzStarterbotAuto docs from BioBuzzStarterbotAuto_How_It_Works.md (the file you edit):
#   1. BioBuzzStarterbotAuto_flowchart.html -> BioBuzzStarterbotAuto_flowchart.png   (Google Chrome, headless)
#   2. .md + .png                -> BioBuzzStarterbotAuto_How_It_Works.docx (Node.js + the "docx" package)
#   3. .docx                     -> BioBuzzStarterbotAuto_How_It_Works.pdf  (Microsoft Word; skipped if Word is missing)
#
# Usage (macOS, from anywhere):   TeamCode/docs/BioBuzzStarterbotAuto_build_docs.sh
# Needs: Node.js, Google Chrome. Word only for the PDF.
# The docx package is installed in a temp folder, so no node_modules ends up in the repo.
set -euo pipefail

DOCS="$(cd "$(dirname "$0")" && pwd)"
WORK="${TMPDIR:-/tmp}/biobuzz-docx"
CHROME="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
mkdir -p "$WORK"

echo "1/3 Flowchart PNG"
"$CHROME" --headless=new --disable-gpu --hide-scrollbars --force-device-scale-factor=2 \
    --window-size=672,785 --user-data-dir="$WORK/chrome-profile" \
    --screenshot="$DOCS/BioBuzzStarterbotAuto_flowchart.png" "file://$DOCS/BioBuzzStarterbotAuto_flowchart.html" >/dev/null 2>&1

echo "2/3 Word document"
if [ ! -d "$WORK/node_modules/docx" ]; then
    npm install --silent --prefix "$WORK" docx@9.5.1 >/dev/null
fi
NODE_PATH="$WORK/node_modules" node "$DOCS/BioBuzzStarterbotAuto_build_docx.js" 2>/dev/null
# Drop the ZIP's folder entries: Word can refuse a .docx that has them when opened via cloud co-editing.
zip -d "$DOCS/BioBuzzStarterbotAuto_How_It_Works.docx" '*/' >/dev/null 2>&1 || true

echo "3/3 PDF"
if [ -d "/Applications/Microsoft Word.app" ]; then
    # Word is sandboxed: export inside its own container folder, then copy the PDF back.
    BOX="$HOME/Library/Containers/com.microsoft.Word/Data/Documents/ultron-docs-build"
    mkdir -p "$BOX"
    cp "$DOCS/BioBuzzStarterbotAuto_How_It_Works.docx" "$BOX/doc.docx"
    rm -f "$BOX/doc.pdf"
    osascript -e "
        tell application \"Microsoft Word\"
            open (POSIX file \"$BOX/doc.docx\")
            set d to active document
            save as d file name \"$BOX/doc.pdf\" file format format PDF
            close d saving no
        end tell" >/dev/null
    cp "$BOX/doc.pdf" "$DOCS/BioBuzzStarterbotAuto_How_It_Works.pdf"
    rm -rf "$BOX"
else
    echo "    Microsoft Word not found: PDF not updated (open the .docx and save it as PDF)."
fi

echo "Done: $DOCS/BioBuzzStarterbotAuto_How_It_Works.docx and .pdf"
