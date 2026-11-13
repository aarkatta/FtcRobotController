// Builds BioBuzzStarterbotAuto_How_It_Works.docx from BioBuzzStarterbotAuto_How_It_Works.md (+ BioBuzzStarterbotAuto_flowchart.png).
//
// Easiest: run ./BioBuzzStarterbotAuto_build_docs.sh in this folder (it does every step, including the PDF).
//
// By hand:
//   npm install --prefix "$TMPDIR/biobuzz-docx" docx@9.5.1          (once; keeps node_modules out of the repo)
//   NODE_PATH="$TMPDIR/biobuzz-docx/node_modules" node BioBuzzStarterbotAuto_build_docx.js
// Optional arguments: <input.md> <flowchart.png> <output.docx>  (defaults: the files next to this script)
//
// The Markdown this understands: # / ## / ### headings, paragraphs, "- " bullets, "- [ ] " checklist
// items, | tables |, and inline `code`, **bold**, *italic*. The line "*The flowchart ...*" is replaced by
// the flowchart image. Edit the flowchart in BioBuzzStarterbotAuto_flowchart.html, then re-render the PNG.
const fs = require("fs");
const path = require("path");
const {
  Document, Packer, Paragraph, TextRun, ImageRun, Table, TableRow, TableCell,
  AlignmentType, LevelFormat, WidthType, ShadingType, BorderStyle, HeadingLevel,
} = require("docx");

const here = (f) => path.join(__dirname, f);
const [
  mdPath = here("BioBuzzStarterbotAuto_How_It_Works.md"),
  pngPath = here("BioBuzzStarterbotAuto_flowchart.png"),
  outPath = here("BioBuzzStarterbotAuto_How_It_Works.docx"),
] = process.argv.slice(2);
const lines = fs.readFileSync(mdPath, "utf8").replace(/\r/g, "").split("\n");

const BODY_FONT = "Calibri";
const CODE_FONT = "Consolas";
const PAGE_CONTENT_DXA = 9360;          // US Letter 8.5in - 2 x 1in margins
const BORDER = { style: BorderStyle.SINGLE, size: 4, color: "BFBFBF" };

// ---------- inline markdown: `code`, **bold**, *italic* ----------
function inline(text, fmt = {}, extra = {}) {
  const runs = [];
  const re = /(`[^`]+`)|(\*\*(.+?)\*\*)|(\*([^*]+)\*)/;
  let rest = text;
  while (rest.length) {
    const m = rest.match(re);
    if (!m) { runs.push(run(rest, fmt, extra)); break; }
    if (m.index > 0) runs.push(run(rest.slice(0, m.index), fmt, extra));
    if (m[1]) runs.push(run(m[1].slice(1, -1), { ...fmt, code: true }, extra));
    else if (m[2]) runs.push(...inline(m[3], { ...fmt, bold: true }, extra));
    else runs.push(...inline(m[5], { ...fmt, italics: true }, extra));
    rest = rest.slice(m.index + m[0].length);
  }
  return runs;
}
function run(t, fmt, extra) {
  return new TextRun({
    text: t, bold: fmt.bold, italics: fmt.italics,
    font: fmt.code ? CODE_FONT : undefined,
    size: fmt.code ? (extra.codeSize || 20) : extra.size,
    color: extra.color,
  });
}

// ---------- tables ----------
function splitRow(line) {
  return line.trim().replace(/^\|/, "").replace(/\|$/, "").split("|").map(s => s.trim());
}
function plainLen(s) { return s.replace(/[`*]/g, "").length; }
function columnWidths(rows) {
  const n = rows[0].length;
  const weight = Array.from({ length: n }, (_, c) =>
    Math.max(8, Math.min(60, Math.max(...rows.map(r => plainLen(r[c] || ""))))));
  const total = weight.reduce((a, b) => a + b, 0);
  const w = weight.map(x => Math.floor(PAGE_CONTENT_DXA * x / total));
  w[n - 1] += PAGE_CONTENT_DXA - w.reduce((a, b) => a + b, 0);
  return w;
}
function makeTable(rows) {
  const widths = columnWidths(rows);
  return new Table({
    width: { size: PAGE_CONTENT_DXA, type: WidthType.DXA },
    columnWidths: widths,
    rows: rows.map((cells, r) => new TableRow({
      tableHeader: r === 0,
      cantSplit: true,
      children: cells.map((c, i) => new TableCell({
        width: { size: widths[i], type: WidthType.DXA },
        borders: { top: BORDER, bottom: BORDER, left: BORDER, right: BORDER },
        shading: r === 0 ? { type: ShadingType.CLEAR, color: "auto", fill: "F2F2F2" } : undefined,
        margins: { top: 60, bottom: 60, left: 100, right: 100 },
        children: [new Paragraph({
          spacing: { before: 0, after: 0 },
          children: inline(c, r === 0 ? { bold: true } : {}, { size: 20, codeSize: 18 }),
        })],
      })),
    })),
  });
}

// ---------- flowchart image ----------
function flowchart() {
  const png = fs.readFileSync(pngPath);
  const w = png.readUInt32BE(16), h = png.readUInt32BE(20);   // PNG IHDR
  const widthPx = 400;                                        // ~4.2 in at 96 dpi: fits on page 1 under the intro
  return new Paragraph({
    alignment: AlignmentType.CENTER,
    spacing: { before: 120, after: 120 },
    children: [new ImageRun({
      type: "png", data: png,
      transformation: { width: widthPx, height: Math.round(widthPx * h / w) },
      altText: {
        title: "BioBuzzStarterbotAuto steps",
        description: "Flowchart: INIT, SPIN_UP, SHOOT, LEAVE, TURN_TO_PARK, TO_PARK, FINAL_TURN, DONE, "
          + "with shortcuts from SPIN_UP to LEAVE (never at speed) and from TURN_TO_PARK to DONE (wrong way).",
        name: "BioBuzzStarterbotAuto flowchart",
      },
    })],
  });
}

// ---------- block parser ----------
const children = [];
let i = 0;
while (i < lines.length) {
  const line = lines[i];
  if (!line.trim()) { i++; continue; }

  if (line.startsWith("### ")) {
    children.push(new Paragraph({ heading: HeadingLevel.HEADING_2, children: inline(line.slice(4)) })); i++; continue;
  }
  if (line.startsWith("## ")) {
    children.push(new Paragraph({ heading: HeadingLevel.HEADING_1, children: inline(line.slice(3)) })); i++; continue;
  }
  if (line.startsWith("# ")) {
    children.push(new Paragraph({ heading: HeadingLevel.TITLE, children: inline(line.slice(2)) })); i++; continue;
  }
  // The Markdown points to the flowchart; in the Word file the flowchart itself goes here.
  if (/^\*The flowchart .*\*$/.test(line.trim())) { children.push(flowchart()); i++; continue; }

  if (line.trim().startsWith("|")) {
    const rows = [];
    while (i < lines.length && lines[i].trim().startsWith("|")) {
      const cells = splitRow(lines[i]);
      if (!cells.every(c => /^:?-{3,}:?$/.test(c))) rows.push(cells);   // skip the --- separator row
      i++;
    }
    children.push(makeTable(rows));
    children.push(new Paragraph({ spacing: { before: 0, after: 60 }, children: [] }));
    continue;
  }
  if (/^- \[[ x]\] /.test(line)) {
    children.push(new Paragraph({ numbering: { reference: "checks", level: 0 }, children: inline(line.slice(6)) }));
    i++; continue;
  }
  if (line.startsWith("- ")) {
    children.push(new Paragraph({ numbering: { reference: "bullets", level: 0 }, children: inline(line.slice(2)) }));
    i++; continue;
  }
  // paragraph: join following non-special lines
  let text = line.trim(); i++;
  while (i < lines.length && lines[i].trim() && !/^(#|\||- )/.test(lines[i].trim())) {
    text += " " + lines[i].trim(); i++;
  }
  const isDateLine = children.length === 1 && /^[A-Z][a-z]{2} \d{1,2}, \d{4}/.test(text);
  children.push(new Paragraph({
    children: inline(text, {}, isDateLine ? { color: "595959", size: 20 } : {}),
  }));
}

const doc = new Document({
  title: "BioBuzzStarterbotAuto: How It Works and What It Needs",
  description: "How the BioBuzzStarterbotAuto autonomous works, its hardware configuration and its telemetry",
  styles: {
    default: { document: { run: { font: BODY_FONT, size: 22 }, paragraph: { spacing: { after: 120, line: 264 } } } },
    paragraphStyles: [
      { id: "Title", name: "Title", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: BODY_FONT, size: 48, bold: true, color: "1F1F1F" },
        paragraph: { spacing: { before: 0, after: 80 }, keepNext: true } },
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: BODY_FONT, size: 32, bold: true, color: "1F1F1F" },
        paragraph: { spacing: { before: 320, after: 120 }, keepNext: true, keepLines: true, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { font: BODY_FONT, size: 26, bold: true, color: "404040" },
        paragraph: { spacing: { before: 240, after: 100 }, keepNext: true, keepLines: true, outlineLevel: 1 } },
    ],
  },
  numbering: {
    config: [
      { reference: "bullets", levels: [{ level: 0, format: LevelFormat.BULLET, text: "•", alignment: AlignmentType.LEFT,
        style: { paragraph: { indent: { left: 540, hanging: 270 }, spacing: { after: 80 } } } }] },
      { reference: "checks", levels: [{ level: 0, format: LevelFormat.BULLET, text: "☐", alignment: AlignmentType.LEFT,
        style: { paragraph: { indent: { left: 540, hanging: 300 }, spacing: { after: 80 } } } }] },
    ],
  },
  sections: [{
    properties: { page: { size: { width: 12240, height: 15840 },
      margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 } } },
    children,
  }],
});

Packer.toBuffer(doc).then(buf => { fs.writeFileSync(outPath, buf); console.log("wrote", outPath, buf.length, "bytes"); });
