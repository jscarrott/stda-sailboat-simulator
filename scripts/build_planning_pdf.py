"""Render docs/planning.md to a self-contained PDF.

Pipeline: markdown -> HTML (python-markdown, with tables + fenced code)
-> PDF (WeasyPrint, which renders the embedded SVGs and the PNG).

Run:  uv run python scripts/build_planning_pdf.py
Out:  docs/planning.pdf
"""
from __future__ import annotations

from pathlib import Path

import markdown
from weasyprint import HTML, CSS

ROOT = Path(__file__).resolve().parents[1]
MD = ROOT / 'docs' / 'planning.md'
OUT = ROOT / 'docs' / 'planning.pdf'

CSS_STYLES = """
@page {
  size: A4;
  margin: 18mm 16mm;
  @bottom-center { content: counter(page) " / " counter(pages); font-size: 9pt; color: #666; }
}
body {
  font-family: "Helvetica", "Arial", sans-serif;
  font-size: 10.5pt;
  line-height: 1.45;
  color: #1f2328;
}
h1 { font-size: 22pt; margin: 0 0 12pt; }
h2 { font-size: 14pt; margin: 18pt 0 6pt; border-bottom: 1px solid #d0d7de; padding-bottom: 3pt; }
h3 { font-size: 12pt; margin: 14pt 0 4pt; }
p  { margin: 0 0 8pt; }
ul, ol { margin: 0 0 8pt 18pt; }
li { margin-bottom: 3pt; }
strong { color: #1f2328; }
code {
  font-family: "Menlo", "Consolas", monospace;
  font-size: 9.5pt;
  background: #f6f8fa;
  padding: 1pt 3pt;
  border-radius: 3px;
}
pre {
  background: #f6f8fa;
  padding: 8pt;
  border-radius: 4px;
  overflow-x: auto;
  font-size: 9.5pt;
}
table {
  border-collapse: collapse;
  margin: 8pt 0 14pt;
  width: 100%;
  font-size: 10pt;
}
th, td {
  border: 1px solid #d0d7de;
  padding: 5pt 8pt;
  text-align: left;
  vertical-align: top;
}
th { background: #f6f8fa; }
img {
  max-width: 100%;
  height: auto;
  display: block;
  margin: 8pt auto;
  page-break-inside: avoid;
}
hr { border: none; border-top: 1px solid #d0d7de; margin: 14pt 0; }
"""


def main() -> None:
    md_text = MD.read_text()
    html_body = markdown.markdown(
        md_text,
        extensions=['tables', 'fenced_code', 'sane_lists'],
    )
    html_doc = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>Route planning</title></head>
<body>{html_body}</body></html>"""

    # base_url lets relative image paths (diagrams/foo.svg) resolve.
    HTML(string=html_doc, base_url=str(MD.parent)).write_pdf(
        target=str(OUT),
        stylesheets=[CSS(string=CSS_STYLES)],
    )
    print(f'wrote {OUT} ({OUT.stat().st_size / 1024:.1f} KB)')


if __name__ == '__main__':
    main()
