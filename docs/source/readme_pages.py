"""
Pages of the documentation generated from the README files of the repository.

The README files remain the only source of their contents: the pages are written into
`pages/` at the start of the build. The relative links of the README files are
converted to links to the pages, or to the files on GitHub.

"""

import re
from pathlib import Path

ROOT = Path(__file__).parents[2]
OUT = Path(__file__).parent / "pages"
GITHUB = "https://github.com/Aalto-Electric-Drives/motulator-export"

# Pages: name -> (title, README file, sections), the title None for the title of the
# section or the file, and the sections None for the whole file
PAGES: dict[str, tuple[str | None, str, list[str] | None]] = {
    "installation": ("Installation", "README.md", ["Requirements", "Getting started"]),
    "export": (
        "PLECS and Simulink export",
        "README.md",
        [
            "Exporting your own system",
            "Simulating from Python",
            "Working with the models",
        ],
    ),
    "native": (None, "README.md", ["Native simulation"]),
    "supported": (None, "README.md", ["Supported systems"]),
    "development": (None, "README.md", ["Development"]),
    "plecs": ("PLECS models", "examples/README.md", None),
    "simulink": (None, "motulator_export/simulink/README.md", None),
    "c_port": (None, "motulator_export/c/README.md", None),
    "matlab_port": (None, "motulator_export/matlab/README.md", None),
}
# Links to the sections of the main README and to the other documents
ANCHORS = {"#native-simulation": "native.md", "#supported-systems": "supported.md"}
DOCS = {"README.md": "../index.md", "examples/native": "../examples/index.rst"}

LINK = re.compile(r"(?<!!)\[([^\]]+)\]\(([^)\s]+)\)")


def _sections(text: str) -> dict[str, str]:
    """Sections of the file by the titles of the level-2 headings."""
    sections, title = {}, None
    for line in text.splitlines(keepends=True):
        if line.startswith("## "):
            title = line[3:].strip()
            sections[title] = ""
        elif title is not None:
            sections[title] += line
    return sections


def _link(source: str, target: str) -> str:
    """Target of a link of the file `source` in the documentation."""
    if re.match(r"[a-z]+:", target):
        return target
    if target.startswith("#"):
        return ANCHORS.get(target, target) if source == "README.md" else target
    path, _, anchor = target.partition("#")
    resolved = (ROOT / source).parent.joinpath(path).resolve().relative_to(ROOT)
    repo_path = resolved.as_posix()
    pages = {file: name for name, (_, file, sections) in PAGES.items() if not sections}
    if repo_path in pages:
        doc = f"{pages[repo_path]}.md"
    elif repo_path in DOCS:
        doc = DOCS[repo_path]
    else:
        kind = "tree" if (ROOT / resolved).is_dir() else "blob"
        return f"{GITHUB}/{kind}/main/{repo_path}" + (f"#{anchor}" if anchor else "")
    return doc + (f"#{anchor}" if anchor else "")


def _page(title: str | None, source: str, sections: list[str] | None) -> str:
    """Contents of a page."""
    text = (ROOT / source).read_text(encoding="utf-8")
    if sections is None:
        if title is not None:
            text = re.sub(r"\A# .*\n", f"# {title}\n", text)
    else:
        parts = _sections(text)
        if len(sections) == 1 and title is None:
            # The section as the page, its subsections one level up
            body = re.sub(r"^###", "##", parts[sections[0]], flags=re.M)
            text = f"# {sections[0]}\n" + body
        else:
            text = f"# {title}\n\n" + "".join(f"## {s}\n{parts[s]}" for s in sections)
    note = f"<!-- Generated from {source} by docs/source/readme_pages.py -->\n\n"
    return note + LINK.sub(lambda m: f"[{m[1]}]({_link(source, m[2])})", text)


def write_pages() -> None:
    """Write the pages generated from the README files."""
    OUT.mkdir(exist_ok=True)
    for name, (title, source, sections) in PAGES.items():
        (OUT / f"{name}.md").write_text(
            _page(title, source, sections), encoding="utf-8"
        )
