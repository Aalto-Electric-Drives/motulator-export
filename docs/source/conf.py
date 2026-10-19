# ruff: noqa

# Configuration file for the Sphinx documentation builder. The configuration follows
# that of motulator, so that the documentation has the same style.
#
# For a full list of options, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup ------------------------------------------------------------------------

import os
import sys
from importlib.metadata import version as package_version

sys.path.insert(0, os.path.abspath("../.."))
sys.path.insert(0, os.path.abspath("."))

os.environ["BUILDING_DOCS"] = "1"

# -- Project information ---------------------------------------------------------------

project = "motulator-export"
copyright = "2026, Aalto Electric Drives"
author = "Aalto Electric Drives"

# The full version, including alpha/beta/rc tags
release = package_version("motulator-export")

# -- General configuration -------------------------------------------------------------

extensions = [
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "numpydoc",
    "sphinx_copybutton",
    "sphinx.ext.mathjax",
    "sphinx_gallery.gen_gallery",
    "myst_parser",
]

myst_enable_extensions = [
    "dollarmath",  # For $...$ and $$...$$ math
    "deflist",  # For definition lists
    "html_admonition",  # For note/warning boxes
    "html_image",  # For images in HTML
    "colon_fence",  # For ::: fenced blocks
]
myst_heading_anchors = 3

source_suffix = {".rst": "restructuredtext", ".md": "markdown"}

# Links to the documentation of motulator, e.g., {class}`motulator.drive.model.Drive`
intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "motulator": ("https://aalto-electric-drives.github.io/motulator/", None),
}

extensions.append("autoapi.extension")
autoapi_type = "python"
autoapi_dirs = ["../../motulator_export"]
autodoc_typehints = "description"
autodoc_typehints_format = "short"
autoapi_options = [
    "members",
    "show-inheritance",
    "show-module-summary",
    "imported-members",
]
autoapi_template_dir = "../_templates/autoapi"
autoapi_python_class_content = "class"
autoapi_keep_files = True
autoapi_add_toctree_entry = False
autoapi_member_order = "alphabetical"

sphinx_gallery_conf = {
    "examples_dirs": ["../../examples/native"],
    "gallery_dirs": ["examples"],
    "within_subsection_order": "gallery_order.example_order",
}

exclude_patterns = ["_build", "build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output -----------------------------------------------------------

html_theme = "sphinx_book_theme"
html_theme_options = {
    "repository_url": "https://github.com/Aalto-Electric-Drives/motulator-export",
    "use_repository_button": True,
    "navigation_with_keys": False,
}
html_static_path = ["_static"]
html_css_files = ["css/custom.css"]
