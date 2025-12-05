---
title: Command Line Interface
description: Use the Phospho CLI to interact with the Phospho API
---

Use the phospho CLI to run **offline tests.**

# Installation

The phospho CLI is a Python package. Install it with pip:

```bash
pip install phospho
phospho --version # Check the installation
```

# Initialization

<Info>Make sure you have installed the CLI and created a phospho account.</Info>


Login to the CLI with the `init` command:

```bash
phospho init
```

This does two things:


1. It stores phospho credentials in your home directory: `~/.phospho/config`. Use the `config` command to see the stored credentials:

```bash
phospho config
```

2. It creates a file `phospho_testing.py` in the current directory. You can [edit this file](/docs/python/testing) to customize your tests.


# Run the tests

To run the tests in `phospho_testing.py`, use the `test` command:

```bash
phospho test
```

Discover the results by following the link in the terminal output or by visiting the [phospho platform](https://platform.phospho.ai).

# Add tests and customize tests

Tests are written in Python. Edit the `phospho_testing.py` file to add your tests.

<div class="grid cards" markdown>

-   :material-language-python:{ .lg .middle } __phospho testing module__

    ---

    Learn how to edit phospho tests

    [:octicons-arrow-right-24: Read more](#)

</div>
