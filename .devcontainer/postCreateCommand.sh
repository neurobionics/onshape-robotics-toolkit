#! /usr/bin/env bash

# Install Dependencies
uv sync --dev --extra docs

# Install pre-commit hooks
uv run pre-commit install --install-hooks
