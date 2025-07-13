#! /usr/bin/env bash

# Install Dependencies
uv sync --dev

# Install pre-commit hooks
uv run pre-commit install --install-hooks
