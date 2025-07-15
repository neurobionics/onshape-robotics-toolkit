.PHONY: install
install: ## Install the uv environment and install the pre-commit hooks
	@uv venv
	@uv sync --dev
	@uv run maturin develop --release
	@uv run pre-commit install

.PHONY: check
check: ## Run code quality tools.
	@uv run pre-commit run -a
	@uv run ruff check .
	@uv run deptry .

.PHONY: test
test: ## Test the code with pytest
	@uv run pytest --cov --cov-config=pyproject.toml --cov-report=xml

.PHONY: build
build: clean-build ## Build wheel file using uv
	@uv run maturin develop --release

.PHONY: clean-build
clean-build: ## clean build artifacts
	@if exist dist rmdir /s /q dist

.PHONY: version
version: ## Update the project version
	@uv version $VERSION

.PHONY: publish
publish: ## publish a release to pypi.
	@uv publish --dry-run
	@uv publish

.PHONY: build-and-publish
build-and-publish: build publish ## Build and publish.

.PHONY: docs-test
docs-test: ## Test if documentation can be built without warnings or errors
	@uv run mkdocs build -s

.PHONY: docs
docs: ## Build and serve the documentation
	@uv run mkdocs serve

.PHONY: docs-deploy
docs-deploy: ## Deploy the documentation to GitHub pages
	@uv run mkdocs gh-deploy

.PHONY: help
help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

.DEFAULT_GOAL := help
