# Contributing to `onshape-robotics-toolkit`

Contributions are welcome, and they are greatly appreciated!
Every little bit helps, and credit will always be given.

# Types of Contributions

## Report Bugs

Report bugs at https://github.com/neurobionics/onshape-robotics-toolkit/issues

If you are reporting a bug, please include:

- Your operating system name and version.
- Any details about your local setup that might be helpful in troubleshooting.
- Detailed steps to reproduce the bug.

## Fix Bugs

Look through the GitHub issues for bugs.
Anything tagged with "bug" and "help wanted" is open to whoever wants to implement a fix for it.

## Implement Features

Look through the GitHub issues for features.
Anything tagged with "enhancement" and "help wanted" is open to whoever wants to implement it.

## Write Documentation

`onshape-robotics-toolkit` could always use more documentation, whether as part of the official docs, in docstrings, or even on the web in blog posts, articles, and such.

## Submit Feedback

The best way to send feedback is to file an issue at https://github.com/neurobionics/onshape-robotics-toolkit/issues.

If you are proposing a new feature:

- Explain in detail how it would work.
- Keep the scope as narrow as possible, to make it easier to implement.
- Remember that this is a volunteer-driven project, and that contributions are welcome :)

# Get Started!

Ready to contribute? Here's how to set up `onshape-robotics-toolkit` for local development.
Please note this documentation assumes you already have `uv` and `Git` installed and ready to go.

1. Fork the `onshape-robotics-toolkit` repo on GitHub.

2. Clone your fork locally:

```bash
cd <directory_in_which_repo_should_be_created>
git clone git@github.com:YOUR_NAME/onshape-robotics-toolkit.git
cd onshape-robotics-toolkit
```

3. Install the environment and pre-commit hooks:

**If you are on Windows and do not have GNU Make installed:**

Run the following commands directly in your terminal:

```powershell
uv sync --dev --extra docs
uv run pre-commit install
```

If you also need the simulation extras (MuJoCo, Optuna, etc.), run:

```powershell
uv sync --dev --extra docs --extra simulation
```

**If you have GNU Make installed:**

You can use the shortcut:

```bash
make install
```

This runs the same commands for you automatically.

4. Set up your Onshape API credentials. Create a `.env` file in the project root:

```
ONSHAPE_ACCESS_KEY=...
ONSHAPE_SECRET_KEY=...
```

5. Create a branch for local development:

```bash
git checkout -b name-of-your-bugfix-or-feature
```

Now you can make your changes locally.

6. Don't forget to add test cases for your added functionality to the `tests` directory.

7. When you're done making changes, check that your changes pass all quality checks (linting, type checking, dependency checks):

```bash
make check
```

8. Validate that all unit tests are passing:

```bash
make test
```

9. Commit your changes and push your branch to GitHub:

```bash
git add .
git commit -m "Your detailed description of your changes."
git push origin name-of-your-bugfix-or-feature
```

10. Submit a pull request through the GitHub website.

> **Note:** The CI/CD pipeline runs tests across multiple Python versions (3.10–3.12) automatically on every pull request. You can also run this locally with `tox` if you have multiple Python versions installed, but this is not required before opening a PR.

# Pull Request Guidelines

Before you submit a pull request, check that it meets these guidelines:

1. The pull request should include tests.

2. If the pull request adds functionality, the docs should be updated.
   Put your new functionality into a function with a docstring, and add the feature to the list in `README.md`.
