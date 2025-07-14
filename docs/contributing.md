# Contributing to `onshape-robotics-toolkit`

Contributions are welcome, and they are greatly appreciated! Every little bit helps :)

---

## Types of Contributions

### Report Bugs

Report bugs [here](https://github.com/neurobionics/onshape-robotics-toolkit/issues)

If you are reporting a bug, please include:

- Your operating system name and version.
- Any details about your local setup that might be helpful in troubleshooting.
- Detailed steps to reproduce the bug.

### Fix Bugs

Look through the GitHub issues for bugs. Anything tagged with "bug" and "help wanted" is open to whoever wants to implement a fix for it.

### Implement Features

Look through the GitHub issues for features. Anything tagged with "enhancement" and "help wanted" is open to whoever wants to implement it.

### Write Documentation

`onshape-robotics-toolkit` could always use more documentation, whether as part of the official docs, in docstrings, or even on the web in blog posts, articles, and such.

### Submit Feedback

The best way to send feedback is to file an issue [here](https://github.com/neurobionics/onshape-robotics-toolkit/issues).

If you are proposing a new feature:

- Explain in detail how it would work.
- Keep the scope as narrow as possible, to make it easier to implement.
- Remember that this is a volunteer-driven project, and that contributions are welcome :)

---

## Get Started!

Ready to contribute? Here's how to set up `onshape-robotics-toolkit` for local development. Please note this documentation assumes you already have `uv` and `Git` installed and ready to go.

Fork the `onshape-robotics-toolkit` repo on GitHub.

Clone your fork locally:

```sh
cd <directory_in_which_repo_should_be_created>
git clone git@github.com:YOUR_NAME/onshape-robotics-toolkit.git
```

Now we need to install the environment. Navigate into the directory

```sh
cd onshape-robotics-toolkit
```

Then, install and activate the environment with:

```sh
uv sync --dev --extra docs
```

Please ensure that uv is installed on your system. If not, you can install it by following the instructions [here](https://docs.astral.sh/uv/getting-started/installation/).

Install pre-commit to run linters/formatters at commit time:

```sh
uv run pre-commit install
```

Create a branch for local development:

```sh
git checkout -b name-of-your-bugfix-or-feature
```

Now you can make your changes locally.

Don't forget to add test cases for your added functionality to the [tests](http://_vscodecontentref_/0) directory.

When you're done making changes, check that your changes pass the formatting tests.

```sh
make check
```

Now, validate that all unit tests are passing:

```sh
make test
```

Before raising a pull request you should also run tox. This will run the tests across different versions of Python:

```sh
tox
```

This requires you to have multiple versions of python installed. This step is also triggered in the CI/CD pipeline, so you could also choose to skip this step locally.

Commit your changes and push your branch to GitHub:

```sh
git add .
git commit -m "Your detailed description of your changes."
git push origin name-of-your-bugfix-or-feature
```

Submit a pull request through the GitHub website.

---

## Pull Request Guidelines

Before you submit a pull request, check that it meets these guidelines:

1. The pull request should include tests.

2. If the pull request adds functionality, the docs should be updated. Put your new functionality into a function with a docstring, and add the feature to the list in [README.md](http://_vscodecontentref_/1).
