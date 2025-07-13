# syntax=docker/dockerfile:1

FROM python:3.10-slim-buster

# Install uv
RUN pip install uv

# Copy only requirements to cache them in docker layer
WORKDIR /code
COPY uv.lock pyproject.toml /code/

# Project initialization:
RUN uv sync --no-dev

# Copy Python code to the Docker image
COPY onshape_robotics_toolkit /code/onshape_robotics_toolkit/

CMD [ "python", "onshape_robotics_toolkit/foo.py"]
