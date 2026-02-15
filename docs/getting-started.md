# Getting Started with `onshape-robotics-toolkit`

Welcome to the `onshape-robotics-toolkit` library! This guide will help you set up and start using the library to interact with Onshape's powerful REST API.

The Onshape API allows developers to access, manipulate, and extend Onshape's CAD platform programmatically. The API communicates via HTTP requests, returning data in JSON format.

---

## How Onshape API Works

The Onshape API supports the following HTTP methods:

- **GET**: Retrieve information (e.g., document details, element properties).
- **POST**: Create or update resources (e.g., add features, update parts).
- **DELETE**: Remove resources (e.g., delete configurations).

Each API request typically consists of:

1. **Method**: Defines the action (e.g., GET, POST, DELETE).
2. **URL**: Specifies the endpoint and target resource.
3. **Query Parameters**: Optional key-value pairs to refine the request.
4. **Headers**: Metadata such as content type and authorization tokens.
5. **Payload Body**: Data sent with POST requests.

---

## Understanding API URLs

An Onshape API URL is structured to identify specific documents, workspaces, and elements:

**Example URL:**

```
https://cad.onshape.com/api/documents/e60c4803eaf2ac8be492c18e/w/d2558da712764516cc9fec62/e/6bed6b43463f6a46a37b4a22
```

**Breakdown:**

- **Base URL**: `https://cad.onshape.com/api` – The entry point for API requests.
- **Document ID**: `e60c4803eaf2ac8be492c18e` – The unique identifier for the document.
- **Workspace ID**: `d2558da712764516cc9fec62` – The active workspace within the document.
- **Element ID**: `6bed6b43463f6a46a37b4a22` – A specific element in the workspace (e.g., a part studio or assembly).

---

## Authentication: Secure Your API Calls

Access to the Onshape API requires authentication using API keys. Follow these steps to set up authentication for your project:

### Obtain API Keys

1. Log in to your Onshape account and navigate to the **Developer Portal**.
2. Generate your **Access Key** and **Secret Key**.

### Configure the Library

Create a `.env` file in the root directory of your project to securely store your API keys:

```plaintext
ONSHAPE_ACCESS_KEY = <your_access_key>
ONSHAPE_SECRET_KEY = <your_secret_key>
```

The `onshape-robotics-toolkit` library will automatically read these keys to authenticate your requests.

---

## Install the Library

Install the `onshape-robotics-toolkit` library via pip:

```sh
pip install onshape-robotics-toolkit
```

---

## First API Call: Example Usage

Here's an example of making a simple GET request to list documents using the `onshape-robotics-toolkit` library:

```python
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.models.document import Document

# Initialize the client
client = Client(env=".env")

# Create a Document object from a URL
doc = Document.from_url(
    url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
)

# Retrieve the assembly and its JSON representation
assembly = client.get_assembly(
    did=doc.did,
    wtype=doc.wtype,
    wid=doc.wid,
    eid=doc.eid
)

# Print the assembly details
print(assembly)
```

## What's Next?

- Check out more [examples and tutorials](tutorials/edit.md) in the `onshape-robotics-toolkit` GitHub repository.
- Explore the [Onshape API Documentation](https://onshape-public.github.io/docs/) for detailed API reference.
