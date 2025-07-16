# Error Handling

## Error Handling Flow

```mermaid
graph TD
    A[HTTP Response] --> B{Status Code Check}

    B -->|2xx Success| C[Return Data]
    B -->|Error Status| D[Enhanced Error Handler]

    D --> E{Status Code}

    E -->|400| F["Bad Request (400)"]
    E -->|401| G["Unauthorized (401)"]
    E -->|403| H["Forbidden (403)"]
    E -->|404| I["Not Found (404)"]
    E -->|405| J["Method Not Allowed (405)"]
    E -->|406| K["Not Acceptable (406)"]
    E -->|409| L["Conflict (409)"]
    E -->|415| M["Unsupported Media Type (415)"]
    E -->|429| N["Too Many Requests (429)<br/>Extract Retry-After Header"]
    E -->|499| O["Timeout (499)"]
    E -->|500| P["Internal Server Error (500)"]
    E -->|503| Q["Service Unavailable (503)<br/>Extract Retry-After Header"]
    E -->|307| R["Temporary Redirect (307)"]
    E -->|Other 5xx| S["Server Error (5xx)"]
    E -->|Other| T["HTTP Error (xxx)"]

    N --> U[Include Retry Duration<br/>in Error Message]
    Q --> U

    F --> V[ApiError::Server]
    G --> W[ApiError::Auth]
    H --> W
    I --> V
    J --> V
    K --> V
    L --> V
    M --> V
    U --> V
    O --> V
    P --> V
    R --> V
    S --> V
    T --> V

    V --> X[Python Exception Mapping]
    W --> X

    X --> Y[PyValueError for<br/>Client Issues]
    X --> Z[PyRuntimeError for<br/>Server Issues]
```

## Handled HTTP Status Codes

| Code | Description            | Error Type       | Special Handling            |
| ---- | ---------------------- | ---------------- | --------------------------- |
| 400  | Bad Request            | ApiError::Server | Descriptive message         |
| 401  | Unauthorized           | ApiError::Auth   | Auth-specific handling      |
| 403  | Forbidden              | ApiError::Auth   | Auth-specific handling      |
| 404  | Not Found              | ApiError::Server | Document/resource not found |
| 405  | Method Not Allowed     | ApiError::Server | Operation not supported     |
| 406  | Not Acceptable         | ApiError::Server | Media type issues           |
| 409  | Conflict               | ApiError::Server | Duplicate values            |
| 415  | Unsupported Media Type | ApiError::Server | Invalid data types          |
| 429  | Too Many Requests      | ApiError::Server | Includes retry duration     |
| 499  | Timeout                | ApiError::Server | Request timeout             |
| 500  | Internal Server Error  | ApiError::Server | Server issues               |
| 503  | Service Unavailable    | ApiError::Server | Includes retry duration     |
| 307  | Temporary Redirect     | ApiError::Server | Redirect handling           |
