---
title: LLMs
description: "Call LLMs through the phospho proxy"
---

!!! note
    Access to this feature is restricted. Contact us at contact@phospho.ai to
    request access.

To access any model through the phospho proxy, you need to have a phospho API key and a project on the phospho platform. You can get one by signing up on [phospho.ai](https://platform.phospho.ai).

To access the Tak API, please refer to the [Tak API page](/docs/models/tak).

# OpenAI

The phospho proxy is OpenAI compatible. You can use the OpenAI client to send requests to the phospho API. Messages sent through the phospho proxy will appear in your phospho dashboard.

Available models:

- `gpt-4o`
- `gpt-4o-mini`

To access these models through the phospho proxy, you need to:

- set the base_url to `https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/` (instead of `https://api.openai.com/v1/`)
- set the OPENAI_API_KEY to your phospho API key
- set the model to the desired model with the prefix `openai:` ( e.g. `openai:gpt-4o` or `openai:gpt-4o-mini`)

=== "Open AI Python"

    ```python openai python sdk
    import openai

    from openai import OpenAI
    client = OpenAI(api_key="PHOSPHO_API_KEY", base_url="https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/")

    completion = client.chat.completions.create(
      model="openai:gpt-4o",
      messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": "Hello!"}
      ]
    )

    print(completion.choices[0].message)

    ```

=== "API"

    ```bash curl
    curl https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/chat/completions \
      -H "Content-Type: application/json" \
      -H "Authorization: Bearer $PHOSPHO_API_KEY" \
      -d '{
        "model": "openai:gpt-4o",
        "messages": [
          {
            "role": "system",
            "content": "You are a helpful assistant."
          },
          {
            "role": "user",
            "content": "Hello!"
          }
        ]
      }'
    ```

    ```javascript openai javascript sdk
    // Same as for the python SDK
    ```

# Mistral ai

The phospho proxy is Mistral ai compatible. You can use the Mistral client to send requests to the phospho API. Messages sent through the phospho proxy will appear in your phospho dashboard.

Available models:

- `mistral-small-latest`
- `mistral-small-latest`

To access these models through the phospho proxy, you need to:

- set the server_url to `https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/`
- set the MISTRAL_API_KEY to your phospho API key
- set the model to the desired model with the prefix `mistral:` ( e.g. `mistral:mistral-large-latest` or `mistral:mistral-small-latest`)

=== "Mistral ai Python"

```python
import mistralai

from mistralai import Mistral
client = Mistral(api_key="PHOSPHO_API_KEY", server_url="https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/")

completion = client.chat.complete(
  model="mistral:mistral-large-latest",
  messages=[
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "Hello!"}
  ]
)

print(completion.choices[0].message)

```

=== "API"

```bash
curl https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/v1/chat/completions \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $PHOSPHO_API_KEY" \
  -d '{
    "model": "mistral:mistral-large-latest",
    "messages": [
      {
        "role": "system",
        "content": "You are a helpful assistant."
      },
      {
        "role": "user",
        "content": "Hello!"
      }
    ]
  }'
```

```javascript mistralai javascript sdk
// Same as for the python SDK
```


# Anthropic

Docs coming soon.
