---
title: Intent Embeddings
description: "Generate specific embeddings with phospho"
---

!!! note
This model is in preview. Contact us for production or latency sensitive
specs.

You can generate embeddings for text using the `intent-embed` model. Intent Embed is a mdoel that generates embeddings for text, specifically to represent the user intent. Potential use cases include:

- User Intent classification
- Intent similarity
- Out of topic exclusion
- Intent clustering and analytics
- And more

Read the technical paper here: [Phospho Intent Embeddings](https://research.phospho.ai/phospho_intent_embed.pdf).

# Requirements

Create an account on [phospho.ai](https://platform.phospho.ai) and get your API key.
You need to have setup a billing method. You can add a it in the Settings of your dashboard [here](https://platform.phospho.ai/org/settings/billing).

# Usage

## Using the OpenAI client

The phospho embedding endpoint is OpenAI compatible. You can use the OpenAI client to send requests to the phospho API.

```python

from openai import OpenAI

client = OpenAI(
    api_key="YOUR_PHOSPHO_API_KEY",
    base_url="https://api.phospho.ai/v2",
)

response = client.embeddings.create(
    model="intent-embed",
    input="I want to use the phospho intent embeddings api",
    encoding_format="float",
)

print(response)

```

For now, the input must be a single string. Passing more than one string will result in an error.

## Using the API directly

To send a request, add:

- `text`: The text to embed, usually a user query or message.
- `model`: must be set to `intent-embed`.

Optionally, to link this embedding to one of your projects, you can specify the following optional parameters:

- `project_id`: The project id you want to link this embedding to.

=== "API"

```bash
curl -X 'POST' \
  'https://api.phospho.ai/v2/embeddings' \
  -H 'accept: application/json' \
  -H 'Authorization: Bearer YOUR_PHOSPHO_API_KEY' \
  -H 'Content-Type: application/json' \
  -d '{
  "input": "Your text to embed here",
  "model": "intent-embed"
}'
```

=== "Python"

```
import requests

url = 'https://api.phospho.ai/v2/predict'
headers = {
    'accept': 'application/json',
    'Authorization': 'Bearer YOUR_PHOSPHO_API_KEY',
    'Content-Type': 'application/json'
}
data = {
  "input": "Your text to embed here",
  "model": "intent-embed"
}

response = requests.post(url, json=data, headers=headers)

print(response.json()['embeddings'])

```

You will get a response with the embeddings for the input text. The embeddings are a list of floats.

```json
{
  "object": "list",
  "data": [
    {
      "object": "embedding",
      "embedding": [
        -0.045429688,
        -0.039863896,
        0.0077658836,
        ...],
      "index": 0
    }
  ],
  "model": "intent-embed",
  "usage": {
    "prompt_tokens": 3,
    "total_tokens": 3
  }
}
```

These embeddings can stored in vector databases like Pinecone, Milvus, Chroma, Qdrand, etc. for similarity search, clustering, and other analytics.

# Pricing

The pricing is based on the number of tokens in the input text.

**Note:** You need to have a billing method setup to use the model. Acces your [billing portal](https://platform.phospho.ai/org/settings/billing) to add one.

| Model name     | Price per 1M input tokens |
| -------------- | ------------------------- |
| `intent-embed` | $0.94                     |

!!! info
You are billed in \$1 increment.

[Contact us](mailto:contact@phospho.ai) for high volume pricing.
