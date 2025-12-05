---
title: Multimodal LLM
description: "Enable your LLM app to understand any images"
---

Enable your LLM app to understand images with the phospho multimodal model.
For optimal performance, this model is not censored or moderated. Ensuring this model is used in a safe way is your responsability.

# Requirements

Create an account on [phospho.ai](https://platform.phospho.ai) and get your API key.
You need to have setup a billing method. You can add a it in the Settings of your dashboard [here](https://platform.phospho.ai/org/settings/billing).

# Sending a request

To send a request, add:

- `text`: your text prompt. For instance: "What is this?"
- `image_url`: either a URL of the image or the base64 encoded image data.
  The `inputs` list must be of lenght 1.

Optionally, to better control the generation, you can specify the following optional parameters:

- `max_new_tokens` (int): default to 200. Max 250. The maximum number of tokens that can be generated in the response.
- `temperature` (float, between 0.1 and 1.0) Higher values like 0.8 will make the output more random, while lower values like 0.2 will make it more focused and deterministic.
- `repetition_penalty` (float): Default to 1.15. This parameter helps in reducing the repetition of words in the generated content.
- `top_p` (float, between 0.0 and 1.0): Default to 1.0. This parameter controls the diversity of the response by limiting the possible next tokens to the top p percent most likely.

If you pass a URL, make sure it is a generally available image (for instance by passing the link in a Private Navigation window).
To encode an image in base 64, you can use [this website](https://base64.guru/converter/encode/image).

=== "API"

```bash

curl -X 'POST' \
  'https://api.phospho.ai/v2/predict' \
  -H 'accept: application/json' \
  -H 'Authorization: Bearer YOUR_PHOSPHO_API_KEY' \
  -H 'Content-Type: application/json' \
  -d '{
  "inputs": [{"text": "What is this?", "image_url": "http://images.cocodataset.org/val2017/000000039769.jpg"}],
  "model": "phospho-multimodal"
}'
```

=== "Python"

```python
import requests

url = 'https://api.phospho.ai/v2/predict'
headers = {
    'accept': 'application/json',
    'Authorization': 'Bearer YOUR_PHOSPHO_API_KEY',
    'Content-Type': 'application/json'
}
data = {
    "inputs": [{"text": "What is this?", "image_url": "http://images.cocodataset.org/val2017/000000039769.jpg"}],
    "model": "phospho-multimodal"
}

response = requests.post(url, json=data, headers=headers)

print(response.json()['predictions'][0]['description'])

```

!!! note
This API endpoint is for preview and not optimal for production scale serving.
Contact us for on premise deployment or high performance endpoints.

# Pricing

The pricing is based on the number of images sent.

**Note:** You need to have a billing method setup to use the model. Acces your [billing portal](https://platform.phospho.ai/org/settings/billing) to add one.

| Model name           | Price per 100 images | Price per 1000 images |
| -------------------- | -------------------- | --------------------- |
| `phospho-multimodal` | $1                   | $10                   |

!!! info
You are billed in \$1 increment.

    _Example: if you send 150 images, you will be billed \$2._

[Contact us](mailto:contact@phospho.ai) for high volume pricing.
