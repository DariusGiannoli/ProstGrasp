---
title: Tak API
description: "Call Tak through API"
---

!!! note
    Access to this feature is restricted. Contact us at contact@phospho.ai to
    request access.

Please note that the version available via API is different from the one available online at [tak.phospho.ai](https://tak.phospho.ai).

To access the API, you need to have a phospho API key and a project on the phospho platform. You can get one by signing up on [phospho.ai](https://platform.phospho.ai).

The tak API endpoint is OpenAI compatible. You can use the OpenAI client to send requests to the tak API. Messages sent will appear in your phospho dashboard.

Available models:

- `tak-large`: leverages GPT-4o, can search the web and the news.

## Capabilities

Tak can search the web and the news to provide up to date information on a wide range of topics.
It can also perform standard LLM tasks such as summarization, translation, and question answering.
Answers are formated in markdown and contains the sources of the information (link in Markdown format).

Tak can handle tasks requiring multiple web searches in a single query, such as: `What is NVIDIA current stock price? And what is Apple stock price?`

Streaming is supported.

## Limits

The default rate limit is 500 requests per minute. The maximum context window is 128k tokens.

## Sending requests

To send requests, you need to:

- set the base_url to `https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/` (instead of `https://api.openai.com/v1/`)
- set the OPENAI_API_KEY to your phospho API key
- set the model to the desired model to `phospho:tak-large`
- no need to specify a `system` message. If you add one, it won't be followed.

=== "OpenAI Python SDK"

    ```python
    import openai

    from openai import OpenAI
    client = OpenAI(api_key="PHOSPHO_API_KEY", base_url="https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/")

    completion = client.chat.completions.create(
      model="phospho:tak-large",
      messages=[
        {"role": "user", "content": "What are the latest AI news in France?"}
      ]
    )

    print(completion.choices[0].message)

    # Or with streaming

    response = client.chat.completions.create(
        model='phospho:tak-large',
        messages=[
            {'role': 'user', 'content': "Count to 10"}
        ],
        temperature=0,
        stream=True  # this time, we set stream=True
    )

    for chunk in response:
        print(chunk.choices[0].delta.content, end="", flush=True)

    ```

=== "API"

    ```bash
    curl https://api.phospho.ai/v2/{YOUR_PHOSPHO_PROJECT_ID}/chat/completions \
      -H "Content-Type: application/json" \
      -H "Authorization: Bearer $PHOSPHO_API_KEY" \
      -d '{
        "model": "openai:gpt-4o",
        "messages": [
          {
            "role": "user",
            "content": "What are the latest AI news in France?"
          }
        ]
      }'
    ```

=== "JavaScript OpenAI SDK"

    ```javascript openai javascript sdk
    // Same as for the python SDK
    ```

# Pricing

The pricing is based on the number of tokens in input messages and output completion.

**Note:** You need to have a billing method setup to use the model. Acces your [billing portal](https://platform.phospho.ai/org/settings/billing) to add one.

| Model name  | Price per 1M input tokens | Price per 1M output tokens |
| ----------- | ------------------------- | -------------------------- |
| `tak-large` | $5                        | $20                        |
