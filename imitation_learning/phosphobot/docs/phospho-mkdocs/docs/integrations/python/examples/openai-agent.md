---
title: OpenAI CLI agent
---

# OpenAI agent

This is an example of a minimal OpenAI assistant in the console. Every interaction is logged to phospho.

It demonstrates how to use `phospho.wrap()` with streaming content.

## Installation

```
pip install --upgrade phospho openai
```

## Setup 

Create a `.env` file:
```
PHOSPHO_PROJECT_ID=...
PHOSPHO_API_KEY=...
OPENAI_API_KEY=...
```

If you don't have a phospho API key and project ID,  go to [Getting Started](/docs/getting-started) for the step by step instructions.

## Implementation

In `assistant.py`, add the following code:

```python

import phospho
import openai

from dotenv import load_dotenv

load_dotenv()

phospho.init()
openai_client = openai.OpenAI()

messages = []

print("Ask GPT anything (Ctrl+C to quit)", end="")

while True:
    prompt = input("\n>")
    messages.append({"role": "user", "content": prompt})

    query = {
        "messages": messages,
        "model": "gpt-3.5-turbo",
        "stream": True,
    }
    response = openai_client.chat.completions.create(**query)

    phospho.log(input=query, output=response, stream=True)

    print("\nAssistant: ", end="")
    for r in response:
        text = r.choices[0].delta.content
        if text is not None:
            print(text, end="", flush=True)
```


Launch the script and chat with the agent. 

```
python assistant.py
```

Go to the phospho dashboard to monitor the interactions. 