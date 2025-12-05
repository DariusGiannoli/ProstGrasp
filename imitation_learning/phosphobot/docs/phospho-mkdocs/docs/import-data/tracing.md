---
title: Log intermediate steps
description: Log all the intermediate steps of your LLM app pipeline
---

To help you debug and deep dive into your LLM apps logs, you can set up tracing using the `phospho` library.

This traces every intermediate steps of your LLM app pipeline, from the input text to the output text.

## Setup 

### Install phospho

!!! info 
    This feature is currently only available for Python. NodeJS version coming soon!


Make sure you have the `phospho` module installed:

```bash
pip install -U phospho
```

### Install OpenTelemetry instrumentations

phospho leverages [OpenTelemetry instrumentations](https://opentelemetry.io/ecosystem/registry/) to trace your LLM app pipeline. To trace a library, you need to install the corresponding instrumentation. 

For example, here is how to  trace OpenAI and Mistral API calls:

```bash
# This will trace OpenAI API calls
pip install opentelemetry-instrumentation-openai
# This will trace MistralAI API calls
pip install opentelemetry-instrumentation-mistralai
```

Refer to this [list of available instrumentations](https://github.com/traceloop/openllmetry/tree/main/packages) to find the one that fits your needs.

### Initialize phospho

Initialize phospho with `phospho.init()` and enable tracing with `tracing=True`:

```python
import phospho

phospho.init(tracing=True)
```

## Automatic tracing

All calls to the installed instrumentations are traced. 

For example, when you do `phospho.log`, the OpenAI API calls will be linked to this log. 

```python
import phospho

phospho.init(tracing=True)

# This is your LLM app code
openai_client = OpenAI()
color = openai_client.chat.completions.create(
    messages=[{"role": "user", "content": "Say a color"}],
    model="gpt-4o-mini"
)
animal = openai_client.chat.completions.create(
    messages=[{"role": "user", "content": "Say an animal"}],
    model="gpt-4o-mini",
)

# This is how you log to phospho
# All the API calls made by the OpenAI client will me linked to this log
phospho.log(
    input="Give me a color and an animal",
    output=f"Color: {color}, Animal: {animal}",
)
```

You can view intermediate steps in the [Phospho dashboard](https://app.phospho.ai/) when reading a message transcript.

In the automatic tracing mode, the link between API calls and logs is done using the timestamps. If you want more control, you can use the context tracing or manual tracing. 

## Context tracing 

To have more control over which instrumentations calls are linked to which logs, define a context using the `phospho.tracer()` context block or `@phospho.trace()` decorator syntax.

### Context block

This links all calls to the instrumentations made inside the context block to the phospho log. For example, this will link the OpenAI API call to the log:

```python
with phospho.tracer(): 
    messages = [{"role": "user", "content": "Say good bye"}]
    openai_client.chat.completions.create(
        messages=messages,
        model="gpt-4o-mini",
        max_tokens=1,
    )
    phospho.log(input="Say good bye", output=response)
```

To add `session_id`, `task_id` and `metadata`, pass them as arguments to the context block:

```python
with phospho.tracer(
    task_id="some_id", 
    session_id="my_session_id", 
    metadata={"user_id": "bob"}
): 
    messages = [{"role": "user", "content": "Say good bye"}]
    openai_client.chat.completions.create(
        messages=messages,
        model="gpt-4o-mini",
        max_tokens=1,
    )
    phospho.log(input="Say good bye", output=response)
```

### Decorator syntax

This works the same way as the context block.

```python
@phospho.trace()
def my_function():
    messages = [{"role": "user", "content": "Say good bye"}]
    openai_client.chat.completions.create(
        messages=messages,
        model="gpt-4o-mini",
        max_tokens=1,
    )
    phospho.log(input="Say good bye", output=response)

my_function()
```

!!! note
    The context is `phospho.tracer`, while the decorator is `phospho.trace`, without the `r`.


To add `session_id`, `task_id` and `metadata`, pass them as arguments to the decorator:

```python
@phospho.trace(
    task_id="some_id", 
    session_id="my_session_id", 
    metadata={"user_id": "bob"}
)
def my_function():
    messages = [{"role": "user", "content": "Say good bye"}]
    openai_client.chat.completions.create(
        messages=messages,
        model="gpt-4o-mini",
        max_tokens=1,
    )
    phospho.log(input="Say good bye", output=response)
```

## Manual tracing

Pass intermediate steps as a `steps` parameter to `phospho.log` to trace your pipeline:

```python
phospho.log(
    input="Give me a color and an animal",
    output=f"Color: {color}, Animal: {animal}",
    steps=[
        {"name": "OpenAI API call", "input": "Say a color", "output": color},
        {"name": "OpenAI API call", "input": "Say an animal", "output": animal},
    ]
)
```

This is useful to trace custom modules, which don't have an Opentelemetry instrumentation available. For example, document retrieval, data augmentation, etc.
