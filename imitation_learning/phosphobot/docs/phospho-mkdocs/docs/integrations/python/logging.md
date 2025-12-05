---
title: Log to phospho with Python
description: "Collect interactions and tasks"
---

## Log tasks to phospho

phospho is a text analytics tool. To send text, you need to **log tasks**.

### What's a task in phospho?

**Tasks are the basic bricks that make up your LLM apps.** If you're a programmer, you can think of tasks like functions.

A task is made of at least two things:

- `input (str)`: What goes into a task. Eg: what the user asks to the assistant.
- `output (Optional[str])`: What goes out of the task. Eg: what the assistant replied to the user.

The Task abstraction helps you structure your app and quickly explain what it does to an outsider: "Here's what goes in, here's what goes out."

It's the basic unit of text analytics. You can analyze the input and output of a task to understand the user's intent, the system's performance, or the quality of the response.

### Examples of tasks

- Call to an LLM (input = query, output = llm response)
- Answering a question (input = question, output = answer)
- Searching in documents (input = search query, output = document)
- Summarizing a text (input = text, output = summary)
- Performing inference of a model (input = X, output = y)

## How to log a task?

### Install phospho module

The phospho [Python module](https://pypi.org/project/phospho/) in the easiest way to log to phospho. It is compatible with Python 3.9+.

```bash
pip install --upgrade phospho
```

<Info>
  The phospho module is open source. [Feel free to contribute!](https://github.com/phospho-app/phospho)
</Info>

### Initialize phospho

In your app, initialize the phospho module. By default, phospho will look for `PHOSPHO_API_KEY` and `PHOSPHO_PROJECT_ID` environment variables.

!!! tip
    Learn how to get your api key and project id by [clicking
    here!](/docs/getting-started)

```python
import phospho

phospho.init()
```

You can also pass the `api_key` and `project_id` parameters to `phospho.init`.

```python
phospho.init(api_key="phospho-key", project_id="phospho-project-id")
```

### Log with `phospho.log`

To log messages to phospho, use `phospho.log`. This function logs a task to phospho. A task is a pair of input and output strings. The output is optional.


phospho is a text analytics tool. You can log any string input and output this way:

```python
input_text = "Hello! This is what the user asked to the system"
output_text = "This is the response showed to the user by the app."

# This is how you log a task to phospho
phospho.log(input=input_text, output=output_text)
```

The output is optional.

The input and output logged to phospho are displayed in the dashboard and used to perform text analytics.

## Common use cases

### Log OpenAI queries and responses

phospho aims to be battery included. So if you pass something else than a `str` to `phospho.log`, phospho extracts what's usually considered "the input" or "the output".

For example, you can pass to `phospho.log` the same `input` as the arguments for `openai.chat.completions.create`. And you can pass to `phospho.log` the same `output` as OpenAI's `ChatCompletion` objects.

```python
import openai
import phospho

phospho.init()
openai_client = openai.OpenAI(api_key="openai-key")

input_prompt = "Explain quantum computers in less than 20 words."

# This is your LLM app code
query = {
    "messages": [{"role": "system", "content": "You are a helpful assistant."},
                 {"role": "user", "content": input_prompt},
    ],
    "model": "gpt-4o-mini",
}
response = openai_client.chat.completions.create(**query)

# You can directly pass as dict or a ChatCompletion as input and output
log = phospho.log(input=query, output=response)
print("input:", log["input"])
print("output:", log["output"])
```

```text
input: Explain quantum computers in less than 20 words.
output: Qubits harness quantum physics for faster, more powerful computation.
```

Note that the input is a dict. 

### Log a list of OpenAI messages 

In conversational apps, your conversation history is often a list of messages with a `role` and a `content`. This is because it's the format expected by OpenAI's chat API.

You can directly log this messages list as an input or an output to `phospho.log`. The input, output, and system prompt are automatically extracted based on the messages' role.

```python
# This is your conversation history in a chat app
messages = [
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "Explain quantum computers in less than 20 words."},
]

# Your LLM app code generates a response
response = openai_client.chat.completions.create(
    messages=messages,
    model="gpt-4o-mini",
)

# You append the response to the conversation history
messages.append({"role": response.choices[0].role, "content": response.choices[0].message.content, } )

# You can log the conversation history as input or output
log = phospho.log(input=messages, output=messages)

print("input:", log["input"])
print("output:", log["output"])
print("system_prompt:", log["system_prompt"]) # system prompt is automatically extracted
```

```text
input: Explain quantum computers in less than 20 words.
output: Qubits harness quantum physics for faster, more powerful computation.
system_prompt: You are a helpful assistant.
```

Note that consecutive messages with the same role are **concatenated** with a newline.

```python
messages = [
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "Explain quantum computers in less than 20 words."},
    {"role": "user", "content": "What is the speed of light?"},
]
log = phospho.log(input=messages)
```

```text
input: Explain quantum computers in less than 20 words.\nWhat is the speed of light?
```

If you need more control, consider using custom extractors.

### Custom extractors

Pass custom extractors to `phospho.log` to extract the input and output from any object. The custom extractor is a function that is applied to the input or output before logging. The function should return a string.

The original object is converted to a dict (if jsonable) or a string, and stored in `raw_input` and `raw_output`.

```python
phospho.log(
    input={"custom_input": "this is a complex object"},
    output={"custom_output": "which is not a string nor a standard object"},
    # Custom extractors return a string
    input_to_str_function=lambda x: x["custom_input"],
    output_to_str_fucntion=lambda x: x["custom_output"],
)
```

```text
input: this is a complex object
output: which is not a string nor a standard object
```

## Log metadata

You can log additional data with each interaction (user id, version id,...) by passing arguments to `phospho.log`.

```python
log = phospho.log(
    input="log this",
    output="and that",
    # There is a metadata field
    metadata={"always": "moooore"},
    # Every extra keyword argument is logged as metadata
    log_anything_and_everything="even this is ok",
)
```

## Log streaming outputs

phospho supports streamed outputs. This is useful when you want to log the output of a streaming API.

### Example: OpenAI streaming

Out of the box, phospho supports streaming OpenAI completions. Pass `stream=True` to `phospho.log` to handle streaming responses. 

When iterating over the response, phospho will automatically concatenate each chunk until the streaming is finished.

```python

from openai.types.chat import ChatCompletionChunk
from openai._streaming import Stream

query = {
    "messages": [{"role": "system", "content": "You are a helpful assistant."},
                 {"role": "user", "content": "Explain quantum computers in less than 20 words."},
    ],
    "model": "gpt-4o-mini",
    # Enable streaming on OpenAI
    "stream": True
}
# OpenAI completion function return a Stream of chunks
response: Stream[ChatCompletionChunk] = openai_client.chat.completions.create(**query)

# Pass stream=True to phospho.log to handle this
phospho.log(input=query, output=response, stream=True)
```

### Example: Local Ollama streaming

Let's assume you're in a setup where you stream text from an API. The stream is a [generator](https://realpython.com/introduction-to-python-generators/) that yields chunks of the response. The generator is [immutable](https://realpython.com/python-mutable-vs-immutable-types/) by default.

To use this as an `output` in `phospho.log`, you need to: 

1. Wrap the generator with `phospho.MutableGenerator` or `phospho.MutableAsyncGenerator` (for async generators) 
2. Specify a `stop` function that returns `True` when the streaming is finished. This is used to trigger the logging of the task.

Here is an example with an [Ollama endpoint](https://ollama.com) that streams responses.

```python
r = requests.post(
    # This is a local streaming Ollama endpoint
    "http://localhost:11434/api/generate",
    json={
        "model": "mistral-7b",
        "prompt": "Explain quantum computers in less than 20 words.",
        "context": [],
    },
    # This connects to a streaming API endpoint
    stream=True,
)
r.raise_for_status()
response_iterator = r.iter_lines()

# response_iterator is a generator that streams the response token by token
# It is immutable by default
# In order to directly log this to phospho, we need to wrap it this way
response_iterator = phospho.MutableGenerator(
    generator=response_iterator,
    # Indicate when the streaming stops
    stop=lambda line: json.loads(line).get("done", False),
)

# Log the generated content to phospho with Stream=True
phospho.log(input=prompt, output=response_iterator, stream=True)

# As you iterate over the response, phospho combines the chunks
# When stop(output) is True, the iteration is completed and the task is logged
for line in response_iterator:
    print(line)
```

## Wrap functions with `phospho.wrap`

If you wrap a function with `phospho.wrap`, phospho automatically logs a task when they are called:

- The passed arguments are logged as `input`
- The returned value is logged as `output`

You can still use [custom extractors](#custom-extractors) and log metadata.

### Use the `@phospho.wrap` decorator

If you want to log every call to a python function, you can use the `@phospho.wrap` decorator. This is a nice pythonic way to structure your LLM app's code.

```python
@phospho.wrap
def answer(messages: List[Dict[str, str]]) -> Optional[str]:
    response = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
    )
    return response.choices[0].delta.content
```

### How to log metadata with phospho.wrap?

Like phospho.log, every extra keyword argument is logged as metadata.

```python
@phospho.wrap(metadata={"more": "details"})
def answer(messages: List[Dict[str, str]]) -> Optional[str]:
    response = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
    )
    return response.choices[0].delta.content
```

### Wrap an imported function with phospho.wrap

If you can't change the function definition, you can wrap it this way:

```python
# You can wrap any function call in phospho.wrap
response = phospho.wrap(
    openai_client.chat.completions.create,
    # Pass additional metadata
    metadata={"more": "details"},
)(
    messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": "Explain quantum computers in less than 20 words."},
    ],
    model="gpt-4o-mini",
)
```

If you want to wrap all calls to a function, override the function definition with the wrapped version:

```python
openai_client.chat.completions.create = phospho.wrap(
    openai_client.chat.completions.create
)
```

### Wrap a streaming function with phospho.wrap

phospho.wrap can handle streaming functions. To do that, you need two things:

1. Pass `stream=True`. This tells phospho to concatenate the string outputs.
2. Pass a `stop` function, such that `stop(output) is True` when the streaming is finished and trigger the logging of the task.

```python
@phospho.wrap(stream=True, stop=lambda token: token is None)
def answer(messages: List[Dict[str, str]]) -> Generator[Optional[str], Any, None]:
    streaming_response: Stream[
        ChatCompletionChunk
    ] = openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=messages,
        stream=True,
    )
    for response in streaming_response:
        yield response.choices[0].delta.content
```