---
title: Log to phospho with Javascript
description: "Collect interactions and tasks"
---

## Log tasks to phospho

**Tasks are the basic bricks that make up your LLM apps.** If you're a programmer, you can think of tasks like functions.

A task is made of at least two things:

- `input (string)`: What goes into a task. Eg: what the user asks to the assistant.
- `output (string?)`: What goes out of the task. Eg: what the assistant replied to the user.

Example of tasks you can log to phospho:

- Call to an LLM (input = query, output = llm response)
- Answering a question (input = question, output = answer)
- Searching in documents (input = search query, output = document)
- Summarizing a text (input = text, output = summary)
- Performing inference of a model (input = X, output = y)

## Install the phospho module

The phospho [JavaScript module](https://www.npmjs.com/package/phospho) is the easiest way to log to phospho. It is compatible with Node.js.

Types are available for your Typescript codebase.

```bash
npm i phospho
# with yarn
yarn add phospho
```

!!! info
    The phospho module is an open source work in progress. [Your help is deeply
    appreciated!](https://github.com/phospho-app/phosphojs)

## Initialize phospho

In your app, initialize the phospho module. By default, phospho will look for `PHOSPHO_API_KEY` and `PHOSPHO_PROJECT_ID` environment variables.

!!! tip
    Learn how to get your api key and project id by [clicking
    here!](getting-started)

```javascript
import { phospho } from "phospho";

phospho.init();
```

You can also pass the `api_key` and `project_id` parameters to `phospho.init`.

```javascript
// Initialize phospho
phospho.init({ apiKey: "api_key", projectId: "project_id" });
```

## Log with phospho.log

The most minimal way to log a task is to use `phospho.log`.

### Logging text inputs and outputs

```javascript
const question = "What's the capital of Fashion?";

const myAgent = (query) => {
  // Here, you'd do complex stuff.
  // But for this example we'll just return the same answer every time.
  return "It's Paris of course.";
};

// Log events to phospho by passing strings directly
phospho.log({
  input: question,
  output: myAgent(question),
});
```

Note that the output is optional. If you don't pass an output, phospho will log `null`.

### Logging OpenAI queries and responses

phospho aims to be battery included. So if you pass something else than a `string` to `phospho.log`, phospho extracts what's usually considered "the input" or "the output".

For example, if you use the OpenAI API:

```javascript
// If you pass full OpenAI queries and results to phospho, it will extract the input and output for you.
const question = "What's the capital of Fashion?";
const query = {
  model: "gpt-3.5-turbo",
  temperature: 0,
  seed: 123,
  messages: [
    {
      role: "system",
      content:
        "You are an helpful frog who gives life advice to people. You say *ribbit* at the end of each sentence and make other frog noises in between. You answer shortly in less than 50 words.",
    },
    {
      role: "user",
      content: question,
    },
  ],
  stream: false,
};
const result = openai.chat.completions.create(query);
const loggedContent = await phospho.log({ input: query, output: result });

// Look at the fields "input" and "output" in the logged content
// Original fields are in "raw_input" and "raw_output"
console.log("The following content was logged to phospho:", loggedContent);
```

### Custom extractors

Pass custom extractors to `phospho.log` to extract the input and output from any object. The original object will be converted to a dict (if jsonable) or a string and stored in `raw_input` and `raw_output`.

```javascript
phospho.log({
  input: { custom_input: "this is a complex object" },
  output: { custom_output: "which is not a string nor a standard object" },
  // Custom extractors return a string
  inputToStrFunction: (x) => x.custom_input,
  outputToStrFunction: (x) => x.custom_output,
});
```

## Logging additional metadata

You can log additional data with each interaction (user id, version id,...) by passing arguments to `phospho.log`.

```javascript
log = phospho.log({
  input: "log this",
  output: "and that",
  // There is a metadata field
  metadata: { always: "moooore" },
  // Every extra keyword argument is logged as metadata
  log_anything_and_everything: "even this is ok",
});
```

## Streaming

phospho supports streamed outputs. This is useful when you want to log the output of a streaming API.

### Example with phospho.log

Pass `stream: true` to `phospho.log` to handle streaming responses. When iterating over the response, phospho will automatically log each chunk until the iteration is completed.

For example, you can pass streaming OpenAI responses to `phospho.log` the following way:

```javascript
// This should also work with streaming
const question = "What's the capital of Fashion?";
const query = {
  model: "gpt-3.5-turbo",
  temperature: 0,
  seed: 123,
  messages: [
    {
      role: "system",
      content:
        "You are an helpful frog who gives life advice to people. You say *ribbit* at the end of each sentence and make other frog noises in between. You answer shortly in less than 50 words.",
    },
    {
      role: "user",
      content: question,
    },
  ],
  stream: true,
};
const streamedResult = await openai.chat.completions.create(query);

phospho.log({ input: query, output: streamedResult, stream: true });

for await (const chunk of streamedResult) {
  process.stdout.write(chunk.choices[0]?.delta?.content || "");
}
```
