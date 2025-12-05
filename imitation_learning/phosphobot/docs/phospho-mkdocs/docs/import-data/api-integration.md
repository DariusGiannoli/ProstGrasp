---
title: "Setup logging in your app"
description: "Log text messages to phospho in real time"
---

You can setup the logging to phospho in your app in a few minutes.

## 1. Get your phospho API key and your project id

Go to the [phospho platform](https://platform.phospho.ai/). Login or create an account if you don't have one.

If this is your first time using phospho, a Default project has been created for you. On the main page, note down the **project id** and follow the link to create a new **API key**.

If you already have a project, go to Settings. Your project id is displayed on the top of the page. To create an API key, click on the _Manage Organization & API keys_ button. Store your **API key** safely!

## 2. Setup phospho logging in your app

### Add environment variables

In your code, add the following environment variables:

```bash
export PHOSPHO_API_KEY="your_api_key"
export PHOSPHO_PROJECT_ID="your_project_id"
```

### Log to phospho

The basic abstraction of phospho is the **task**. If you're a programmer, you can think of tasks like a function.

- `input (str)`: The text that goes into the system. _Eg: the user message._
- `output (Optional[str])`: The text that comes out of the system. _Eg: the system response._

We prefer to use this abstraction because of its flexibility. You can log any text to a task, not just chat messages: _call to an LLM, answering a question, searching in documents, summarizing a text, performing inference of a model, steps of a chain-of-thought..._

Tasks can be grouped into **sessions**. Tasks and Sessions can be attached to **users**.

### How to setup logging?


=== "Python"

    The phospho [Python module](https://pypi.org/project/phospho/) in the easiest way to log to phospho. It is compatible with Python 3.9+.

    ```bash
    pip install --upgrade phospho
    ```

    To log tasks, use `phospho.log`. The logged tasks are analyzed by the phospho analytics pipeline.

    ```python
    import phospho

    # By default, phospho reads the PHOSPHO_API_KEY and PHOSPHO_PROJECT_ID from the environment variables
    phospho.init()

    # Example
    input = "Hello! This is what the user asked to the system"
    output = "This is the response showed to the user by the app."

    # This is how you log a task to phospho
    phospho.log(
      input=input,
      output=output,
      # Optional: for chats, group tasks together in sessions
      # session_id = "session_1",
      # Optional: attach tasks to users
      # user_id = "user_1",
      # Optional: add metadata to the task
      # metadata = {"system_prompt": "You are a helpful assistant."},
    )
    ```

    <div class="grid cards" markdown>

    -   :material-language-python:{ .lg .middle } __More about logging in Python__

        ---

        Did you know you could log OpenAI completions, streaming outputs and metadata? Learn more by clicking here.

        [:octicons-arrow-right-24: Read more](#)

    </div>

=== "Javascript"

    The phospho [JavaScript module](https://www.npmjs.com/package/phospho) is the easiest way to log to phospho. It is compatible with Node.js.

    Types are available for your Typescript codebase.

    ```bash
    npm i phospho
    ```

    To log tasks, use `phospho.log`. The logged tasks are analyzed by the phospho analytics pipeline.

    ```js
    import { phospho } from "phospho";

    // By default, phospho reads the PHOSPHO_API_ID and PHOSPHO_PROJECT_KEY from the environment variables
    phospho.init();

    // Example
    const input = "Hello! This is what the user asked to the system";
    const output = "This is the response showed to the user by the app.";

    // This is how you log a task to phospho
    phospho.log({
      input,
      output,
      // Optional: for chats, group tasks together in sessions
      // session_id: "session_1",
      // Optional: attach tasks to users
      // user_id: "user_1",
      // Optional: add metadata to the task
      // metadata: { system_prompt: "You are a helpful assistant." },
    });
    ```

    <div class="grid cards" markdown>

    -   :material-language-javascript:{ .lg .middle } __More about logging in Javascript__

        ---

        Did you know you could log OpenAI completions, streaming outputs and metadata? Learn more by clicking here.

        [:octicons-arrow-right-24: Read more](#)

    </div>


=== "API"

    You can directly log to phospho using [the /log endpoint](https://api.phospho.ai/v2/redoc#tag/Logs) of the API.

    ```bash
    curl -X POST https://api.phospho.ai/v2/log/$PHOSPHO_PROJECT_ID \
    -H "Authorization: Bearer $PHOSPHO_API_KEY" \
    -H "Content-Type: application/json" \
    -d '{
        "batched_log_events": [
            {
                "input": "your_input",
                "output": "your_output",
                "session_id": "session_1",
                "user_id": "user_1",
                "metadata": {"system_prompt": "You are a helpful assistant."},
            }
        ]
    }'
    ```

    !!! info 
        The `session_id`, `user_id` and `metadata` fields are **optional.**

    <div class="grid cards" markdown>

    -   :material-webhook:{ .lg .middle } __API reference__

        ---

        Create a tailored integration with the API. Learn more by clicking here.

        [:octicons-arrow-right-24: Read more](#)

    </div>


=== "Langchain"

    We provide a Langchain callback in our [Python module](https://pypi.org/project/phospho/).

    ```bash
    pip install --upgrade phospho
    ```

    ```python
    from phospho.integrations import PhosphoLangchainCallbackHandler

    chain = ... # Your Langchain agent or chain

    chain.invoke(
        "Your chain input",
        # Add the callback handler to the config
        config={"callbacks": [PhosphoLangchainCallbackHandler()]},
    )
    ```

    <div class="grid cards" markdown>

    -   :material-bird:{ .lg .middle } __Langchain guide__

        ---

        Customize what is logged to phospho by customizing the callback. Learn more by clicking here.

        [:octicons-arrow-right-24: Read more](#)

    </div>


=== "Supabase"

    Integrate phospho to your Supabase app is as simple as using the **phospho API**.

    !!! note
        Follow the Supabase guide to leverage the power of product analytics in your
        Supabase app!


    <div class="grid cards" markdown>

    -   :material-lightning-bolt:{ .lg .middle } __Read the supabase guide__

        ---

        Get started with Supabase and phospho. Learn more by clicking here.

        [:octicons-arrow-right-24: Read more](#)

    </div>



## 3. Get insights in the dashboard

phospho run analytics pipelines on the messages logged. Discover the insights in the [phospho dashboard](https://phospho.ai/).

## Next steps

<div class="grid cards" markdown>

-   :material-tag:{ .lg .middle } __Automatic tagging__

    ---

    Automatically annotate your text data and be alerted. **Take action.**

    [:octicons-arrow-right-24: Learn more](#)

-   :material-account-group:{ .lg .middle } __Unsupervised clustering__

    ---

    Group users' messages based on their intention. **Find out what your users are talking about.**

    [:octicons-arrow-right-24: Learn more](#)

-   :material-ab-testing:{ .lg .middle } __AB Testing__

    ---

    Run experiments and iterate on your LLM app, while keeping track of performances. **Keep shipping.**

    [:octicons-arrow-right-24: Learn more](#)

-   :material-valve:{ .lg .middle } __Flexible evaluation pipeline__

    ---

    Discover how to run and design a text analytics pipeline using natural language. **No code needed.**

    [:octicons-arrow-right-24: Learn more](#)

-   :material-account-search:{ .lg .middle } __User analytics__

    ---

    Detect user languages, sentiment, and more. **Get to know power users.**

    [:octicons-arrow-right-24: Learn more](#)

</div>
