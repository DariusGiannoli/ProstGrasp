---
title: Quickstart
description: Run evaluations and detect events in your messages in minutes.
---

Get started with **phospho lab**, the core of phospho. This is what the hosted version of phospho leverages to deliver insights.

![phospho diagram](https://github.com/phospho-app/phospho/raw/dev/phospho_diagram.png)

!!! note
    Looking to setup logging to the phospho hosted version? [Read this guide instead.](/docs/getting-started)

The **phospho lab** is a tool that allows you to run evaluations and detect events in your messages. 

1. Define custom workloads and jobs
2. Run them on your messages in parallel
3. Optimize your models and configurations

## Installation

Install the phospho package with the `lab` extra:

```bash
pip install "phospho[lab]"
```

You need to set your OPENAI_API_KEY as an environment variable.

```bash
export OPENAI_API_KEY=your_openai_api_key
```

If you don't want to use OpenAI, you can setup [Ollama](https://github.com/ollama/ollama) and set the following environment variables:

```bash
export OVERRIDE_WITH_OLLAMA_MODEL=mistral
```

This will replace all calls to OpenAI models with calls to the `mistral` model running with Ollama. Make sure you've downloaded Item.

## Create a workload

The phospho lab lets you run extractions on your messages.

Start by creating a workload. A workload is a set of jobs that you want to run on your messages.

```python
from phospho import lab

# Create the phospho workload
workload = lab.Workload()
```

## Define jobs

Define jobs and add them to the workload. For example, let's add an event detection job. Those are the jobs you can setup in phospho cloud.

```python
# Define the job configurations
class EventConfig(lab.JobConfig):
    event_name: str
    event_description: str

# Let's add an event detection task to our workload
workload.add_job(
            lab.Job(
                id="question_answering",
                job_function=lab.job_library.event_detection,
                config=EventConfig(
                    event_name="question_answering",
                    event_description="The user asks a question to the assistant",
                ),
            )
        )
```

## Run the workload

Now, you can run the workload on your messages. 

Messages are a basic abstraction. They can be user messages or LLM outputs. They can contain metadata or additional information. It's up to the jobs to decide what to do with them.

```python
# Let's add some messages to analyze
message = lab.Message(
                    id="my_message_id",
                    role="User",
                    content="What is the weather today in Paris?",
                )

# Run the workload on the message
# Note that this is an async function. Use asyncio.run to run it in a script.
await workload.async_run(
            messages=[message],
            executor_type="sequential",
        )
```

## Gather results

Results are stored in the workload.

```python
# Check the results of the workload
message_results = workload.results["my_message_id"]

print(f"Result of the event detection: {message_results['question_answering'].value}")
```

You can also get them in a pandas dataframe. 

```python
workload.results_df()
```