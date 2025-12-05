---
title: Optimize Jobs
description: You can use the built in optimizer to find the optimal model and hyperparameters for your jobs.
---

In this guide, we will use the `lab`from the `phospho `package to run an event extraction task on a dataset.
First, we will run on a subset of the dataset with several models:

- the OpenAI API
- the Mistral AI API
- a local Ollama model

Then, we will use the `lab` optimizer to find the best model and hyperparameters for the task in term of performance, speed and price.

Finally, we will use the `lab` to run the best model on the full dataset and compare the results with the subset.

Feel free to only use the APIs or Ollama models you want.

## Installation and setup

You will need:

- an OpenAI API key (find yours [here](https://platform.openai.com/api-keys))
- a Mistral AI API key (find yours [here](https://console.mistral.ai/api-keys/))
- Ollama running on your local machine, with the Mistral 7B model installed. You can find the installation instructions for Ollama [here](https://ollama.com)

```
pip install --upgrade phospho
```

### (Optional) Install Ollama

If you want to use Ollama, install the [Ollama app](https://ollama.com) on your desktop, launch it, and install the python package to interact with it:

```
pip install ollama
```

Test your installation by running the following script:

```python
import ollama

try:
  # Let's check we can reach your local Ollama API
  response = ollama.chat(model='mistral', messages=[
    {
      'role': 'user',
      'content': 'What is the best French cheese? Keep your answer short.',
    },
  ])
  print(response['message']['content'])
except Exception as e:
  print(f"Error: {e}")
  print("You need to have a local Ollama server running to continue and the mistral model downloaded. \nRemove references to Ollama otherwise.")
```

## Define the phospho workload and jobs

```python
from phospho import lab
from typing import Literal

# Create a workload in our lab
workload = lab.Workload()

# Setup the configs for our job
# Model are ordered from the least desired to the most desired
class EventConfig(lab.JobConfig):
    event_name: str
    event_description: str
    model_id: Literal["openai:gpt-4", "mistral:mistral-large-latest", "mistral:mistral-small-latest", "ollama:mistral-7B"] = "openai:gpt-4"

# Add our job to the workload
workload.add_job(
    lab.Job(
        name="sync_event_detection",
        id="question_answering",
        config=EventConfig(
            event_name="Question Answering",
            event_description="User asks a question to the assistant",
            model_id="openai:gpt-4"
        )
    )
)
```

# Loading a message dataset

Let's load a dataset of messages from huggingface, so we can run our extraction job on it.

```bash
pip install datasets
```

```python
from datasets import load_dataset

dataset = load_dataset("daily_dialog")

# Generate a sub dataset with 30 messages
sub_dataset = dataset["train"].select(range(30))

# Let's print one of the messages
print(sub_dataset[0]["dialog"][0])

# Build the message list for our lab
messages = []
for row in sub_dataset:
    text = row["dialog"][0]
    messages.append(lab.Message(content=text))

# Run the lab on it
# The job will be run with the default model (openai:gpt-3.5-turbo)
workload_results = await workload.async_run(messages=messages, executor_type="parallel")

# Compute alternative results with the Mistral API and Ollama
await workload.async_run_on_alternative_configurations(messages=messages, executor_type="parallel")
```

### Apply the optimizer to the pipeline

For the purpose of this demo, we consider a considertion good enough if it matches gpt-4 on at least 80% of the dataset. Good old Paretto.

You can check the current configuration of the workload with:

```python
workload.jobs[0].config.model_id
```

To run the optimizer, just run the following:

```python
workload.optimize_jobs(accuracy_threshold=0.8)

# let's check the new model_id (if it has changed)
workload.jobs[0].config.model_id
```

For us, `mistral:mistral-small-latest` was selected.

## Run our workload on the full dataset, with optimized parameters

We can now run the workload on the full dataset, with the optimized model.

```python
sub_dataset = dataset["train"] # Here you can limit the dataset to a subset if you want to test faster and cheaper

# Build the message list for our lab
messages = []
for row in sub_dataset:
    text = row["dialog"][0]
    messages.append(lab.Message(content=text))

# The job will be run with the best model (mistral:mistral-small-latest in our case)
workload_results = await workload.async_run(messages=messages, executor_type="parallel")
```

## Analyze the results

```python
boolean_result = []

# Go through the dict
for key, value in workload_results.items():
    result = value['question_answering'].value
    boolean_result.append(result)

# Let's count the number of True and False
true_count = boolean_result.count(True)
false_count = boolean_result.count(False)

print(f"In the dataset, {true_count/len(boolean_result)*100}% of the messages are a question. The rest are not.")
```

In our case:

```
In the dataset, 44.5% of the messages are a question. The rest are not.
```

## Going further

You can use the `lab` to run other tasks, such as:

- Named Entity Recognition
- Sentiment Analysis
- Evaluations
- And more!

You can also play around with differnet models, different hyperparameters, and different datasets.

You want to have such analysis on your own LLM app, in real time? Check out the cloud hosted version of phospho, available on [phospho.ai](https://phospho.ai)
