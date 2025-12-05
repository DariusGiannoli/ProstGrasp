---
title: Event Fine-tuning
description: "phospho enables you to fine-tune an LLM to detect specific events."
---

<Warning>
  LLM fine-tuning for event detection is in Alpha. Contact us to request access.
</Warning>

## Preparing your data

To fine-tune a model for event detection, you need to prepare a `csv` dataset that contains the following columns:

- `detection_scope` (`Literal`): can only be one of the following values: `task_input_only` or `task_output_only`
- `task_input` (`str`): the input text for a task (uusually the user input)
- `task_output` (`str`): the output text for a task (usually the assistant response)
- `event_description` (`str`): the event description, like the prompt you use to define the event you want to dectect while using phospho
- `label` (`bool`): True if the event is indeed present in the text, False otherwise

A good dataset size is at least 2000 examples.

## Uploading the dataset to phospho

To upload the dataset to phospho, use directly the API. Don't forget to set your API key in the `Authorization` header.

```bash
curl -X 'POST' \
  'https://api.phospho.ai/v2/files' \
  -H 'accept: application/json' \
  -H 'Authorization: Bearer $PHOSPHO_API_KEY' \
  -H 'Content-Type: multipart/form-data' \
  -F 'file=@/path/to/your/local/file.csv.csv;type=text/csv'
```

Keep the `file_id` returned by the API, you will need it to fine-tune the model.

## Launching the fine-tuning

We recomend using the `mistralai/Mistral-7B-Instruct-v0.1` model for event detection.
Once the dataset is uploaded, you can fine-tune the model using the following API call:

```bash
curl -X 'POST' \
  'https://api.phospho.ai/v2/fine_tuning/jobs' \
  -H 'accept: application/json' \
  -H 'Authorization: Bearer $PHOSPHO_API_KEY' \
  -H 'Content-Type: application/json' \
  -d '{
  "file_id": "YOUR_FILE_ID",
  "parameters": {"detection_scope": "YOUR_DETECTION_SCOPE", "event_description": "YOUR EVENT DESCRIPTION HERE"},
  "model": "mistralai/Mistral-7B-Instruct-v0.1"
}'
```

Note the fine-tuning id returned by the API, you will need it to check the status of the job. It should take approximately 20 minutes to complete.

The finetuning job will take some time to complete. You can check the status of the job using the following API call:

```bash
curl -X 'GET' \
  'https://api.phospho.ai/v2/fine_tuning/jobs/FINE_TUNING_JOB_ID' \
  -H 'accept: application/json' \
  -H 'Authorization: Bearer $PHOSPHO_API_KEY'
```

When the fine-tuning job is completed, you can get the fine-tuned model id in the `fine_tuned_model` field of the response.

## Using the fine-tuned model for your event detection

You can now use the fine-tuned model to detect events in your text. To do so, update the configs.

First, get your current project settings:

```bash
curl -X 'GET' \
  'https://api.phospho.ai/v2/projects/YOUR_PROJECT_ID' \
  -H 'accept: application/json' \
  -H 'Authorization: Bearer $PHOSPHO_API_KEY'
```

<Warning>
  The POST request will overwrite the current project settings. Make sure to
  include all the settings you want to keep in the new settings object.
</Warning>

In the settings object, add (or change) the `detection_engine` to the `fine_tuned_model` id you got from the fine-tuning job. Then, update the project settings:

```bash
curl -X 'POST' \
  'https://api.phospho.ai/v2/projects/YOUR_PROJECT_ID' \
  -H 'accept: application/json' \
  -H 'Authorization: Bearer $PHOSPHO_API_KEY' \
  -H 'Content-Type: application/json' \
  -d '{
  "settings": YOUR_UPDATED_SETTINGS_OBJECT
}'
```

You're all set! You can now use the fine-tuned model to detect events in your text.
