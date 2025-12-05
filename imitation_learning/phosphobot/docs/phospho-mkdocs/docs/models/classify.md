---
title: Classification
description: "Train, use and download classification models"
---

<Note>
  Request access to the preview by contacting us at contact@phospho.ai
</Note>

phospho can handle all the data processing, data engineering and model training for you.
For now, only binary classification models are supported (learn [here](https://en.wikipedia.org/wiki/Binary_classification) what binary classification is).

# Why train your custom classification model?

Most LLM chains involve classification steps where the LLM is prompted with a classification task.
Training your own classification model can help you to:

- improve the accuracy of the classification
- reduce the latency of the classification (as you have the model running in the application code)
- reduce the cost of the classification (as you don't have to call an external LLM API)
- reduce risks of downtime (as you don't depend on an external LLM API)

# Available models

`phospho-small` is a small text classification model that can be trained with a few examples (minimum 20 examples).
It runs on CPU and once trained using phospho, you can download your trained model from Hugging Face.

# Train a model on your data

To train a model, you need to provide a list of examples for the modelat least 20 examples containing text, labels and a label description.
Each example should have the following fields:

- `text` (str): the text to classify (for example, a user message)
- `label` (bool): True or False according to the classification
- `label_text` (str): a few word description of the label when true (for example, "user asking for pricing")

For example, your examples could look like this:

```json
[
    {
      "text": "Can I have a discount on phospho pro?",
      "label": true,
      "label_text": "user asking for pricing"
    },
    {
      "text": "I want to know more about phospho pro",
      "label": false,
      "label_text": "user asking for pricing"
    },
    ...
  ]
```

Start the training using the following API call or python code snippet:

=== "API"

    ```bash HTTP API

    curl -X 'POST' \
      'https://api.phospho.ai/v2/train' \
      -H 'accept: application/json' \
      -H 'Authorization: Bearer $PHOSPHO_API_KEY' \
      -H 'Content-Type: application/json' \
      -d '{
      "model": "phospho-small",
      "examples": [
        {
          "text": "How much is phospho pro?",
          "label": true,
          "label_text": "user asking for pricing"
        },
        {
          "text": "I want to know more about phospho pro",
          "label": false,
          "label_text": "user asking for pricing"
        },
        ...
      ],
      "task_type": "binary-classification"
    }'
    ```

=== "Python"

    ```python
    import phospho

    phospho.init()

    my_examples = [
        {
          "text": "How much is phospho pro?",
          "label": True,
          "label_text": "user asking for pricing"
        },
        {
          "text": "I want to know more about phospho pro",
          "label": False,
          "label_text": "user asking for pricing"
        },
        ...
      ]

    model = phospho.train("phospho-small", my_examples)

    print(model)
    ```

You will get a model object in the response. You will need the `model_id` to use the model. It should look like this: `phospho-small-8963ba3`.

```json
{
  "id": "YOUR_MODEL_ID",
  "created_at": 1714418246,
  "status": "training",
  "owned_by": "YOUR_ORG_ID",
  "task_type": "binary-classification",
  "context_window": 514
}
```

The training will take a few minutes. You can check the status of the model using the following API call:

=== "API"

    ```bash

    curl -X 'GET' \
      'https://api.phospho.ai/v2/models/YOUR_MODEL_ID' \
      -H 'accept: application/json' \
      -H 'Authorization: Bearer $PHOSPHO_API_KEY'
    ```

    ```python Python
    import requests
    import os

    model_id = "YOUR_MODEL_ID" # model["id"] if you run the above code
    url = f"https://api.phospho.ai/v2/models/{model_id}"

    headers = {"accept": "application/json",
              "Content-Type": "application/json",
              "Authorization": f"Bearer {os.environ['PHOSPHO_API_KEY']}"
              }

    response = requests.get(url, headers=headers)

    print(response.text)
    ```

Your model will be ready when the status will changed from `training` to `trained`.

## Use the model

You can use the model 2 ways:

- directly download it from Hugging Face (`phospho-small` runs on CPU)
- through the phospho API

### Download and use locally your model (recommended for production)

You can download the model from phospho Hugging Face repo. The model id is the same as the one you got when training the model.

For example, if the model id is `phospho-small-8963ba3`, you can download the model from Hugging Face with the id `phospho-app/phospho-small-8963ba3`.

Then you can use the model like any other Hugging Face model:

```python
from setfit import SetFitModel

model = SetFitModel.from_pretrained("phospho-app/phospho-small-8963ba3")

outputs = model.predict(["This is a sentence to classify", "Another sentence"])
```

Make sure to have enough RAM to load the model and the tokenizer in memory. The model is 420MB.

### Use the model through the API

<Warning>
  {" "}
  AI Models predict endpoints are in preview and not yet ready for production trafic.
</Warning>

To use the model through the API, you need to send a POST request to the `/predict` endpoint with the model id and the batch of text to classify.
If it's the first request you send, you might experience a delay as the model is loaded in memory.

=== "API"

```bash API

  curl -X 'POST' \
    'https://api.phospho.ai/v2/predict' \
    -H 'accept: application/json' \
    -H 'Authorization: Bearer $PHOSPHO_API_KEY' \
    -H 'Content-Type: application/json' \
    -d '{
    "inputs": [
      "Can I have a discount on phospho pro?"
    ],
    "model": "YOUR_MODEL_ID"
  }'
```

```python Python
# Coming soon!
```

## List your models

You can also list all the models you have have access to and that can accept requests:

=== "API"

    ```bash HTTP API

    curl -X 'GET' \
      'https://api.phospho.ai/v2/models' \
      -H 'accept: application/json' \
      -H 'Authorization: Bearer $PHOSPHO_API_KEY'
    ```

    ```python Python
    # Coming soon!
    ```
