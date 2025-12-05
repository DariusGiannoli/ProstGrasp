---
title: Create Custom Jobs
description: Create custom jobs and run them on your messages with phospho
---

phospho comes with several built-in jobs that you can use to process your messages: zero-shot evaluation, classification based evaluation, event detection...

But you can also create your own jobs and run them on your messages. This is what we call a custom job.

## Creating a custom job function

To create a custom job function, you need to create a function that:

- takes a `lab.Message` as input
- can take additional parameters if needed (they will be passed as `JobConfig`)
- returns a `lab.JobResult`.
  The `lab.JobResult` should contain the result of the job function and the type of the result.

For instance, to define a simple job that checks if a message contains a forbidden word, you can create a Job function like this:

```python
from phospho import lab
from typing import List
import re

def my_custom_job(message: lab.Message, forbidden_words: List) -> lab.JobResult:
    """
    For each each message, me will check if the forbidden words are present in the message.
    The function will return a JobResult with a boolean value
    (True if one of the words is present, False otherwise).
    """

    pattern = r'\b(' + '|'.join(re.escape(word) for word in forbidden_words) + r')\b'

    # Use re.search() to check if any of the words are in the text
    if re.search(pattern, message.content):
        result = True
    else:
        result = False

    return lab.JobResult(
        job_id="my_custom_job",
        result_type=lab.ResultType.bool,
        value=result,
    )
```

## Running a custom job

Once you have defined your custom job function, you can create a Job in your workload that will run this job function on your messages.

You need to pass the function in the `job_function` of the `lab.Job` object.

In our example:

```python
# Create a workload in our lab
workload = lab.Workload()

# Add our job to the workload
workload.add_job(
    lab.Job(
        id="regex_check",
        job_function=my_custom_job, # We add our custom job function here
        config=lab.JobConfig(
            forbidden_words=["cat", "dog"]
        ),
    )
)
```

This workload can then be run on your messages using the `async_run` method.

```python
await workload.async_run(
    messages=[
        # No forbiden word is present.
        lab.Message(
            id="message_1",
            content="I like elephants.",
        ),
        # One forbiden word is present.
        lab.Message(
            id="message_2",
            content="I love my cat.",
        )
    ]
)

# Let's see the results
for i in range(1, 3):
    print(
        f"In message {i}, a forbidden word was detected: {workload.results['message_'+str(i)]['regex_check'].value}"
    )

# In message 1, a forbidden word was detected: False
# In message 2, a forbidden word was detected: True
```
