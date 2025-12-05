---
title: Testing with Python
description: "Test your agent before deploying it to production"
---

Evaluate your app's performance before deploying it to production. 

The phospho testing framework allows you to test your app with historical data, custom datasets, and custom tests.

The phospho python module **parallelizes** the function calls to **speed up** the testing process.

## Getting started

To get started, install the phospho python module.

```bash
pip install -U phospho
```

Create a new file `phospho_testing.py`:

```python
import phospho

phospho_test = phospho.PhosphoTest()
```

In this file, you can then write your tests.


## Backtesting

To use data from the phospho platform, you can use the backtest source loader.

```python
import phospho 

phospho_test = phospho.PhosphoTest()

@phospho_test.test(
    source_loader="backtest",  # Load data from logged phospho data
    source_loader_params={"sample_size": 3},
)
def test_backtest(message: phospho.lab.Message) -> str | None:
    client = phospho.lab.get_sync_client("mistral")
    response = client.chat.completions.create(
        model="mistral-small",
        messages=[
            {"role": "system", "content": "You are an helpful assistant"},
            {"role": message.role, "content": message.content},
        ],
    )
    return response.choices[0].message.content
```


## Dataset .CSV, .XLSX, .JSON

To test with a custom dataset, you can use the dataset source loader.


```python
import phospho 

phospho_test = phospho.PhosphoTest()

@phospho_test.test(
    source_loader="dataset", 
    source_loader_params={"path": "path/to/dataset.csv"},
)
def test_backtest(column_a: str, column_b: str) -> str | None:
    client = phospho.lab.get_sync_client("mistral")
    response = client.chat.completions.create(
        model="mistral-small",
        messages=[
            {"role": "system", "content": "You are an helpful assistant"},
            {"role": "user", "content": column_a},
        ],
    )
    return response.choices[0].message.content
```


Supported file formats: `csv`, `xlsx`, `json`

!!! info
    The columns of the dataset file should match the function arguments.

Example of a local csv file:

```txt
column_a, column_b
"What's larger, 3.9 or 3.11?", "3.11"
```

## Custom tests

To write custom tests, you can just create a function and decorate it with `@phospho_test.test()`. 

At the end, add `phospho.log` to send the data to phospho for analysis.

```python
import phospho

phospho_test = phospho.PhosphoTest()

@phospho_test.test()
def test_simple():
    client = phospho.lab.get_sync_client("mistral")
    response = client.chat.completions.create(
        model="mistral-small",
        messages=[
            {"role": "system", "content": "You are an helpful assistant"},
            {"role": "user", "content": "What's bigger: 3.11 or 3.9?"},
        ],
    )
    response_text = response.choices[0].message.content
    # Use phospho.log to send the data to phospho for analysis
    phospho.log(
        input="What's bigger: 3.11 or 3.9?",
        output=response_text,
        # Specify the version_id of the test
        version_id=phospho_test.version_id,
    )
```

## Run using python

To run the tests, use the `run` method of the `PhosphoTest` class.

```python
phospho_test.run()
```

The `executor_type` can be either:
- `parallel` (default): parallelizes the backtest and dataset source loader calls. 
- `parallel_jobs`: all functions are called in parallel.
- `sequential`: great for debugging.

## Run using the phospho CLI

You can also use the phospho command line interface to run the tests. In the folder where `phospho_testing.py` is located, run:

```bash
phospho init # Run this only once
phospho test
```

The executor type can be specified with the `--executor-type` flag. 

```bash
phospho test --executor-type=parallel_jobs
```

Learn more using the `--help` flag:

```bash
phospho test --help
```

<div class="grid cards" markdown>

-   :octicons-terminal-16:{ .lg .middle } __phospho CLI__

    ---

    Learn how to install phospho command line interface

    [:octicons-arrow-right-24: Read more](#)

</div>
