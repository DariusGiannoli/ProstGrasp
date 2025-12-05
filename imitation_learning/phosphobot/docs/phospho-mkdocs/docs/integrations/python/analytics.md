---
title: Analyze your logs in Python
description: "Run custom analytics jobs in Python on your phospho logs"
---

Use the `phospho` Python package to run custom analytics jobs on your logs.

## Setup

Instal the package and set your API key and project ID as environment variables.

```bash
pip install phospho pandas
export PHOSPHO_API_KEY=your_api_key
export PHOSPHO_PROJECT_ID=your_project_id
```

## Load logs as a DataFrame

The best way to analyze your logs is to load them into a [pandas](https://pandas.pydata.org) DataFrame. This format is compatible with most analytics libraries.

### One row = one (task, event) pair

Phospho provides a `tasks_df` function to load the logs into a flattened DataFrame. Note that you need to have the `pandas` package installed to use this function.

```python
import phospho

phospho.init()
phospho.tasks_df(limit=1000) # Load the latest 1000 tasks
```

This will return a DataFrame where one row is one (task, event) pair.

Example:

| task_id                          | task_input | task_output | task_metadata                                      | task_eval | task_eval_source | task_eval_at        | task_created_at     | session_id                       | session_length | event_name                  | event_created_at    |
| -------------------------------- | ---------- | ----------- | -------------------------------------------------- | --------- | ---------------- | ------------------- | ------------------- | -------------------------------- | -------------- | --------------------------- | ------------------- |
| b58aacc6102f4a5e9d2364202ce23bf2 | Some input | Some output | \{'client_created_at': 1709925970, 'last_update... | success   | owner            | 2024-03-08 19:27:49 | 2024-03-09 15:09:31 | 71ee278ab2874666ae157c28a69c1679 | 2              | correction by user          | 2024-03-08 19:27:43 |
| b58aacc6102f4a5e9d2364202ce23bf2 | Some input | Some output | \{'client_created_at': 1709925970, 'last_update... | success   | owner            | 2024-03-08 19:27:49 | 2024-03-09 15:09:31 | 71ee278ab2874666ae157c28a69c1679 | 2              | user frustration indication | 2024-03-08 19:27:43 |
| b58aacc6102f4a5e9d2364202ce23bf2 | Some input | Some output | \{'client_created_at': 1709925970, 'last_update... | success   | owner            | 2024-03-08 19:27:49 | 2024-03-09 15:09:31 | 71ee278ab2874666ae157c28a69c1679 | 2              | follow-up question          | 2024-03-08 19:27:43 |

This means that:

- If a task has multiple events, there will be multiple rows with the same `task_id` and different `event_name`.
- If a task has no events, it will have one row with `event_name` as `None`.

### One row = one task

If you want one row to be one task, pass the parameter `with_events=False`.

```python
phospho.tasks_df(limit=1000, with_events=False)
```

Result:

| task_id                          | task_input | task_output | task_metadata                 | task_eval | task_eval_source | task_eval_at        | task_created_at     | session_id                       | session_length |
| -------------------------------- | ---------- | ----------- | ----------------------------- | --------- | ---------------- | ------------------- | ------------------- | -------------------------------- | -------------- |
| 21f3b21e8646402d930f1a02159e942f | Some input | Some output | \{'client_created_at':42f'... | failure   | owner            | 2024-03-08 19:53:59 | 2024-03-09 16:45:18 | a6b1b4224f874608b6037d41d582286a | 2              |
| 64382c6093b04a028a97a14131a4ab32 | Some input | Some output | \{'client_created_at':42f'... | success   | owner            | 2024-03-08 19:27:48 | 2024-03-09 15:51:07 | 9d13562051a84d6c806d4e6f6a58fb37 | 1              |
| b58aacc6102f4a5e9d2364202ce23bf2 | Some input | Some output | \{'client_created_at':42f'... | success   | owner            | 2024-03-08 19:27:49 | 2024-03-09 15:09:31 | 71ee278ab2874666ae157c28a69c1679 | 3              |

### Ignore session features

To ignore the sessions features, pass the parameter `with_sessions=False`.

```python
phospho.tasks_df(limit=1000, with_sessions=False)
```

## Run custom analytics jobs

To run custom analytics jobs, you can leverage all the power of the Python ecosystem.

If you have a lot of complex ML models to run and LLM calls to make, consider the phospho lab that streamlines some of the work for you.

<Card title="Discover the phospho lab" href="/local/quickstart">
Set up the phospho lab to run custom analytics jobs on your logs
</Card>

## Update logs from a DataFrame

After running your analytics jobs, you might want to update the logs with the results.

You can use the `push_tasks_df` function to push the updated data back to Phospho. This will override the specified fields in the logs.

```python
# Fetch the 3 latest tasks
tasks_df = phospho.tasks_df(limit=3)
```

### Update columns

Make changes to columns. **Not all columns are updatable.** This is to prevent accidental data loss.

Here is the list of **updatable columns:**

- `task_eval: Literal["success", "failure"]`
- `task_eval_source: str`
- `task_eval_at: datetime`
- `task_metadata: Dict[str, object]` (Note: this will override the whole metadata object, not just the specified keys)

<Info>
If you need to update more fields, feel free to open an issue on the [GitHub repository](https://github.com/phospho-app/phospho/issues), submit a PR, or directly [reach out](mailto:contact@phospho.ai).
</Info>
    
```python
# Make some changes
tasks_df["task_eval"] = "success"
tasks_df["task_metadata"] = tasks_df["task_metadata"].apply(
    # To avoid overriding the whole metadata object, use **x to unpack the existing metadata
    lambda x: {**x, "new_key": "new_value", "stuff": 44}
)
```

### Push updated data

To push the updated data back to Phospho, use the `push_tasks_df` function.

- You need to pass the `task_id`
- As a best practice, pass **only** the columns you want to update.

```python
# Select only the columns you want to update
phospho.push_tasks_df(tasks_df[["task_id", "task_eval"]])

# To check that the data has been updated
phospho.tasks_df(limit=3)
```

You're all set. Your custom analytics are now also available in the Phospho UI.
