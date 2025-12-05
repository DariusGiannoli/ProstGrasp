---
title: Automatic evaluation
description: 'Evaluate and score your LLM app'
---

phospho enables you to evaluate the quality (success or failure) of the interactions between your users and your LLM app.

Every time you log a task, phospho will **automatically evaluate** the success of the task. 

## How does phospho evaluate tasks?

The evaluation is based on LLM self-critique.

The evaluation leverages the following sources of information:
- The tasks annotated in the phospho webapp, by **you and your team**
- The **user feedbacks** sent to phospho
- The `system_prompt (str)` parameter in `metadata` when logging 
- Previous tasks in the **same session**

If the information are not available, phospho will use default heuristics. 

## How to improve the automatic evaluation?

To improve the automatic evaluation, you can:
- Label tasks in the phospho webapp. **Invite** your team members to help you!
- Gather [user feedback](/docs/guides/user-feedback)
- Pass the `system_prompt (str)` parameter in `metadata` when [logging](/docs/guides/sessions-and-users#metadata)
- Group tasks in [sessions](/docs/guides/sessions)
- Override the task evaluations with [the analytics endpoints](/docs/integrations/python/analytics#update-logs-from-a-dataframe)

## Annotate in the phospho webapp

In the phospho dashboard, you can annotate tasks as a success or a failure. 

### Thumbs up / Thumbs down

In the Transcript tab, view tasks to access the thumbs up and thumbs down buttons.
- A thumbs up means that the task was successful. 
- A thumbs down means that the task failed.

Update the evaluation by clicking on the thumbs.

The button **changes color** to mark that this task was evaluated by a human, and not by phospho.

### Notes

Add notes and any kind of text with the **Notes** button next to the thumbs. 

If there is a note already written, the color of the button changes. 

## Annotate with User feedback

You can gather annotations any way you want. For example, if you have your own tool to collect feedback (such as thumbs up/thumbs down in your chat interface), you can chose to use the phospho API.

Trigger [the API endpoint](https://api.phospho.ai/v2/redoc#tag/Tasks/operation/post_flag_task_tasks__task_id__flag_post) to send your annotations to phospho at scale.

Read the [full guide about user feedback](/docs/guides/user-feedback) to learn more.

## Visualize the results

Visualize the aggregated results of the evaluations in the _Dashboard_ tab of the phospho webapp.

You can also visualize the results for each task in the _Sessions_ tab. Click on a session to see the list of tasks in the session. 

A green thumbs up means that the task was successful. A red thumbs down means that the task failed. Improve the automatic evaluation by clicking on the thumbs to annotate the task if needed.