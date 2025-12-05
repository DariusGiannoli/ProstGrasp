---
title: Automatic tagging
description: "phospho automatically tags your data and warns you when needed"
---

## How are tags detected?

Every message logged to phospho goes through an analytics pipeline. In this pipeline, phospho looks for **tags** defined in your project settings.

Tags are described in **natural language**. Create tags to detect topics, hallucinations, behaviours, intents, or any other concept you want to track.

Tags are displayed on the platform and you can use them to filter data.

Be notified when a tag is detected with **webhooks**.

### Example of tags

- The user is trying to book a flight
- The user thanked the agent for its help
- The user is asking for a refund
- The user bought a product
- The assistant responded something that could be considered financial advice
- The assistant talked as if he was a customer, and not a support

## Create tags

Go to the **Analytics** tab of the [phospho dashboard](https://platform.phospho.ai/), and click Add Tagger on the right.

You will find some event templates like Coherence and Plausibility to get you started.

![Events tab](../images/explore/events%20detection/Create%20event.png)


### Tag definition

The event description is a natural language description of the tag. Explain how to detect the tag in an interaction as if you were explaining it to a 5 years old or an alien.

In the description, refer to your user as "the user" and to your LLM app as "the assistant".

!!! example "Example of an event description"
    > _The user is trying to book a flight. The user asked a question about a flight.
    Don't include fight suggestions from the agent if the user didn't ask for it._

Manage Tags in the **Analytics** tab. Click delete to delete a tag detector.

### Tag suggestion

Note that you can also use the magic wand button on any session to get a suggestion for a possible tag that has been detected in the session.

![Tag suggestion](../images/explore/events%20detection/Event%20suggestion.png)

The button is right next to "Events" in the Session tab.

## Webhooks

Add an optional webhook to be notified when an event is detected. Click on **Additional settings** to add the webhook URL and the eventual Authorization header.

### What is a webhook?

Webhooks are automated messages sent from apps when something happens. They have a payload and are sent to a unique URL, which is like an app's phone number or address. 

If you have an LLM app with a backend, you can create webhooks.

### How to use the webhook?

Every time the event is detected, phospho will send a `POST` request to the webhook with this payload:

```json
{
    "id": "xxxxxxxxx", // Unique identifier of the detected event
    "created_at": 13289238198, // Unix timestamp (in seconds)
    "event_name": "privacy_policy", // The name of the event, as written in the dashboard
    "task_id": "xxxxxxx", // The task id where the event was detected
    "session_id": "xxxxxxx", // The session id where the event was detected
    "project_id": "xxxxxxx", // The project id where the event was detected
    "org_id": "xxxxxxx", // The organization id where the event was detected
    "webhook": "https://your-webhook-url.com", // The webhook URL
    "source": "phospho-unknown", // Starts with phospho if detected by phospho
}
```

Retrieve the messages using the `task_id` and the [phospho API.](https://api.phospho.ai/v2/redoc#tag/Tasks/operation/get_task_tasks__task_id__get)

### Examples

Use webhooks to send slack notifications, emails, SMS, notifications, UI updates, or to trigger a function in your backend.