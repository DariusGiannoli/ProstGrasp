---
title: "Getting started"
description: "Clusterize your text messages in 5 minutes. No code required."
---

This guide will help you get started with the hosted version of [phospho](https://platform.phospho.ai).

1. **Create an account if needed**. Go to [phospho.ai](https://platform.phospho.ai). This is free.
2. **Import your first messages**. Upload a csv file with text messages (or log to the API).
3. **Run your first clustering**. Discover the results in your dashboard.

## 1. Signing up

Go to the [phospho platform](https://platform.phospho.ai/). Login or create an account if you don't have one.

We recommend you use your company email address to create an account. This will let you easily invite your team members to collaborate on the same project.

## 2. Import your first messages

There are [several ways](./import-data/import-file.md) to import your data to phospho. The easiest is to upload a file. Let's see how to do it.

### Format your file

Format your `.csv` or `.xlsx` file to have the following columns:

- `input` : the input text data, ususally the user message
- `output` : the output text, ususally the LLM app response

Additonally, you can add the following columns:

- `task_id`: an id of the task (input/output couple)
- `session_id`: an id of the session. Messages with the same session_id will be grouped together
- `user_id`: the id of the user that sent the message
- `created_at`: the creation date of the task (format it like `"2021-09-01 12:00:00"`)
- other columns will be stored as _metadata_ and can be used for filtering

The maximum upload size with this method is 500MB.

### Upload the file to the plateform

Click the setting icon at the top right of the screen and select `Import data`.

![Import data](./images/import/import_data.png)


Then, click the **Upload dataset** button and use **Choose file** button to select your file. After selecting the file, click on the **Send file** button.

Your file will be uploaded and processed in a few minutes, depending on the size of the file.

## 3. Run your first clustering

!!! note

    You need to have a **payment method** set up to run a clustering. Add a
    payment method in Settings and claim your free credits.


Now that you imported data, you can run your first clustering.

Go to the **Clusters** page by clicking on **Clusters** on the sidebar.

On top, click on the `Configure clusters detection`button. Change the parameters if needed. Finally, click on the `Run cluster analysis` button.

The clustering will take some time to run. When it's finished, you'll see the results on the page.

![Clustering](./images/clustering-demo.gif)

Deep dive into the clusters or try different parameters to get different results.

### Tips

!!! info

    Running a clustering costs 2 credits per message.

- Click on a cluster to see the messages inside.
- Click on the pickaxe icon to breakdown a cluster into smaller clusters.
- Try different parameters (filters, scope, user query) to get different results.

Learn more about clustering [here.](/docs/analytics/clustering)

## Tired of uploading files? Setup the API

Learn more about how to [log to phospho](/docs/import-data/api-integration) in your app in a few minutes.

## Next steps

<div class="grid cards" markdown>

-   :material-tag-multiple:{ .lg .middle } __Automatic tagging__

    ---

    Automatically annotate your text data and be alerted. **Take action.**

    [:octicons-arrow-right-24: Tagging](/docs/analytics/tagging)

-   :material-cluster:{ .lg .middle } __Unsupervised clustering__

    ---

    Group users' messages based on their intention. **Find out what your users are talking about.**

    [:octicons-arrow-right-24: Clustering](/docs/analytics/clustering)

-   :material-test-tube:{ .lg .middle } __AB Testing__

    ---

    Run experiments and iterate on your LLM app, while keeping track of performances. **Keep shipping.**

    [:octicons-arrow-right-24: AB Testing](/docs/analytics/ab-testing)

-   :material-cog-sync:{ .lg .middle } __Flexible evaluation pipeline__

    ---

    Discover how to run and design a text analytics pipeline using natural language. **No code needed.**

    [:octicons-arrow-right-24: Evaluation pipeline](/docs/analytics/events)

-   :material-account-details:{ .lg .middle } __User analytics__

    ---

    Detect user languages, sentiment, and more. **Get to know power users.**

    [:octicons-arrow-right-24: User analytics](/docs/analytics/language)

</div>

