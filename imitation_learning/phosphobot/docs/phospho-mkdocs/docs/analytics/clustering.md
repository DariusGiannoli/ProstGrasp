---
title: Clustering
description: "Group users messages based on their intention"
---

Clustering lets your group user messages based on their intention. This is great to get a feeling of "what are my users talking about?" and to identify the most common topics.

![Clustering](../images/clustering-demo.gif)

## How it works

The phospho clustering uses a combination of **user intent embedding** and **unsupervized clustering algorithms** to group messages together.

The user intent embedding is a representation of the user intention in a high dimensional space. This representation is generated using a deep learning model trained on a large dataset of user messages. [Learn more here.](https://research.phospho.ai/phospho_intent_embed.pdf)

We are constantly evaluating and improving the clustering algorithms to provide the best results.

## How to run a clustering

To use the clustering feature, you need to have a phospho account and an API key. You can get one by signing up on [phospho.ai](https://platform.phospho.ai).

1. **Import data**. If not already done, [import your data](/docs/import-data/import-file) and setup a payment method.

2. **Configure clustering**. Go to the **Clusters** tab and click on the *Configure clustering detection* button.
  Select the scope of data to cluster: either messages or sessions.
  Filter the data by setting a date range, a specific tag, and more.

3. **Run clustering**.
  Click on the *Run cluster analysis* button to start the clustering process. Depending on the number of messages, it can take a few minutes.

<iframe
  width="100%"
  height="600px"
  src="https://www.youtube.com/embed/WwvBXJieiW0?si=q--tDY1iqON0GHo5"
  allowFullScreen
></iframe>

## How to interpret the results

The clustering results are presented in two formats:

- **3D Dot Cloud Graph**: Each point in the graph corresponds to an embedding of a message (or a session). Clusters are distinct groups of these points. 

- **Cluster Cards**: Each cluster is also displayed as a card. The card shows the cluster size and an automatic summary of a random sample of messages. Click on "Explore" in any card to view the messages in the cluster.

## How to run a clustering with a custom instruction?

By default, the clustering is run based on: `user intent` 

You can however modify this instruction in *Advanced settings*. 

Change the clustering instruction to refine how messages are grouped, to provide insights that are more aligned with your needs. You just need to enter the **topic** you want to cluster on.

Examples of what you can enter:
- For a medical chatbot: `type of disease` 
- For a customer support chatbot: `type of issue (refund, delivery, etc.)`
- For a chatbot in the e-commerce industry: `product mentioned` 

## How to run a custom clustering algorithms?

You can use the user intent embeddings to run your own clustering algorithms. The embeddings are available through the API. [Learn more here.](/docs/models/embeddings)

## Next steps

Based on the clusters, define more analytics to run on your data in order to never miss a beat on what your users are talking about. Check the [event detection page](/docs/analytics/events) for more information.
