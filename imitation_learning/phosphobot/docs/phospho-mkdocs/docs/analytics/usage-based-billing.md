---
title: Usage-based billing
description: "How phospho usage is measured"
---

This documents documents the `usage based` billing plan of the hosted phospho platform.

## What is usage-based billing?

Every analytics run on phospho consumes a certain amount of credits. 

At the end of the month, the total credits consumed by all the analytics runs are calculated and the user is billed based on the total credits consumed.

The cost per credit depends on the plan you are on.

## How many credits does an analytics run consume?

| Analytics run | Credits consumed |
----------------|------------------|
| Logging 1 Task | 0 |
| Event detection on 1 Task: Tagger | 1 |
| Event detection on 1 Task: Scorer | 1 |
| Event detection on 1 Task: Classifier | 1 |
| Clustering on 1 Task | 2 |
| Event detection on 1 Session: Tagger | 1 * number of tasks in the session |
| Event detection on 1 Session: Scorer | 1 * number of tasks in the session |
| Event detection on 1 Session: Classifier | 1 * number of tasks in the session |
| Clustering on 1 Session | 2 * number of tasks in the session |
| Language detection on 1 Task | 1 |
| Sentiment detection on  1 Task | 1 |

## How to optimize credit consumption?

- Instead of using multiple taggers, use a single classifier
- Filter the scope of clustering to only the required tasks
- Disable unnecessary analytics in Project settings

