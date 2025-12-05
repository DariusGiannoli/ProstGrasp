---
title: Self-hosting
description: "Host phospho on your own infrastructure using phospho open source"
---

The phospho platform can be hosted on your own infrastructure. [The code is open source and available here.](https://github.com/phospho-app/phospho)

This is useful if you want to keep your data private or if you have specific data compliance requirements.

## How to deploy phospho with Docker?

The platform can be deployed using Docker. Start by cloning the phospho repository.

```bash
git clone https://github.com/phospho-app/phospho.git
```

Once the environment variables are set up, you can then use [Docker compose](https://docs.docker.com/compose/intro/features-uses/) to quickly build and deploy the platform.

```bash
docker compose up
```

Please follow [this guide for the complete instructions](https://github.com/phospho-app/phospho/blob/dev/DeploymentGuide.md) on how to setup environment variables.

## How to deploy phospho on the Cloud?

phospho is compatible with any cloud provider thanks to its container-based architecture.

- Google Cloud platform (feel free to refer to the [deployment scripts here](https://github.com/phospho-app/phospho/tree/dev/.github/workflows))
- Microsoft Azure
- Amazon Web Services

To get started easily, we recommend you use [Porter.run](https://docs.porter.run/introduction).

### Contact us

To get help, feel free to reach out at [contact@phospho.ai](mailto:contact@phospho.ai)
