---
title: Import from Langfuse 🪢
description: Sync your Langfuse data with phospho
---

# Go to Langfuse and head to settings

Go to your [langfuse](https://cloud.langfuse.com/) account and head to the settings page, in the bottom left.

You will reach the API Keys page where you can create a new API key.

![langfuse api key](../images/import/langfuse_api_keys.png)

Click on Create new API keys, you will need both the secret key and the public key.

# Head to phospho and import your data

Click the settings icon at the top right of the screen and select `Import data`.

![Click the settings icon](../images/import/import_data.png)

Then click, the **Import from langfuse** button.

You can now copy your Secret Key and your Public Key in the input fields.

<Note>
  This data is encrypted and stored securely. We need it to periodically fetch
  your data from LangFuse and import it into phospho.
</Note>

![Import from langfuse](../images/import/start_sending_data.png)

Your data will be synced to your project in a minute.

# Next steps

Default evaluators like language and sentiment will be run on messages. To create more events and to run them on your data, see the [event detection page](/docs/guides/events)
