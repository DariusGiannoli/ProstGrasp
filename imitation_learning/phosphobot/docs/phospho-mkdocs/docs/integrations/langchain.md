---
title: Log to phospho in Python Langchain
description: Add AI analytics to your Langchain agent with phospho
---

phospho can be added to a Langchain agent as a callback handler. By default, the task input is the beginning of the chain, and the task output is the end result. Intermediate steps are also logged.

```python
from phospho.integrations import PhosphoLangchainCallbackHandler

chain = ... # Your Langchain agent or chain

chain.invoke(
    "Your chain input",
    # Add the callback handler to the config
    config={"callbacks": [PhosphoLangchainCallbackHandler()]},
)
```

## Detailed setup in a retrieval agent

### 1. Setup

Set the following environment variables:

```
export PHOSPHO_API_KEY=...
export PHOSPHO_PROJECT_ID=...
export OPENAI_API_KEY=...
```

!!! tip
    Learn how to get your project id and api key by [clicking
    here!](getting-started)

Install requirements:

```
pip install phospho openai langchain faiss-cpu
```

### 2. Add callback

The phospho module implements the Langchain callback as well as other helpful tools to interact with phospho. Learn more in the [python doc.](/docs/integrations/python)

!!! info
    The phospho module is an open source work in progress. [Your help is deeply
    appreciated!](https://github.com/phospho-app/phospho)

For example, let's create a file called `main.py` with the agent code.

phospho is integrated with langchain via the `PhosphoLangchainCallbackHandler` callback handler. This callback handler will log the input and output of the agent to phospho.

```python

from langchain.prompts import ChatPromptTemplate
from langchain_community.chat_models import ChatOpenAI
from langchain_community.embeddings import OpenAIEmbeddings
from langchain_community.vectorstores import FAISS
from langchain_core.output_parsers import StrOutputParser
from langchain_core.runnables import RunnablePassthrough

vectorstore = FAISS.from_texts(
    [
        "phospho is the LLM analytics platform",
        "Paris is the capital of Fashion (sorry not sorry London)",
        "The Concorde had a maximum cruising speed of 2,179 km (1,354 miles) per hour, or Mach 2.04 (more than twice the speed of sound), allowing the aircraft to reduce the flight time between London and New York to about three hours.",
    ],
    embedding=OpenAIEmbeddings(),
)
retriever = vectorstore.as_retriever()
template = """Answer the question based only on the following context:
{context}

Question: {question}
"""
prompt = ChatPromptTemplate.from_template(template)
model = ChatOpenAI()

retrieval_chain = (
    {"context": retriever, "question": RunnablePassthrough()}
    | prompt
    | model
    | StrOutputParser()
)


# To integrate with Phospho, add the following callback handler

from phospho.integrations import PhosphoLangchainCallbackHandler


while True:
    text = input("Enter a question: ")
    response = retrieval_chain.invoke(
        text, 
        config={
            "callbacks": [PhosphoLangchainCallbackHandler()]
        }
    )
    print(response)

```

The integration with phospho is done by adding the `PhosphoLangchainCallbackHandler` to the config of the chain. You can learn more about callbacks in the [langchain doc](https://python.langchain.com/docs/modules/callbacks/).

### 3. Test

Start the RAG agent and ask questions about the documents.

```bash
python main.py
```

The agent answers question based on retrieved documents (RAG, Retrieval Augmented Generation).

```text
Enter a question: What's the top speed of the Concorde?
The Concorde top speed is 2,179km per hour.
```

The conversation and the intermediate retrievals steps (such as the documents retrieved) are logged to phospho.


## Custom logging in langchain

For more advanced manual logging with a langchain, you can inherit from the `PhosphoLangchainCallbackHandler` and add custom behaviour.

The callback has a reference to the `phospho` object, which can be used to log custom data.

```python
from phospho.integrations import PhosphoLangchainCallbackHandler

class MyCustomLangchainCallbackHandler(PhosphoLangchainCallbackHandler):

    def on_agent_finish(self, finish: AgentFinish, **kwargs: Any) -> Any:
        """Run on agent end."""

        # Do something custom here
        self.phospho.log(input="...", output="...")

```

You can refer to the [langchain doc](https://python.langchain.com/docs/modules/callbacks/) to have the full list of callbacks available.
