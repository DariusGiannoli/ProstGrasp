import os
import openai
from openai import OpenAI
from dotenv import load_dotenv
import json

load_dotenv()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

SYSTEM_PROMPT_PATH = os.path.join(os.path.dirname(__file__), "..", "prompts", "prompt_compact.txt")
with open(SYSTEM_PROMPT_PATH, "r") as f:
    system_prompt = f.read()

def get_llm_response(text: str):
    stream = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": text}
        ],
        max_tokens=400,
        temperature=0.7,
        stream=True
    )

    full_response = ""
    for chunk in stream:
        delta = chunk.choices[0].delta
        content = delta.content or ""
        full_response += content

    result = full_response.strip()

    # 🧼 Clean json wrapper
    if result.startswith("```json"):
        result = result.replace("```json", "").replace("```", "").strip()

    # ✅ Try to parse JSON command
    if result.startswith("{"):
        try:
            return json.loads(result)
        except Exception:
            pass  # Fall back to plain reply

    # 🗣️ Return plain reply if it's not a JSON
    return { "reply": result }
