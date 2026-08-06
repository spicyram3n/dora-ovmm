import os

from openai import OpenAI

def get_client() -> OpenAI:
    api_key = os.getenv("DEEPSEEK_API_KEY")
    if not api_key:
        raise ValueError("Set the DEEPSEEK_API_KEY environment variable")
    return OpenAI(base_url="https://api.deepseek.com/v1", api_key=api_key)

def strip_think(raw: str) -> str:
    "deepseek reasoner prefixes the answer with a <think>..</think> tag; we want to keep only the json that follows it"

    if "</think>" in raw:
        return raw.split("</think>")[-1].strip()

    return raw


def ask_json(client, system, user, schema, model="deepseek-reasoner"):
    """One chat turn that has to come back as json matching `schema`.

    The reasoner ignores response_format, so the object is taken as the
    outermost {...} in the reply rather than by splitting on ```json fences,
    which break the moment the model wraps its answer differently. Validating
    matters as much as parsing: a model that invents a furniture id would
    otherwise send the robot to a node that does not exist.
    """
    reply = client.chat.completions.create(
        model=model,
        messages=[{"role": "system", "content": system},
                  {"role": "user", "content": user}],
    ).choices[0].message.content

    text = strip_think(reply)
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end < start:
        raise ValueError(f"no json object in model reply: {text[:200]}")
    return schema.model_validate_json(text[start:end + 1])