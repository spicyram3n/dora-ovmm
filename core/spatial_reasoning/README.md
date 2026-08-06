# spatial_reasoning

Everything that talks to an LLM. Kept apart from `scene_graph/` on purpose, so
that building a graph never needs an API key or a network connection.

The model is scoped narrowly: it names rooms and guesses which furniture an
object is likely to be at. It is never asked for coordinates, never asked to
do arithmetic, and never trusted without checking — a furniture id it invents
is dropped before it can send the robot anywhere.

## Files

| File | Purpose |
|---|---|
| `_deepseek.py` | The client, `<think>` stripping, and `ask_json`: one turn in, validated pydantic out |
| `rooms.py` | Groups furniture into named rooms and writes them onto the graph |
| `query_sg.py` | Scenario A and B: where should the robot look for this object? |

## Running

Nothing here is a program. `rooms.assign` is called by
`core/build_scene_graph.py`, and `query_sg.search_order` by
`core/search_object.py`.

```python
from spatial_reasoning import query_sg
from spatial_reasoning._deepseek import get_client

for location in query_sg.search_order(scene, "pringles", get_client(),
                                      cache="outputs/scene_graph/apartment/locations"):
    print(location.source, location.label, location.room, location.reason)
```

## The two scenarios

- **B — the object is already in the graph.** Its recorded furniture is the
  first place to try, and costs no LLM call at all.
- **A — it is not.** The model is given this scene's furniture and asked for
  the top *k* places, most likely first.

`search_order` is a **generator** so that B falls through to A only when the
robot actually comes back empty-handed. A list would pay for the LLM call
every time, including when the remembered location was right.

## Caching

`predict(..., cache=<dir>)` writes the model's ranking to
`<dir>/<object>.json` and reuses it, so the same search does not pay twice.
The whole ranking is stored, not the filtered result, so the file still serves
a later search that has already ruled some furniture out. Delete the file to
force a fresh answer.

## Environment

`DEEPSEEK_API_KEY` must be set. `get_client` raises a clear error if it is not.
