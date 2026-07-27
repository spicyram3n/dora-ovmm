# Bug: sam3_trigger.py silently failed to save its output image

**Where:** `sam3_trigger.py`

**Symptom:** Script printed `saved /home/ws/zenoh_examples/sam3_result.png`
every run, but the file never existed.

**Root cause:** `/home/ws/zenoh_examples/` was never created.
`cv2.imwrite` does not create missing directories and does not raise an
error on failure, it just returns `False`. The return value was never
checked, so the script kept reporting success.

**Old code:**
```python
OUT_PATH = "/home/ws/zenoh_examples/sam3_result.png"
```

**Fixed code:**
```python
OUT_PATH = Path(__file__).resolve().parent / "sam3_result.png"
```

Now the output saves next to the script itself, a path that always exists.
