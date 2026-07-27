# Bug: zenoh query params use ; not &

**Where:** `core/grasping/sam3_client.py`, `core/grasping/graspgenx_client.py`

**Symptom:** SAM3 always returned 0 instances, even for objects clearly
visible in the camera frame (confirmed by direct inspection of the captured
image, and by testing with conf=0.0 to rule out threshold filtering).

**Root cause:** Zenoh selector query strings separate parameters with `;`,
not `&` like HTTP query strings. Copying the HTTP-style `?a=1&b=2` pattern
into the zenoh client meant the second parameter's value got glued onto the
end of the first parameter's value instead of being parsed separately.

Confirmed by the server log itself: `[sam3] 'big pringles can&conf=0.5': 0
instance(s)`. The whole `&conf=0.5` ended up inside the prompt string, so
SAM3 was asked to segment a nonsense concept and found nothing.

This is why the old FastAPI/HTTP version of the server never hit this: HTTP
query strings actually do use `&`, so the same-looking code was correct
there and wrong here.

**Old code (sam3_client.py):**
```python
query = f"sam3/detect?prompt={prompt}&conf={conf}"
```

**Fixed code:**
```python
# zenoh selectors separate parameters with ';', not '&' like HTTP
# query strings. An '&' here would end up glued onto prompt's value.
query = f"sam3/detect?prompt={prompt};conf={conf}"
```

**Old code (graspgenx_client.py):**
```python
params = f"num_grasps={num_grasps}"
if gripper_name:
    params += f"&gripper_name={gripper_name}"
```

**Fixed code:**
```python
# zenoh selectors separate parameters with ';', not '&' like HTTP query strings.
params = f"num_grasps={num_grasps}"
if gripper_name:
    params += f";gripper_name={gripper_name}"
```

**Verified:** after the fix, SAM3 found both cans in the test frame with
scores 0.96-0.98, using the exact same prompts that returned 0 before.
