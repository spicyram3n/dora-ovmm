# Boxer → scene graph

Robot RGB-D keyframes → 3D boxes → a scene graph the reasoner can query.

> **Last run, 2026-09-15:** stopped after 78 of 580 frames, so the graph covers only the start of the route. Result: `config/realrobot/scene_graph/lab_20260811.json`, with 11 furniture and 34 objects. For the whole lab, rerun step 2, then step 3.

- Run every command from the **repo root**.
- Steps 0 and 2 need **Docker**, so run them on the GPU host.
- Steps 3–5 run in the **dev container**.
- Where things land ([details](#whats-where)):
  - generated data for one bag → **`outputs/realrobot/lab_20260811/`**;
  - what the robot runs with (the map, transforms, the scene graph) → **`config/realrobot/`**;
  - this folder keeps only code, weights (`ckpts/`) and `cache/`.
- Another recording? Replace `lab_20260811` everywhere below.

---

## 0. Once

```bash
bash docker/boxer/run_boxer.sh --download-ckpts
```

- [ ] Weights land in `docker/boxer/ckpts/` (1.2 GB).
- The first call also builds `hrl/boxer:latest` (17 GB).

---

## 1. Keyframes → Boxer layout · 1 s

```bash
python3 realrobot/dataprep/make_boxer_scene.py \
  --source outputs/realrobot/lab_20260811/keyframes \
  --output outputs/realrobot/lab_20260811/boxer/scannet/lab_20260811 \
  --registration config/realrobot/map/lab_20260811_boxer_to_map.json
```

- [ ] Prints `Prepared 580/580 keyframes`.

---

## 2. Detect → 3D boxes → fuse · ~40 min on CPU

```bash
BOXER_CPU=1 BOXER_DATA=$PWD/outputs/realrobot/lab_20260811/boxer \
  bash docker/boxer/run_boxer.sh \
  --input /opt/boxer/sample_data/scannet/lab_20260811 \
  --labels=scannet200 --fuse
```

- [ ] `outputs/realrobot/lab_20260811/boxer/lab_20260811/boxer_3dbbs_fused.csv` exists.
- Check the detections in `boxer_viz_final.mp4`, in the same folder.
- GPU free in `nvidia-smi`? Drop `BOXER_CPU=1` for a much faster run.

---

## 3. Boxes → scene graph · 1 s

```bash
python3 -m docker.boxer.to_scene_graph \
  --boxes outputs/realrobot/lab_20260811/boxer/lab_20260811/boxer_3dbbs_fused.csv \
  --transform config/realrobot/map/lab_20260811_boxer_to_map.json \
  --output config/realrobot/scene_graph/lab_20260811.json
```

- [ ] Prints `N furniture, M objects`.

---

## 4. Look at it

```bash
python3 visualization/scene_graph.py \
  --graph config/realrobot/scene_graph/lab_20260811.json \
  --scene outputs/realrobot/lab_20260811/keyframes/scene.ply \
  --output outputs/realrobot/lab_20260811/scene_graph.rrd
```

- [ ] Rerun opens, and the boxes sit on the grey point cloud.
- Add `--save-only` to only write the `.rrd`.
- Reopen a saved recording at any time:

  ```bash
  python3 -m rerun outputs/realrobot/lab_20260811/scene_graph.rrd
  ```

---

## 5. Ask the reasoner

```bash
python3 -m core.reasoner.query 'laptop' \
  --graph config/realrobot/scene_graph/lab_20260811.json
```

- `--top-k N` sets how many furniture guesses DeepSeek returns. `--near X Y` orders remembered matches nearest first.
- With `DEEPSEEK_API_KEY` set, it also asks DeepSeek once the remembered locations run out.
- Mission on the real robot: `python3 -m core.pipeline.mission_tree --target laptop --graph config/realrobot/scene_graph/lab_20260811.json`, on a stack you started by hand ([navigation README](../../core/navigation/README.md#real-robot)). `search.launch.py` and its `graph:=` are simulation-only.

---

## What's where

```text
outputs/realrobot/lab_20260811/     generated data from the 2026-08-11 bag
├── slam/                           data prep step 1: pose graph, map→odom recording
├── keyframes/                      data prep step 2: color/ depth/ poses/, scene.ply, intrinsics.txt
├── boxer/
│   ├── scannet/lab_20260811/frames/   step 1: Boxer's input, hardlinks to keyframes/ (no extra disk use)
│   └── lab_20260811/               step 2: Boxer's results
│       ├── owl_2dbbs.csv             2D boxes, per frame
│       ├── boxer_3dbbs.csv           3D boxes, per frame
│       ├── boxer_3dbbs_fused.csv     one row per object (seen ≥ 4 times, confidence ≥ 0.55)
│       └── boxer_viz/, boxer_viz_final.mp4   drawn detections (the mp4 only if the run finishes)
└── scene_graph.rrd                 step 4: the Rerun recording

config/realrobot/                   what the robot runs with
├── map/
│   ├── lab_20260811.{pgm,yaml}          data prep step 1: the Nav2 map
│   ├── lab_20260811_scan_to_map.json    data prep step 2: identity (keyframes are already in map)
│   └── lab_20260811_boxer_to_map.json   step 1: translation by camera 0's position; undoes Boxer's recentring
└── scene_graph/lab_20260811.json   step 3: the scene graph (the mission saves what it finds back into it)

docker/boxer/ckpts/                        step 0: model weights
docker/boxer/cache/scannet200_classes.csv  every run_boxer.sh call: the prompt list
```

Data prep steps 1–2 are in [realrobot_dataprep.md](../../realrobot/dataprep/realrobot_dataprep.md). The pair `scannet/lab_20260811` + `lab_20260811` is Boxer's convention: it only reads ScanNet from a path containing `/scannet/`, and it names its results folder after the input folder.

---

## Stopped step 2 early?

1. Stop it with **Ctrl-C**, or `docker kill -s INT <container>`. A plain `docker kill` loses the unflushed boxes.
2. Fuse what it already saved:

   ```bash
   BOXER_CPU=1 BOXER_DATA=$PWD/outputs/realrobot/lab_20260811/boxer \
     bash docker/boxer/run_boxer.sh \
     --input /opt/boxer/sample_data/scannet/lab_20260811 \
     --labels=scannet200 --cache3d --fuse
   ```

3. Go to step 3. The graph only covers the frames that finished.

---

## Don't

| Don't | Because |
| --- | --- |
| Forget `BOXER_DATA` in step 2 | Boxer then looks in `outputs/boxer/` and can't find the sequence |
| Pass `--start_n` | It moves Boxer's origin, so the `_boxer_to_map.json` no longer matches. `--skip_n` and `--max_n` are fine |
| Feed `boxer_3dbbs.csv` to step 3 | It has one row per frame, not per object. Use `_fused.csv` |
| Shorten `--input` to a bare name | Boxer picks its ScanNet loader only for a path containing `/scannet/` |

---

## Knobs

| Want | Add |
| --- | --- |
| Faster run, fewer frames | `--stride 2` in step 1 |
| No video | `--skip_viz` in step 2 |
| Fewer weak boxes | `--min-confidence 0.6` in step 3 |
| Quick image test | `bash docker/boxer/run_demo.sh` (upstream Aria demo; writes to `outputs/boxer/`) |

---

## Broken?

| You see | Do |
| --- | --- |
| `Permission denied` on Boxer's files | The container writes as root: `sudo chown -R $USER outputs/realrobot/lab_20260811/boxer docker/boxer/{ckpts,cache}` |
| `File exists` in step 1 | Delete that output folder and rerun |
| `Boxes span several frames` in step 3 | You passed the per-frame CSV. Use `_fused.csv` |
| CUDA out of memory | Use `BOXER_CPU=1` |
| `no space left on device` while building | `docker image prune` |
| `Failed to open connection to X server` in step 4 | The container kept the `DISPLAY` of an older login. Run `echo $DISPLAY` on the host, then `export DISPLAY=:1` (that value) in the container. Rebuilding the container fixes it for good |

---

<details><summary>Why it works (read once)</summary>

- **No conversion.** `extract_rgbd.py` already writes what Boxer's ScanNet loader reads: optical camera→map poses, uint16 mm depth and a 4×4 K. `make_boxer_scene.py` hardlinks the frames.
- **One folder, two mounts.** `run_boxer.sh` mounts `BOXER_DATA` as both Boxer's `sample_data/` and its `output/`. So `--input /opt/boxer/sample_data/scannet/lab_20260811` is `…/boxer/scannet/lab_20260811`, and results land beside it in `…/boxer/lab_20260811/`.
- **One translation.** The loader moves the origin to the first camera. `lab_20260811_boxer_to_map.json` moves it back. `graph.build()` checks that it is rigid, and accepts `source_frame: "map"` only with an identity.
- **The graph's own classes.** `--labels=scannet200` is [scannet200.yaml](../../config/scene_graph/scannet200.yaml) minus `object`, `furniture` and `structure`, which would outscore the specific class. `run_boxer.sh` regenerates the list on every run.
- **Same answer every run.** Fusion breaks tied label votes in Python-set order. `run_boxer.sh` fixes `PYTHONHASHSEED`, so reruns agree.
- **Whole lab in one run.** Boxer lifts each 2D box to 3D on its own and fuses afterwards, so there is no room-sized limit.
- **Upstream:** [facebookresearch/boxer](https://github.com/facebookresearch/boxer) at `1f86542`. Licensed CC-BY-NC, so non-commercial use only.

</details>
