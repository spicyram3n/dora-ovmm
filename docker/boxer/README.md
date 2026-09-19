# Boxer → scene graph

Robot RGB-D keyframes → 3D boxes → a scene graph the reasoner can query.

> **Last run, 2026-09-19:** all 580 frames, whole route. Result: `config/realrobot/scene_graph/lab_20260811.json`, with 50 furniture and 124 objects, divided into 5 rooms on the floor plan by step 3's `--rooms --map`.

- Run every command from the **repo root**.
- Steps 0 and 2 need **Docker**, so run them on the GPU host.
- Steps 3–5 run in the **dev container**.
- Where things land ([details](#whats-where)):
  - generated data for one bag → **`outputs/realrobot/lab_20260811/`**;
  - what the robot runs with (the map, transforms, the scene graph) → **`config/realrobot/`**;
  - this folder keeps only code, weights (`ckpts/`) and `cache/`.
- Another recording? **`export RECORDING=<name>`** and every command below, plus
  every other real-robot tool, follows it — nothing here needs editing. The name
  is the only thing that changes: the paths are a convention built from it
  ([details](#whats-where)). A positional argument to `build_scene_graph.sh`, or
  an explicit `--map` / `--graph` / `--keyframes`, still wins over the export.

---

## All of it in one command

```bash
bash docker/boxer/build_scene_graph.sh
```

Steps 1-3 for `lab_20260811`, on the GPU host, skipping what is already done.
**Starting from a bag?** `--bag <path>` runs the data prep first, so one command
covers a new recording end to end:

```bash
bash docker/boxer/build_scene_graph.sh --bag bags/lab_newdata --rooms lab_newdata
```

That needs ROS *and* Docker on the same host. Where they are split, run
[`realrobot/dataprep/prepare.sh --bag <path> <name>`](../../realrobot/dataprep/realrobot_dataprep.md)
in the container first, then this script on the GPU host.

`--cpu` when the card is busy, `--fuse-only` to resume after a Ctrl-C,
`--graph-only` to redo step 3 alone, `--rooms` to divide and name the rooms,
`--stride N` / `--min-confidence X` as below, and a recording name as the one
positional argument. Steps 1 and 3 are plain Python; if the GPU host has not got
`scipy`/`networkx`, the script stops after the boxes and prints step 3 to run in
the dev container.

`--graph-only` exists because step 2 is the only expensive, machine-bound part:
it wants Docker, the GPU and the weights, and takes ~40 min. Everything after it
reads the one file it leaves behind, `boxer_3dbbs_fused.csv`, so rebuilding the
graph with a different `--min-confidence`, or adding `--rooms`, is seconds:

```bash
bash docker/boxer/build_scene_graph.sh --graph-only --rooms
```

The steps below are the same thing by hand.

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
  --registration config/realrobot/map/lab_20260811/boxer_to_map.json
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
  --transform config/realrobot/map/lab_20260811/boxer_to_map.json \
  --output config/realrobot/scene_graph/lab_20260811.json
```

- [ ] Prints `N furniture, M objects`.

Add `--rooms --map` to divide the furniture into rooms and name them. **Pass the
map.** The division is then made on the floor plan and DeepSeek is asked only for
the names; without `--map` it has to guess the walls from labels alone, and it
guesses wrong (see [below](#rooms-come-from-the-map-not-the-labels)):

```bash
python3 -m docker.boxer.to_scene_graph \
  --boxes outputs/realrobot/lab_20260811/boxer/lab_20260811/boxer_3dbbs_fused.csv \
  --transform config/realrobot/map/lab_20260811/boxer_to_map.json \
  --output config/realrobot/scene_graph/lab_20260811.json \
  --rooms --map config/realrobot/map/lab_20260811/map.yaml
```

- [ ] Prints `5 rooms on lab_20260811.yaml at --room-spacing 6.0 m`, then their names.
- Every furniture piece gets a room; each object inherits its furniture's.
- Dividing the floor needs no model. Without `DEEPSEEK_API_KEY` the rooms come
  out numbered instead of named, and everything downstream still works.
- Redo it whenever step 3 is rerun.

**A different building?** `--room-spacing` is the floor distance at which two
pieces stop counting as one room. Read it off the sweep rather than guessing:

```bash
python3 -m docker.boxer.to_scene_graph --boxes ... --transform ... --output ... \
  --map config/realrobot/map/lab_20260811/map.yaml --room-sweep
```

```text
    2 rooms     8.4 ..  14.9 m      <- ignore: the building split in half
    6 rooms     4.2 ..   5.7 m
    5 rooms     5.8 ..   7.1 m      <- 6.0 m sits here
```

A spacing range that holds the same room count is a real division of the floor
rather than an artefact of the number; take one from the middle of a wide run.
The run at the top is always the whole building falling into two halves.
- Verify, without a GUI:

  ```bash
  python3 - <<'PY'
  import collections, json
  nodes = json.load(open("config/realrobot/scene_graph/lab_20260811.json"))["nodes"]
  print("furniture:", collections.Counter(n["room"] for n in nodes if not n["movable"]))
  print("no room  :", collections.Counter(n["label"] for n in nodes if n["room"] is None))
  PY
  ```

  The second line should hold only wall and ceiling things — posters, lights, a
  tv, a bulletin board. Those have no `on`/`in` edge to inherit a room through,
  so `sg.set_rooms` leaves them null. Anything floor-standing showing up there
  means its furniture edge is missing, not that the rooms are wrong.

---

## 4. Look at it

```bash
python3 visualization/scene_graph.py \
  --graph config/realrobot/scene_graph/lab_20260811.json \
  --scene outputs/realrobot/lab_20260811/keyframes/scene.ply \
  --output outputs/realrobot/lab_20260811/scene_graph.rrd
```

- [ ] Rerun opens, and the boxes sit on the grey point cloud.
- [ ] Prints `174 nodes, 112 edges in map, 5 rooms: ...`, or `no rooms
      assigned` if step 3 ran without `--rooms`.
- With rooms assigned, one hue per room replaces the hue per node, the label
  reads `12: desk [office] (0.71)`, and each room is its own branch of
  `scene/rooms/` in the entity tree — so the viewer's blueprint panel shows and
  hides a whole room at a time. Without rooms, nothing changes.
- Add `--save-only` to only write the `.rrd`.
- Top-down on the Nav2 map instead, labelled:
  `python3 realrobot/offline/plan_overlay.py --survey`. With rooms assigned it
  colours each footprint by its room and writes the room names on the map, which
  is the quickest way to see whether the clusters are right — a piece in one
  room's colour sitting inside another room's cluster is the thing to look for.
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
| Rooms | `--rooms --map <map>.yaml` in step 3; numbered without `DEEPSEEK_API_KEY`, named with it |
| Rooms split differently | `--room-spacing N` in step 3; `--room-sweep` shows what to pick |
| Redo step 3 only | `--graph-only` on `build_scene_graph.sh` |
| Quick image test | `bash docker/boxer/run_demo.sh` (upstream Aria demo; writes to `outputs/boxer/`) |

---

## Broken?

| You see | Do |
| --- | --- |
| `Permission denied` on Boxer's files | The container writes as root: `sudo chown -R $USER outputs/realrobot/lab_20260811/boxer docker/boxer/{ckpts,cache}` |
| `File exists` in step 1 | Delete that output folder and rerun |
| `Boxes span several frames` in step 3 | You passed the per-frame CSV. Use `_fused.csv` |
| CUDA out of memory | Use `BOXER_CPU=1` |
| `Failed to find C compiler` in step 2 | On the GPU, Triton builds a CUDA shim at warmup and the CUDA *runtime* base image has no compiler. Rebuild once for the `gcc python3.12-dev` layer: `BOXER_REBUILD=1 bash docker/boxer/run_boxer.sh --build`. In a hurry, `export TORCH_COMPILE_DISABLE=1` instead and OWLv2 runs eagerly |
| `no space left on device` while building | `docker image prune` |
| `Failed to open connection to X server` in step 4 | The container kept the `DISPLAY` of an older login. Run `echo $DISPLAY` on the host, then `export DISPLAY=:1` (that value) in the container. Rebuilding the container fixes it for good |

---

## What changed, 2026-09-19

| Change | Why | Where |
| --- | --- | --- |
| One definition of a blocker | The base's keep-out set was built three times from furniture alone, while `plan_overlay` also counted objects standing on the floor. The figures and the robot disagreed | `sg.blockers()` in [graph.py](../../core/scene_graph/graph.py); called by `actions._views`, `actions.make_graspable`, `explore.furniture_footprints`, `plan_overlay.blockers` |
| Floor-standing objects now block at runtime | Bins, boxes and computer towers are static and the 240° laser has an unsensed rear sector, so the graph is the only thing that knows about them ahead of a plan. 50 → 61 shapes | same |
| The target is excluded from its own keep-out | A 0.39 m bin grows to a 0.495 m keep-out against `PARK_DISTANCE` 0.45, so it ruled out every base pose that could reach it. Measured: 0 of 24 park poses survived, 8 with it excluded | `exclude=` on `sg.blockers` |
| `person` never blocks | Two people from the 2026-08-11 recording became permanent no-go zones. Their boxes span the 0.19 m scan height, so Nav2's obstacle layer marks them live and clears them when they leave; the recording can only go stale | `TRANSIENT` in [graph.py](../../core/scene_graph/graph.py) |
| `--rooms` on step 3 | Room grouping already existed for the Gazebo graph and was never wired to this pipeline, so every node read `"room": null` | [to_scene_graph.py](to_scene_graph.py), [build_scene_graph.sh](build_scene_graph.sh) |
| Rooms visible in Rerun | One hue and one entity-tree branch per room; the run also prints the room names | [scene_graph.py](../../visualization/scene_graph.py) |
| Rooms on the survey map | `--survey` colours furniture by room and names each one, so a bad cluster is visible against the map rather than only in the JSON | [plan_overlay.py](../../realrobot/offline/plan_overlay.py) |
| Rooms divided on the map | The reasoner never saw the walls, so it split one desk row into `office` + `dining` and merged a corridor-away room into `office`. The division now comes from the floor plan; the model only names the result | [rooms.py](../../core/scene_graph/rooms.py), `deepseek.name` |
| One switch per recording | Ten tools each spelled out their own path to `lab_20260811`, so a new bag meant finding all of them. `export RECORDING=<name>` now moves the lot | [recording.py](../../core/utils/recording.py) |
| A bag to a scene graph in one command | Data prep was two hand-typed commands with the bag and every path spelled out | [prepare.sh](../../realrobot/dataprep/prepare.sh), `build_scene_graph.sh --bag` |

Furniture is unaffected by all of this: it blocks by its fitted footprint exactly
as before. That is still the case blockers exist for — 12 of the 50 pieces never
cross the 0.19 m scan plane at all, so Nav2 would drive the base under their tops.

---

## Rooms come from the map, not the labels

`deepseek.assign` sees only furniture labels and centroids, so asking it to group
them into rooms asks it to guess where the walls are. On this lab it got two of
three wrong: it split the main desk row into `office` and `dining` (`dining` came
out as three chairs 6.5 m apart, each within 1.2 m of an office desk), and it
merged the room reachable only along the corridor into `office`. Only `kitchen`
was right, and that is the one room the labels give away.

So the two halves are now separate steps:

| Step | Decided by | Needs |
| --- | --- | --- |
| Which furniture is in a room together | [`rooms.partition`](../../core/scene_graph/rooms.py) — the map | nothing |
| What that room is called | `deepseek.name` | `DEEPSEEK_API_KEY` |

`rooms.partition` measures how far each pair of furniture is **along the floor
the robot actually drove**, not in a straight line, and clusters on that. A wall
between two desks makes that distance long even when they are a metre apart.
Three things make it hold up on any map rather than this one:

- **Clutter is not structure.** Obstacle blobs shorter than `WALL_SPAN` (1 m on
  the longest side) are table legs and bins, which stand *inside* a room; only
  longer blobs divide one. Without this the lab's free space never gets more than
  1.19 m of clearance and classic distance-transform room segmentation collapses.
- **Unknown is not floor.** This map is 87% unknown — it covers the route, not
  the building. Treating unknown as walkable lets the measurement step straight
  through a wall into the next room.
- **No enclosed rooms required.** Nothing is flood-filled or hulled, so a partial
  map, an open-plan floor and an L-shaped room all behave.

Check the result against the map with
`python3 realrobot/offline/plan_overlay.py --survey`: each room should be one
contiguous patch of colour. A piece in one room's colour sitting inside another
room's patch is a real error.

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
