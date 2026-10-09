# NeoHorse model service

The simulator uses the native `/v1/systemone` text interface from the [official NeoHorse-Jev-4B release](https://huggingface.co/TokenRhythm/NeoHorse-Jev-4B). Its runtime, tokenizer, backbone and decision head must come from the same bundle. Model weights are not committed to this repository.

## Download the complete bundle

Use the Hugging Face CLI in a separate model environment:

```bash
python3.12 -m venv .venv-model
.venv-model/bin/python -m pip install huggingface_hub
.venv-model/bin/hf download TokenRhythm/NeoHorse-Jev-4B \
  --local-dir ./models/NeoHorse-Jev-4B
export MODEL_DIR="$PWD/models/NeoHorse-Jev-4B"
```

The download is large and external to the source repository. You can instead point `MODEL_DIR` at an existing complete release.

## Install its runtime

On a Linux machine with a BF16-capable CUDA GPU, install the bundled native runtime and HTTP dependencies into the model environment:

```bash
.venv-model/bin/python -m pip install \
  "$MODEL_DIR/dist/neohorse_decision-1.0.0-py3-none-any.whl[serve]"
```

The bundle declares its Torch, Transformers and HTTP dependencies. For backend kernels, driver compatibility and an already provisioned ML environment, follow the release's [deployment guide](https://huggingface.co/TokenRhythm/NeoHorse-Jev-4B/blob/main/DEPLOYMENT.md). Do not combine the model environment with the simulation environment or mix packages from unrelated inference releases.

## Start

```bash
MODEL_DIR="$MODEL_DIR" CUDA_VISIBLE_DEVICES=0 bash run_model.sh
```

Defaults: loopback address, port 8000, device `cuda`, CLI `.venv-model/bin/neohorse-decision`. Override `MODEL_PORT`, `MODEL_DEVICE` or `MODEL_BIN` when needed. `run_model.sh` runs in the foreground so the model process remains visible and can be stopped with `Ctrl+C`.

Use `JEV_ENDPOINT=http://127.0.0.1:8000/v1/systemone` in the simulation terminal. To enable the native service's Bearer authentication, set `NEOHORSE_API_KEY` when launching the model and the same value as `JEV_API_KEY` when launching the simulator. Store secrets only in your ignored local configuration.

## Interface contract

The Jev client sends this structure (state values below are abbreviated, not a real sensor sample):

```json
{
  "model": "NeoHorse-Jev-4B",
  "state": {"target_from_camera": {}, "own": {}, "navigation_from_camera": {}},
  "questions": {
    "action": {
      "type": "choice",
      "instructions": {"goal": "…", "rules": "…", "effects": "…"},
      "criteria": {"turn_left": "…", "turn_right": "…", "climb": "…", "descend": "…", "forward": "…", "fire": "…"}
    }
  }
}
```

It expects `answers.action.choice` and `answers.action.probabilities`. A choice must be one of the six action keys; an explicit null choice is recorded as an abstention. No image field is sent. See [sim/duel_tactics.py](../sim/duel_tactics.py) for the complete client.

The runtime's `confidence` is a distribution statistic. It is not an independently calibrated probability that the maneuver will succeed, and it is not used to replace the model's selected action.
