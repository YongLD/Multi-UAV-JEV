# Contributing

Start with [the quick start](docs/quickstart.md) and [the observation boundary](docs/architecture.md).

For changes to tracking, the prompt or controls, report the scene seeds, policy version, model release/runtime, episode durations, hits, contacts, latency and model errors. Keep complete decision logs locally so favorable replays do not substitute for aggregate evaluation.

Keep simulation truth out of Jev's camera-derived input. Describe any new execution constraint openly. Preserve the model's selected tactical action in both logging and execution; do not silently relabel or rerank it.

Do not commit credentials, private machine addresses, model weights, virtual environments, datasets or replay videos. City assets need source and license records. Retain inherited license notices when modifying upstream code.

Minimal checks before a pull request:

```bash
python3 -m compileall -q sim scripts
bash -n setup.sh run.sh run_model.sh
bash run.sh --once --seconds 15 --seed 42
```

The last command requires a ready model endpoint and graphics backend. Inspect the completed case for model errors and verify a rendered camera/depth frame. A short smoke run checks integration, not successful tracking or obstacle avoidance.
