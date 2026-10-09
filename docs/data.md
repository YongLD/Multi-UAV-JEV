# Episode collection and future training

`bash run.sh` continuously starts new seeds and archives **every completed episode**, including success, timeout, crash and manual restart. The default success criterion is ten valid hits. Replay selection is independent: ≥5 hits and ≥the retained best, including ties.

## Stored files

| File | Contents |
| --- | --- |
| `decisions.jsonl` | Each completed request's observable state, model choice, candidate probabilities, confidence/usage where returned, simulation capture time and measured wall latency. The episode loop adds response age, applied flag, executed action and virtual shot result, or the reason for non-execution. |
| `events.jsonl` | Shot and collision/contact events. |
| `referee.jsonl` | Evaluation truth such as actual positions and target geometry; kept separate from model state. |
| `policy.json` | Resolved instructions, candidate descriptions, action meanings and model/scene parameters. |
| `summary.json` | Outcome, duration, hits, shots, contact count, executed action counts, model calls/errors and replay eligibility. |

Text logs can grow over long runs; there is no automatic total archive quota. Move or rotate `runs/cases/` when needed. Raw sensor videos/images are not duplicated into every case folder, but `live/` keeps displayed images, a temporary current recording and one qualifying replay. Completed cases do not keep all videos. All output is ignored by Git.

`action_counts` describes **executed** choices. To calculate distributions over all model responses, count raw `decisions.jsonl` action fields and keep errors, abstentions and stale/non-applied responses as separate categories. The viewer shows the latest distribution plus whole-episode executed counts/shares; these are different statistics.

At episode shutdown, an in-flight request is allowed to finish within the client's shutdown wait. Its response is recorded with `applied=false` and `reason="episode ended"`; it does not extend flight or execute another action. An unstarted waiting observation is not a model decision.

## Inspect one case

```bash
python3 - <<'PY'
import json
from pathlib import Path
from collections import Counter
cases = sorted(Path('runs/cases').glob('*'))
if not cases:
    raise SystemExit('No completed episodes yet')
case = cases[-1]
records = [json.loads(line) for line in (case/'decisions.jsonl').read_text().splitlines()]
print(json.loads((case/'summary.json').read_text()))
print('Raw choices:', Counter(r.get('action', 'error_or_abstain') for r in records))
print('Applied choices:', Counter(r['action'] for r in records if r.get('applied')))
PY
```

## Before fine-tuning

These logs preserve the evidence required to curate training samples; they are not automatically expert demonstrations or a ready-made trainer-specific dataset.

1. Select observable `state` and the matching questions from `policy.json` as the input. Do not copy referee coordinates into a camera-only decision prompt.
2. Generate verified action labels independently—through reviewed successful segments, an explicit expert labeler or carefully defined counterfactual evaluation. An episode with ten hits can still contain bad decisions.
3. Preserve the six candidate keys and physical effects. Do not merge historical `hold`, 15° turns or different altitude steps as if they had identical semantics.
4. Keep failed cases for diagnosis, preference comparisons or a separately designed reward signal. Failure does not tell you which alternative action was correct.
5. Split by whole episode and scenario seed. Neighboring frames from the same chase must not leak across training and evaluation sets.
6. Export to the chosen training runtime's schema only after choosing a training method. No model parameter fine-tuning is performed by this repository.

Referee truth is useful for evaluating labels and outcomes, but it should remain an explicitly separate supervisory/evaluation source. Store an annotation version and provenance alongside curated labels.
