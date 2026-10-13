"""Keep two highest-scoring episodes per fleet size, with immutable video URLs."""
import json
import os
from pathlib import Path
import re
import shutil
import time
import uuid

HIT_GOAL = 30
REPLAY_COUNTS = (1, 2, 4)
REPLAY_LIMIT = 2  # Independent quota for each fleet size.
REPLAY_FPS = 20
MIN_REPLAY_HITS = 5
SET_URL = re.compile(r'/replay/sets/([a-f0-9]{32})/(all|drone_[0-3])\.mp4')


def publish(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
    temporary.replace(path)


def replay_path(live_dir, entry):
    url = entry.get('replay_main_url', '/replay/latest.mp4')
    match = SET_URL.fullmatch(url)
    if match:
        return Path(live_dir)/'replay_sets'/match[1]/(match[2]+'.mp4')
    if url == '/replay/latest.mp4':
        return Path(live_dir)/'replay.mp4'
    return None


def ranked(entries):
    ordered = sorted(entries, key=lambda e:(int(e['hits']),float(e.get('saved_at_epoch',0))), reverse=True)
    selected, counts = [], {count:0 for count in REPLAY_COUNTS}
    for entry in ordered:
        count = int(entry.get('fleet_count',1))
        if count in counts and counts[count] < REPLAY_LIMIT:
            selected.append(entry)
            counts[count] += 1
    return selected


def replay_catalog(entries):
    return {'limit':REPLAY_LIMIT, 'limit_per_fleet':REPLAY_LIMIT,
            'total_limit':REPLAY_LIMIT*len(REPLAY_COUNTS), 'replays':entries,
            'groups':{str(count):[e for e in entries if e.get('fleet_count',1)==count]
                      for count in REPLAY_COUNTS}}


def valid_entry(live_dir, entry):
    try:
        path = replay_path(live_dir, entry)
        return int(entry['hits']) >= MIN_REPLAY_HITS and path is not None and path.is_file()
    except (TypeError, ValueError, KeyError):
        return False


def list_replays(live_dir):
    live_dir = Path(live_dir)
    try:
        entries = json.loads((live_dir/'replay_catalog.json').read_text())['replays']
    except (OSError, ValueError, KeyError):
        try:
            entries = [json.loads((live_dir/'replay_summary.json').read_text())]
        except (OSError, ValueError):
            entries = []
    result = []
    for entry in entries:
        if not valid_entry(live_dir, entry):
            continue
        path = replay_path(live_dir, entry)
        match = SET_URL.fullmatch(entry.get('replay_main_url',''))
        result.append({**entry,'replay_id':entry.get('replay_id',match[1] if match else 'legacy'),
                       'saved_at_epoch':entry.get('saved_at_epoch',path.stat().st_mtime)})
    return ranked(result)


def save_catalog(live_dir, entries):
    live_dir = Path(live_dir)
    root = live_dir/'replay_sets'
    root.mkdir(exist_ok=True)
    previous = list_replays(live_dir)
    materialized = []
    for entry in ranked(entries):
        entry = dict(entry)
        if entry['replay_id'] == 'legacy':
            identity = uuid.uuid4().hex
            folder = root/identity
            folder.mkdir()
            (folder/'all.mp4').hardlink_to(live_dir/'replay.mp4')
            entry.update(replay_id=identity,replay_main_url=f'/replay/sets/{identity}/all.mp4')
        folder = replay_path(live_dir, entry).parent
        publish(folder/'summary.json', entry)
        materialized.append(entry)
    # The catalog is authoritative; old clients continue to receive its first entry.
    publish(live_dir/'replay_catalog.json', replay_catalog(materialized))
    if materialized:
        temporary = live_dir/'replay.mp4.new'
        temporary.unlink(missing_ok=True)
        temporary.hardlink_to(replay_path(live_dir,materialized[0]))
        temporary.replace(live_dir/'replay.mp4')
        publish(live_dir/'replay_summary.json', materialized[0])
    retained = {entry['replay_id'] for entry in materialized}
    for entry in previous:
        if entry['replay_id'] not in retained and entry['replay_id'] != 'legacy':
            folder = root/entry['replay_id']
            if folder.is_dir():
                os.utime(folder, None)  # One-hour grace period begins at eviction.
    for folder in root.iterdir():
        if re.fullmatch(r'[a-f0-9]{32}', folder.name) and folder.name not in retained and folder.is_dir() and time.time()-folder.stat().st_mtime > 3600:
            shutil.rmtree(folder)
    return materialized


def import_history(live_dir, summaries=()):
    entries = {e['replay_id']:e for e in list_replays(live_dir)}
    recovered = []
    for path in (Path(live_dir)/'replay_sets').glob('*/summary.json'):
        try: recovered.append(json.loads(path.read_text()))
        except (OSError,ValueError): pass
    for summary in [*summaries,*recovered]:
        match = SET_URL.fullmatch(summary.get('replay_main_url',''))
        if match and valid_entry(live_dir, summary):
            entries.setdefault(match[1],{**summary,'replay_id':match[1],
                                        'saved_at_epoch':replay_path(live_dir,summary).stat().st_mtime})
    return save_catalog(live_dir, ranked(entries.values()))


def retain_recording(live_dir, directory, summary, *, views=()):
    live_dir, directory = Path(live_dir), Path(directory)
    old = list_replays(live_dir)
    identity = uuid.uuid4().hex
    entry = {**summary,'hit_goal':HIT_GOAL,'replay_id':identity,'saved_at_epoch':time.time(),
             'replay_main_url':f'/replay/sets/{identity}/all.mp4',
             'replay_views':[{'id':v['id'],'label':v['label'],
                             'url':f'/replay/sets/{identity}/{v["file"]}'} for v in views]}
    selected = ranked([*old,entry]) if summary['hits'] >= MIN_REPLAY_HITS else old
    if not any(e['replay_id']==identity for e in selected):
        shutil.rmtree(directory)
        return False
    for name in ['all.mp4']+[v['file'] for v in views]:
        if not (directory/name).is_file() or (directory/name).stat().st_size == 0:
            raise RuntimeError('Incomplete replay set; previous catalog preserved')
    root = live_dir/'replay_sets'
    root.mkdir(exist_ok=True)
    directory.replace(root/identity)
    save_catalog(live_dir, selected)
    summary.update(entry)
    return True


def retain_single_replay(live_dir, video_tmp, summary):
    directory = Path(live_dir)/('replay_pending_'+uuid.uuid4().hex)
    directory.mkdir()
    Path(video_tmp).replace(directory/'all.mp4')
    return retain_recording(live_dir,directory,summary)
