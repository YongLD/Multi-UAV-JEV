import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from replay_store import retain_recording, retain_single_replay, list_replays, replay_path, import_history


class ReplayStoreTests(unittest.TestCase):
    def record(self, live, hits, seed, saved, count=2):
        pending=live/f'pending_{seed}'
        pending.mkdir()
        for name in ['all.mp4']+[f'drone_{i}.mp4' for i in range(count)]:
            (pending/name).write_bytes(f'episode-{seed}-{name}'.encode())
        summary={'hits':hits,'seed':seed,'fleet_count':count}
        with patch('replay_store.time.time',return_value=saved):
            retained=retain_recording(live,pending,summary,views=[
                {'id':f'P-{i+1:02d}','label':f'P-{i+1:02d}','file':f'drone_{i}.mp4'} for i in range(count)])
        return retained, summary

    def test_keeps_top_two_per_group_and_rejects_third(self):
        with tempfile.TemporaryDirectory() as directory:
            live=Path(directory)
            for seed,hits in enumerate((12,28,17),1):
                self.assertTrue(self.record(live,hits,seed,100+seed)[0])
            self.assertEqual([e['hits'] for e in list_replays(live)],[28,17])
            before=(live/'replay_catalog.json').read_bytes()
            self.assertFalse(self.record(live,6,4,104)[0])
            self.assertEqual((live/'replay_catalog.json').read_bytes(),before)
            self.assertTrue(self.record(live,30,5,105)[0])
            entries=list_replays(live)
            self.assertEqual([e['hits'] for e in entries],[30,28])
            for entry in entries:
                self.assertTrue(replay_path(live,entry).is_file())
                self.assertEqual(len(entry['replay_views']),2)
                for view in entry['replay_views']:
                    self.assertTrue((replay_path(live,entry).parent/Path(view['url']).name).is_file())
            self.assertIn(b'episode-5',(live/'replay.mp4').read_bytes())
            self.assertEqual(json.loads((live/'replay_summary.json').read_text())['hits'],30)

    def test_equal_scores_prefer_newer_and_catalog_survives_reload(self):
        with tempfile.TemporaryDirectory() as directory:
            live=Path(directory)
            for seed in range(1,5):self.record(live,15,seed,100+seed)
            self.assertEqual([e['seed'] for e in list_replays(live)],[4,3])
            self.assertEqual(len(json.loads((live/'replay_catalog.json').read_text())['replays']),2)

    def test_legacy_video_migrates_and_single_and_multi_keep_separate_quotas(self):
        with tempfile.TemporaryDirectory() as directory:
            live=Path(directory)
            (live/'replay.mp4').write_bytes(b'legacy-real-video')
            (live/'replay_summary.json').write_text(json.dumps({'hits':18,'seed':1,'fleet_count':1}))
            import_history(live)
            first=list_replays(live)[0]
            self.assertEqual(replay_path(live,first).read_bytes(),b'legacy-real-video')
            self.record(live,12,2,200)
            video=live/'single_next.mp4';video.write_bytes(b'new-single-video')
            self.assertTrue(retain_single_replay(live,video,{'hits':24,'seed':3,'fleet_count':1}))
            self.assertEqual([e['hits'] for e in list_replays(live)],[24,18,12])
            self.assertEqual(replay_path(live,first).read_bytes(),b'legacy-real-video')

    def test_all_three_groups_retain_two_independently_and_history_recovers_evicted_sets(self):
        with tempfile.TemporaryDirectory() as directory:
            live=Path(directory)
            for count in (1,2,4):
                for offset,hits in enumerate((7,8,9)):
                    self.record(live,hits,count*100+offset,100+offset,count=count)
            catalog=json.loads((live/'replay_catalog.json').read_text())
            self.assertEqual(len(catalog['replays']),6)
            self.assertEqual(catalog['total_limit'],6)
            for group in catalog['groups'].values():self.assertEqual([e['hits'] for e in group],[9,8])
            # Simulate the old global catalog that hid other groups; recover
            # existing immutable videos instead of inventing new recordings.
            catalog['replays']=catalog['groups']['1']
            (live/'replay_catalog.json').write_text(json.dumps(catalog))
            import_history(live)
            self.assertEqual(len(list_replays(live)),6)

    def test_incomplete_candidate_cannot_replace_existing_catalog(self):
        with tempfile.TemporaryDirectory() as directory:
            live=Path(directory)
            self.record(live,12,1,101)
            previous=(live/'replay_catalog.json').read_bytes()
            pending=live/'incomplete';pending.mkdir();(pending/'all.mp4').write_bytes(b'video')
            with self.assertRaises(RuntimeError):
                retain_recording(live,pending,{'hits':30,'fleet_count':2},views=[{'id':'P-01','label':'P-01','file':'drone_0.mp4'}])
            self.assertEqual((live/'replay_catalog.json').read_bytes(),previous)


if __name__=='__main__':unittest.main()
