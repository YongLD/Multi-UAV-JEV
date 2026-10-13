"""Synchronized follow-camera recordings, published as an immutable replay set."""
import shutil
import io
import queue
import threading
import uuid
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw


class FleetReplayRecorder:
    def __init__(self, live_dir, count, fps):
        self.live_dir=Path(live_dir)
        self.count, self.fps, self.frames = count, fps, 0
        self.directory = Path(live_dir)/('replay_pending_'+uuid.uuid4().hex)
        self.directory.mkdir()
        self.writers = {}
        self.times = []
        self.error = None
        self.closed = False
        self.has_instruments = False
        try:
            for name in ['all']+[f'drone_{i}' for i in range(count)]:
                self.writers[name] = imageio.get_writer(
                    str(self.directory/(name+'.mp4')), format='FFMPEG', fps=fps,
                    codec='libx264', quality=7, macro_block_size=1,
                    ffmpeg_params=['-threads', '1', '-preset','veryfast','-movflags', '+faststart'])
        except Exception:
            for writer in self.writers.values(): writer.close()
            raise
        # Bounded backpressure preserves every captured frame. Encoding can run
        # alongside physics; overload slows simulation rather than dropping frames.
        self.pending = queue.Queue(maxsize=8)
        self.encoder = threading.Thread(target=self._encode, name='fleet-replay-encoder', daemon=True)
        self.encoder.start()

    def _put(self, item):
        while True:
            if self.error: raise RuntimeError('Replay encoding failed') from self.error
            try:
                self.pending.put(item, timeout=.1)
                return
            except queue.Full: pass

    def append(self, views, sim_time=None, *, instruments=None, publish_live=False):
        if self.closed: raise RuntimeError('Replay recorder is closed')
        if len(views) != self.count:
            raise ValueError('Every active drone must supply a frame at the same simulation time')
        shape=views[0].shape
        if len(shape)!=3 or shape[2]!=3 or any(view.shape!=shape for view in views):
            raise ValueError('Follow frames must have equal RGB dimensions')
        timestamp = self.frames/self.fps if sim_time is None else float(sim_time)
        if not np.isfinite(timestamp) or (self.times and timestamp<=self.times[-1]):
            raise ValueError('Replay frames require increasing simulation times')
        if instruments is not None and len(instruments)!=self.count:
            raise ValueError('Every recorded drone requires its own instrument snapshot')
        # Instrument packets contain detached scalar dictionaries and immutable
        # sensor/overview arrays. Composition runs with encoding, off the physics
        # thread, and consumes the same capture-time snapshot as the follow image.
        self._put(([np.array(view,copy=True) for view in views],instruments,publish_live))
        self.has_instruments = self.has_instruments or instruments is not None
        self.times.append(timestamp)
        self.frames += 1

    def _encode(self):
        hud=None
        try:
            while True:
                packet=self.pending.get()
                if packet is None: break
                views,instruments,publish_live=packet
                if instruments is not None:
                    from urban_hud import UrbanHud
                    if hud is None: hud=UrbanHud(1280,720)
                    views=[hud.draw(np.asarray(Image.fromarray(view).resize((1280,720),Image.Resampling.BILINEAR)),
                                    info['depth'],info['scene'],info['decision'],info['telemetry'],
                                    info.get('enemy_xy'),rgb=info['rgb'],overview=info['overview'])
                           for view,info in zip(views,instruments)]
                self._write(views,publish_live)
        except Exception as exc:
            self.error=exc
        finally:
            for writer in self.writers.values():
                try: writer.close()
                except Exception as exc: self.error=self.error or exc

    def _write(self, views, publish_live=False):
        h, w = views[0].shape[:2]
        cols = min(2, self.count)
        mosaic = np.zeros((h*((self.count+cols-1)//cols), w*cols, 3), dtype=np.uint8)
        for i, view in enumerate(views):
            if view.shape != (h, w, 3):
                raise ValueError('Follow frames must have equal RGB dimensions')
            frame = Image.fromarray(view)
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 100, 25), fill=(20, 35, 45))
            draw.text((10, 7), f'P-{i+1:02d} FOLLOW', fill='white')
            frame = np.asarray(frame)
            if publish_live:
                buffer=io.BytesIO();Image.fromarray(frame).save(buffer,format='JPEG',quality=84)
                target=self.live_dir/f'drone_{i}.jpg';temporary=target.with_suffix('.jpg.tmp')
                temporary.write_bytes(buffer.getvalue());temporary.replace(target)
            self.writers[f'drone_{i}'].append_data(frame)
            row, col = divmod(i, cols)
            mosaic[row*h:(row+1)*h, col*w:(col+1)*w] = frame
        self.writers['all'].append_data(mosaic)

    def close(self):
        if self.closed:
            if self.error: raise RuntimeError('Replay encoding failed') from self.error
            return
        self.closed=True
        try:
            if not self.error: self._put(None)
        finally: self.encoder.join()
        if self.error: raise RuntimeError('Replay encoding failed') from self.error
        from replay_store import publish
        publish(self.directory/'frame_times.json',self.times)

    def retain(self, live_dir, summary):
        from replay_store import retain_recording
        if not self.frames:
            shutil.rmtree(self.directory)
            return False
        summary.update(fleet_count=self.count,replay_fps=self.fps, replay_frames=self.frames,
                       replay_duration_s=self.frames/self.fps,
                       replay_capture='independent_simulation_clock_no_frame_drops',
                       replay_max_frame_gap_s=round(max(np.diff(self.times),default=0.),6),
                       replay_instruments=self.has_instruments,
                       replay_ui='single_drone_full_hud' if self.has_instruments else 'follow_only',
                       replay_camera_size=[1280,720] if self.has_instruments else None,
                       replay_view='all_drone_follow_cameras')
        return retain_recording(live_dir,self.directory,summary,
                                views=[{'id':f'P-{i+1:02d}','label':f'P-{i+1:02d} 跟随'+(' · 完整仪表' if self.has_instruments else ''),
                                        'file':f'drone_{i}.mp4'} for i in range(self.count)])
