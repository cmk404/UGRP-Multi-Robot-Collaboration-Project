# Recorded robot dialogue

`scripts/record_mixed_warehouse.py` now captures each actual policy message via
the existing event callback. `dialogue.jsonl` stores the sender, original message,
physics time, and video-relative time. Sampling uses the physics lock; LLM wall
time is not used as a substitute for video time.

After recording, `scripts/render_warehouse_dialogue.py` produces
`mixed-dialogue-1x.mp4` with a conversation panel and a companion HTML transcript.
The transcript includes every message and seeks the video when a message is
clicked. The overlay shows recent messages only after their recorded timestamp;
no dialogue is generated for display. Playback has 0.1 second frame resolution.

Verified recording: `outputs/warehouse_research/dialogue-obstacle-01/`.
All six displayed messages match the episode's `actor_decision` messages exactly.
The 61.9 second video contains the initial split of work, both participants'
replan messages, and R3's subsequent solo claim. Three cargo tasks completed;
zero peer collision events were recorded. Frames at 8, 17, 28, and 50 seconds
were extracted for visual inspection. The obstacle is a controlled scenario
event; the messages are actual LLM output, including its own descriptions of
the situation.

To render an existing recording with a captured dialogue timeline:

```sh
python -m scripts.render_warehouse_dialogue --video RUN/mixed-solo-joint-1x.mp4 --dialogue RUN/dialogue.jsonl --output RUN/mixed-dialogue-1x.mp4
```
