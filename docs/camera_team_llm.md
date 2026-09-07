# Camera-based team execution with Gemini decisions

The user authorized two steps: camera-only destination transfer, then three simultaneous independent robots. The user subsequently clarified that an LLM must make the decisions. The current Gemini path therefore differs from the earlier deterministic execution demonstrations.

Each robot has a separate `GeminiTransportPlanner`, local action/feedback memory, and asynchronous model future. The planner receives current own wrist and fixed chassis RGB JPEGs, plus its previous own chassis JPEG after the first successful model response, their frame metadata, its own task identity/destination label, and its own execution state. It receives neither seeded coordinates nor referee state, peer memories, or rule-planner recommendations.

Gemini selects approach, pick, forward/turn commands, wait, release, grip verification, and finish. Approach and manipulation are camera-based closed-loop primitives; they do not select transport routes. The robot stops after approach and waits for Gemini to choose pick. Once holding the box, every navigation macro comes from Gemini. Release and finish also require model decisions. Invalid responses are logged and rejected; there is no silent deterministic planner fallback.

The own-RGB drive guardian may veto a dangerous drive. It does not generate a replacement direction. Visual grip uncertainty stops movement and returns to Gemini, which can authorize a physical left/right/home arm probe; no thresholds are relaxed to manufacture success. All executors share one physical clock but run independently rather than waiting for a common team round.

## Fixture and scope

The `camera_team` fixture keeps the 0.82 m square zones and preexisting seeded corridor barriers. Three small boxes have known manufactured markers (14/15/16); box 02 retains its distinct original dimensions. One box starts in each zone; task assignment is fixed cyclic transfer, with seeded box/zone permutation. Fixed assignments establish independent execution, not autonomous role negotiation or evidence of communication benefit.

A fixed forward RGB camera and visible magenta peer bands are added to each simulated robot. The wrist camera is largely occluded by a held box, so the second camera provides navigation visibility. The chassis camera's physical body mount is (0.05, 0, 0.32) m, with nominal floor height 0.3525 m and calibrated pinhole projection. This added hardware has not been validated on the real robot.

Gemini is accessed through the existing local subscription proxy. Audit logs separately record requested and provider-reported model identity, usage, exact original image hashes, raw response, and parsed decisions. A model name alone does not establish a successful task. Messages between robots remain disabled in this step; displayed Korean decision reasons are not conversations.

On 2026-09-06, the user selected Gemini 3.8 Flash. The camera-team runner now defaults to `gemini-3.8-flash`; other historical runners and artifacts retain their original model configuration. The provider-reported model was `gemini-3.8-flash` for all three robots, with separate approach, pick, drive, and grip-check decisions. This establishes model integration, not superiority over another model.

## Artifacts and verification

`evaluate_gemini_team.py` writes actual model responses to `llm-decisions.jsonl`, primitive decisions to `control.jsonl`, raw actuator events to `commands.jsonl`, and independent grip monitoring to `visual-guard.jsonl`. `inputs/` preserves actual camera frames. `evaluation-only.jsonl` records physical state solely for offline scoring. A delivery requires physical lift, placement within the named destination, settling, no cargo attachment constraints, and the model-authorized release's visual confirmation.

The report shows failed and unfinished runs as well as successes. Video combines a spectator view with the latest exact decision images for each robot, labeled with their age. It is not presented as a continuously fresh camera feed between decisions. Contact penetration and overlapping robot/cargo motion are measured separately from individual delivery success.

Example commands (fresh output directories required):

```bash
/opt/anaconda3/bin/python -m scripts.evaluate_gemini_team \
  --output outputs/warehouse_research/my-gemini-cohort/team-41 \
  --seed 41 --robots 3 --seconds 600 --max-calls 120 \
  --impratio 10 --noslip-iterations 3 --record
/opt/anaconda3/bin/python -m scripts.render_visual_team_report \
  outputs/warehouse_research/my-gemini-cohort
```

The explicit contact solver settings are experiment configuration, not physical force or friction changes. Multi-seed task completion and real-hardware transfer must be reported only after actual verification.

## Placement and recovery correction

The first actual 3.8 seed-41 run completed with 0/3 physical deliveries: R1 and R2 released outside their destinations; R3 reached its 120-call limit. Physical replay reproduced all final cargo coordinates exactly. R3 had sustained contact with a legacy yellow demonstration block, not a requested corridor barrier.

Placement evidence now reconstructs the known 0.82 m square from sufficiently supported visible RGB floor boundaries, with calibrated ground projection and a 50 mm footprint margin. It never reads a simulator object pose. Before release, established tracked-grasp identity may survive marker occlusion through the existing visual grip checks. After release, a fresh marker observation is required. Missing geometric support yields uncertainty. Release and finish require fresh inside evidence, and the final offline referee still independently measures physical placement.

Only Gemini may request a reapproach after release; this creates a fresh manipulation tracker and clears stale attachment state, with at most three recovery attempts. On the original R2 released camera pair, the new sensor check reported outside and a fresh actual Gemini 3.8 response chose approach instead of finish.

## Final three-seed cohort and separate recovery probe

The final normal cohort used seeds 41, 58, and 73, three independent robots per seed, with identical 600 simulated-second and 180-call-per-robot limits. All three runs used source hash `8cb3389821bdb89f7c5730a52a5875f9054692ed60ed817c24fcf8cd9ecad48e`. Six of nine assigned deliveries succeeded: seed 41 completed 1/3, seed 58 completed 3/3, and seed 73 completed 2/3. Thus one of three complete teams succeeded. The three failed robot outcomes comprise two visual-grip failures and one proxy-connection failure whose finer subtype is unknown. These failures remain in the denominator.

The cohort used three independent Gemini planners and no robot-to-robot communication. It therefore measures camera-based independent execution, not a communication benefit. The focused camera/LLM/recovery checks passed 51 tests, and the previously run platform checks passed 48 tests; these are separate verification sets rather than a new whole-repository result.

The near-field recovery correction was implemented only after the normal cohort processes had already imported the cohort source. It is not part of the source hash above, and no normal-cohort delivery used it. In the independent `recovery-probe-03`, current physics replayed the old R2 raw commands to the released checkpoint without pose injection. Fresh own-camera evidence reported outside; two fresh provider-reported Gemini 3.8 calls selected approach and then pick. The unchanged strict attachment checks passed and the cargo rose 6.877 cm. This confirms physical re-grasp, but the probe stopped in carrying state and does not establish a completed re-delivery. The probe is explicitly a newer-source diagnostic and is not added to the normal cohort's 6/9 score.

## Camera repair validation (2026-09-06, in progress)

The next revision adds classified bounded transient inference recovery, fresh RGB re-observation after backoff, full wrist-pan sweep plus strict home attachment evidence, bounded own-image progress history, and optional status/natural message delivery. Messages carry observation times, expire after 30 simulated seconds, and do not inject simulator positions. Only validated model-authored broadcasts are displayed as communication; private model reasons remain separate.

The injected-timeout diagnostic `camera-retry-probe-01` verified a subsequent fresh camera request and accepted actual Gemini response. It is not a transport-success run. The development cohort `coela-camera-repaired-01` also encountered actual initial connection errors and resumed subsequent model requests. Its three seed-41 runs retain their originally imported revision and are not pooled with a later revision. Its planned but unstarted seed-89/97 conditions are explicitly superseded in `cohort-status.json`; the original manifest remains intact.

The full redelivery diagnostic `recovery-redelivery-01` failed to release within 20 calls despite lifting and entering the destination. Missing visible boundary support conservatively blocked release. In `recovery-redelivery-02`, a two-visible-edge calibrated fallback allowed fresh inside checks: 16 actual Gemini calls chose approach, pick, 12 drives, release, and finish. Offline rotated-footprint, lift, stability, no-constraint and visual-release gates all passed. This diagnostic reconstructs a recorded physical prefix in the current physics, leaves other robots stationary, and freezes physics during its synchronous inference calls. It does not establish asynchronous team reliability. Both attempts and videos are preserved. The two-edge geometric fallback remains under final conservative-fit review before the matched cohort is frozen.

## Final matched attempt and provider block

`coela-camera-repaired-02` ran all nine declared seed/mode cells under identical source hash `825e9ba1a9da4c77d04cded3aa522befd287c175b34f4fa7b9352bc8a35fbceb`. All nine process outputs and videos exist, but all 27 deliveries ended with `LLM_ERROR:http`: sustained HTTP 429 began during seed 41, and seeds 89/97 received no successful decisions. Preserve the end-to-end 0/9 teams and 0/27 deliveries, label the shared infrastructure failure, and do not infer controller or communication quality from these outcomes. Seed 41 produced 347 accepted model decisions and broadcasts none=0, status=41, natural=34 before interruption. All available camera/message/source/video integrity checks pass; actual response-model evidence is necessarily absent for the six no-response runs.

A subsequent single minimal real Gemini request still returned 429 without Retry-After. The reset time is unknown. No model substitution, proxy restart, account change or quota workaround was performed. Further physical multi-seed verification is blocked by provider availability.

After this cohort closed, the recovery scheduler gained 30/60/120-second bounded rate-limit cooldowns and safe Retry-After support; these are not retroactively part of the saved cohort. `scripts.evaluate_gemini_cohort` now stops scheduling remaining trials after a terminal inference failure even if the simulation process returned zero. Ordinary physical-task failures remain in the comparison and do not abort it. Default trial concurrency is one; each trial still contains three asynchronous Gemini-controlled robots. This avoids multiplying shared account load by running several worlds. The focused current verification suite passed 108 tests before the final report pending-mode correction.

Once the same provider is available, start a NEW output directory:

```bash
/opt/anaconda3/bin/python -m scripts.evaluate_gemini_cohort \
  --output outputs/warehouse_research/coela-camera-repaired-03 \
  --seeds 41 89 97 --jobs 1 --seconds 600 --max-calls 180
```

Do not replace the failed 02 outputs. Validate whole-team delivery, no physical penetration, own-camera/message boundary, actual model responses, and visible playback before claiming the repair works across seeds. Navigation optimality remains unproven.


## Compact inputs and bounded execution (2026-09-06)

The planner now sends a <=6000-character semantic context instead of repeated detailed geometry. It preserves the latest camera-derived placement status/stage/identity, recent three actions and critical feedback. Fresh wrist and nav images remain original; previous nav is conditional on stagnation/comparison evidence. Actual transmitted context and inbox are logged.

Gemini may select drive durations up to4seconds. The executor uses unchanged velocities in <=0.25second slices, each requiring fresh own RGB safety approval. New messages, RGB stagnation, obstacle veto and grip uncertainty stop execution; only Gemini selects the next direction. No hidden route/controller replacement is introduced. Default call ceiling40 and reportedinput ceiling120000 perrobot constrain further trials; inputusage may overshootbyone finalresponse.

Offline memory comparison across422savedrequests reduced memorycharacters95.24%, retaining latestplacement in404eligiblecases. A noAPI physical executor diagnostic performed the same4secondrotation/16RGBchecks using4shortdecisions versus1boundeddecision. These are component evidence, not task or quota efficiency claims.

Actual Gemini solo41 pilot used15calls and61898inputtokens before the explicit60000inputbudget stopped it. Pickupandcarrying succeeded, butdeliveryfailed. Model selected1.5seconddrives; full-task call savings remain unverified. See coela-event-pilot-01 for videoandverification. Do not scale toteam runs on this evidence.
