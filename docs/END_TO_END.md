# One Command End-to-End

Example flow:

1. Ingest and run pipeline
```
bijux gnss ingest --dataset demo --out runs/demo_ingest
bijux gnss run-pipeline --dataset demo --out runs/demo_run
```

2. Inspect artifacts
```
bijux gnss artifact explain runs/demo_run/artifacts/obs.jsonl
```

3. Summarize diagnostics
```
bijux gnss diagnostics summarize runs/demo_run
```

4. Validate against reference
```
bijux gnss validate reference --run-dir runs/demo_run --reference datasets/demo_ref.jsonl
```
