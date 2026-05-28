---
name: opencode-deepseek-delegation
description: Delegate bounded repository tasks to the local `opencode` CLI with a user-confirmed model, then verify and accept locally. Use for low-risk analysis, doc rewrites, checklist/inventory work, and mechanical edits that are easy to validate.
---

# OpenCode Delegation (Generic)

## Overview

Use local `opencode` as a bounded worker and keep final acceptance in Codex.
This skill is model-agnostic and requires model confirmation before execution.

## Choose the Right Tasks

Delegate only work that is narrow, low-risk, and easy to verify locally.

- Read-only code or document analysis
- Markdown rewrites or structured summaries
- Grep, inventory, checklist, or naming cleanup work
- Mechanical edits limited to one or two clearly named files
- Drafting a first-pass explanation that Codex will review and compress

Keep the task local when it is high-risk, blocking, or hard to verify quickly.

- Destructive operations, git history edits, or credential handling
- Architecture changes across many files
- Debugging that depends on tight back-and-forth with the current agent state
- Final code review, acceptance, or test interpretation
- Any task where a bad answer would be expensive to miss

## Mandatory Model Confirmation

Before launching any delegated task, confirm the model with the user in one short line:

- `本次外包使用模型：<provider/model>，是否按这个执行？`

Rules:

- If the user explicitly names a model, use it as-is.
- If not specified, recommend `deepseek/deepseek-v4-flash` as default.
- Do not dispatch jobs before explicit confirmation.

## Verify Runtime First

Confirm `opencode` can run the selected model.

- Run `opencode models <provider>`
- If needed, run `opencode providers list`
- Do not assume static config fully reflects runtime provider state

Quick smoke test:

```powershell
opencode run --model <provider/model> "Reply with exactly OK and nothing else."
```

Accept `OK` as runtime-ready.

## Prepare a Delegation Prompt

Make each subtask self-contained and explicit.

Include:

- The repository or working directory
- The concrete task
- The allowed file scope
- The required output format
- A rule to avoid follow-up questions unless truly blocked

Use this Windows PowerShell pattern for multi-line prompts:

```powershell
$prompt = @'
You are a narrow worker inside repo D:\Desktop\ch32\0.ch32v208_dome\2.4G_RF.
Task: Inspect xmake.lua and list stale source/include paths.
Constraints:
- Read only files needed for this task.
- Do not ask follow-up questions.
- Do not edit files.
Output:
- 1 short summary
- 1 bullet per stale path
- If blocked, start the last line with BLOCKED:
'@

opencode run --model <provider/model> $prompt
```

When edits are allowed, add an explicit contract such as:

- `Edit only these files: ...`
- `Do not touch unrelated files`
- `Return the changed files and a short validation note`
- `Do not output chain-of-thought or long reasoning`
- `If you encounter a small issue inside allowed scope, repair it first and continue`
- `Return a concise final success/failure conclusion at the end`
- `If you encounter a small issue within the allowed scope, repair it yourself and continue`

## Chinese Output Requirement

For Chinese workflow consistency, require Chinese result files.

- Output language in done files: Simplified Chinese
- Completion marker location: `logs/opencode/done/`
- File naming:
- `<job_id>.success.txt`
- `<job_id>.failure.txt`

Each result file should include:

- `任务结论` (成功/失败)
- `完成项` (1-5 concise lines)
- `失败项或风险` (if any)
- `涉及文件` (changed/read files)
- `验证命令与结果` (short)

## Background Workflow

In this repository, prefer fire-and-forget background delegation instead of periodic polling.

Use:

- `scripts/start_opencode_job.ps1`
- `scripts/opencode_job_runner.ps1`
- `logs/opencode/jobs/`
- `logs/opencode/done/`

The default operating model is:

1. Write the prompt into a file.
2. Start the job in the background with `scripts/start_opencode_job.ps1`.
3. Do not keep checking it from Codex.
4. The wrapper writes a completion marker into `logs/opencode/done/`.
5. The user watches for that marker.
6. Only after the user says the job is done should Codex read the result file and continue.

This keeps Codex free to continue local work without spending tokens on waiting or polling.

Example:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\start_opencode_job.ps1 `
  -JobName "ulog-readme" `
  -PromptFile .\temp\ulog-readme-prompt.txt `
  -Model deepseek/deepseek-v4-flash
```

Expected output:

- `job_id=...`
- `job_dir=...`
- `done_dir=...`
- `pid=...`

Completion is signaled by either:

- `logs/opencode/done/<job_id>.success.txt`
- `logs/opencode/done/<job_id>.failure.txt`

## Self-Repair Rule

Always include this condition in delegated tasks:

- If the worker finds a small, local, fixable issue inside allowed scope, it should repair first.
- If a local validation fails due to its own edit, it should do one bounded repair pass.
- Report `BLOCKED` only when scope/inputs are insufficient or repair attempts fail.

## Run and Accept

After `opencode` returns, keep acceptance local.

1. Read the output critically.
2. Inspect the diff if files changed.
3. Run the real local verification yourself.
4. Accept only after the local result matches the claim.
5. Re-prompt with tighter scope if the first pass is vague or drifts.

Use repository-native validation, not only DeepSeek's self-report.

- Build with the local build system
- Run the relevant script or test command
- Re-open the touched files and inspect the exact lines
- Compare the answer against the actual repository state

## Read Results, Not Thoughts

- Read final conclusion, touched files, validation outputs, and blockers first.
- Ignore long planning text unless a failure requires diagnosis.
- Use result deltas to drive the next delegated subtask.

## Keep the Prompt Tight

Use short, concrete delegation requests.

- `Read docs/protocol-state-flow.md and summarize the gateway-to-node flow in 5 bullets. Do not edit files.`
- `Inspect xmake.lua and list missing file references that would cause stale warnings. Do not propose unrelated cleanup.`
- `Edit only docs/design.md to normalize one wording pattern. Return the changed heading names and a 3-bullet summary.`

Avoid broad prompts like `analyze the whole repo` or `fix everything`.

## Operating Rules

- Keep secrets out of the prompt body.
- Prefer one bounded subtask per `opencode run`.
- Treat worker output as a candidate result, not the final truth.
- Keep final user-facing judgment, integration, and verification in Codex.
- In this repository, do not spend tokens polling background `opencode` jobs.
- Read `logs/opencode/done/*.txt` only after the user reports completion.
