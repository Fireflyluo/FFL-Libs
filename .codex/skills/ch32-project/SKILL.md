---
name: ch32-project
description: Repository-specific workflow for this CH584 2.4G RF template project. Use when working in this repository to build, flash, clean, monitor serial debug output, or analyze the firmware according to local conventions in `.vscode/tasks.json`, `scripts/serial_reader.py`, and repository-specific output paths and OpenOCD settings.
---

# CH58x Project

## Overview

Follow the repository's own workflow instead of inventing a generic one.
Treat `.vscode/tasks.json` as the source of truth for build, flash, and clean behavior.

## Core Workflow

When the user asks to build, flash, clean, or monitor logs, mirror the repository conventions.

- `build`: Run `xmake f -P . -m debug --wch_gcc_ver=12 --ch58x_chip=584`, then `xmake -P .`.
- `flash`: Run the OpenOCD command defined in `.vscode/tasks.json` and program `build/cross/riscv/debug/CH584_1toN_module_template.elf`.
- `clean`: Follow the local clean task behavior: `xmake c`, then remove `build` and `.xmake`.
- Always run `build` and `flash` sequentially: wait for `xmake` to finish successfully before starting OpenOCD.
- Do not run build and flash in parallel; this can program stale artifacts and trigger intermittent verify failures.

Prefer the local task meaning even if another build system also exists in the repository.
This repository contains both `xmake.lua` and `CMakeLists.txt`, but the default operational workflow is the VS Code task set backed by `xmake`.

For serial debug on a connected board:

- Single read smoke test: `python scripts/serial_reader.py --port COM8 --baud 115200 --encoding utf-8 --once`
- Continuous monitor: `python scripts/serial_reader.py --port COM8 --baud 115200 --encoding utf-8`
- Timed monitor (recommended to avoid port leaks): `python scripts/serial_reader.py --port COM8 --baud 115200 --encoding utf-8 --duration 8`
- For non-UTF8 payload experiments: switch `--encoding` (for example `gbk`)

Use `COM8` and `115200` as defaults unless the user specifies a different port or baud rate.

## Task Mapping

Read `.vscode/tasks.json` before assuming the commands have changed.
At the time this skill was created, the task mapping was:

- `build`
  Command: `xmake f -P . -m debug --wch_gcc_ver=12 --ch58x_chip=584`
- `compile`
  Command: `xmake -P .`
- `flash`
  Command: `E:/APP/MRS2/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin/openocd.exe`
  Arguments:
  `-s .`
  `-f tools/wch-interface.cfg`
  `-c "program build/cross/riscv/debug/CH584_1toN_module_template.elf verify"`
  `-c "reset run"`
  `-c "exit"`
- `clean`
  Behavior: print clean messages, run `xmake c`, then remove `build` and `.xmake`

If `.vscode/tasks.json` changes later, prefer the file over this summary.

## Analysis Conventions

When analyzing this repository, treat the following as non-core/generated or build-like content unless the user explicitly asks for them:

- `build/`
- `.xmake/`
- `.cache/`
- `project/obj/`

Focus analysis on:

- `app/`
- `bsp/`
- `lib/`
- `sdk/`
- `utils/`
- `scripts/`
- top-level build definitions such as `xmake.lua`, `CMakeLists.txt`, and `.vscode/tasks.json`

## Project Facts

Use these facts as working context for repository-specific help:

- Main MCU family: `CH584/CH585`
- Architecture/toolchain target: `riscv-wch-elf-`, `rv32imc_zba_zbb_zbc_zbs_xw`
- Main app entry: `app/RF_main.c`
- Primary firmware output: `build/cross/riscv/debug/CH584_1toN_module_template.elf`
- Flash configuration file: `tools/wch-interface.cfg`
- Serial debug script: `scripts/serial_reader.py` (default `COM8`, `115200`)

## Operating Rules

- Use repository-relative paths from the workspace root.
- Prefer the task-defined workflow over ad hoc command variants.
- Treat UTF-8 as the repository text encoding standard.
- Report the important output back to the user after running build/flash/clean because terminal output is not directly visible to them.
- When reading serial logs, provide a short, readable summary of the captured output.
- If a hardware-dependent flash step fails, report whether the failure looks like connection, OpenOCD, or artifact-path related.
- When the user asks for engineering analysis, distinguish between code that is compiled into the target and code that is actually exercised by `app/main.c`.

## Delegation-Aware Planning

When a task is large, automatically assess whether it should be split into outsource-ready packets for `opencode`.

Trigger conditions for splitting:

- Estimated duration > 20 minutes
- More than 2 independent modules/files
- Large doc/test matrix work that can be validated mechanically
- Repetitive mechanical edits

If triggered, provide a short split proposal to the user:

- Local critical path (must stay in Codex)
- Outsource packet A/B/C (bounded, verifiable, low-risk)
- Acceptance criteria per packet

Use this packet template:

- `目标`: one clear objective
- `范围`: explicit file list
- `约束`: no unrelated edits, no destructive operations
- `输出`: changed files + concise conclusion
- `验收`: concrete build/test/read checks

For delegation tasks, remind model confirmation before dispatch:

- `本次外包使用模型：<provider/model>，是否按这个执行？`

## Typical Requests

- `build`
- `flash`
- `clean`
- `analyze this project`
- `read COM8 logs`
- `tail serial output`
- `why did flash fail`
- `build this repo the local way`
