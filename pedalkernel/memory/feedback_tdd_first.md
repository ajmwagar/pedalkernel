---
name: TDD first — always write tests before implementation
description: User requires TDD approach — write failing tests first, then implement. Agents that investigate instead of writing tests first get killed.
type: feedback
---

When user says "TDD" or "write tests first", write the failing tests IMMEDIATELY. Do not investigate the codebase first. Do not trace control flow. Write the test, run it, confirm it fails, THEN implement.

**Why:** User killed an agent that spent its entire runtime investigating instead of writing tests first as instructed. Investigation-first wastes time and ignores explicit instructions.

**How to apply:** In dispatch prompts to supervisors, put test-writing as step 1 with explicit "DO NOT investigate first" guardrails. When working directly, write the test file before reading implementation code.
