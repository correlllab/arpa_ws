# Memory & playbook

## Workflow Orchestration

### 1. Plan Node Default
- **Enter plan mode for ANY non-trivial task** (defined as 3+ steps or involving architectural decisions).
- If something goes "sideways", **STOP and re-plan immediately**, rather than continuing to push forward.
- Use **plan mode for verification steps**, not just for initial building.
- Write **detailed specs upfront to reduce ambiguity**.

### 2. Subagent Strategy
- **Use subagents liberally to keep the main context window clean.**
- **Offload research, exploration, and parallel analysis to subagents.**
- For complex problems, **throw more compute at it via subagents.**
- Use **one task per subagent for focused execution.**

### 3. Self-Improvement Loop
- **After ANY correction from the user, update `tasks/lessons.md`** following a specified pattern.
- **Write rules to prevent the same mistake** from occurring again.
- **Ruthlessly iterate on these lessons** until the mistake rate drops.
- **Review lessons at the session start** for the relevant project.

### 4. Verification Before Done
- **Never mark a task complete without proving it works.**
- **Diff behavior between the main branch and your changes** when relevant.
- Ask: **"Would a staff engineer approve this?"**
- **Run tests, check logs, and demonstrate correctness.**

### 5. Demand Elegance (Balanced)
- For non-trivial changes, **pause and ask "is there a more elegant way?"**
- If a fix "feels hacky": **"Knowing everything I know now, implement the elegant solution."**
- **Skip simple, obvious fixes**—don't over-engineer.
- **Challenge your own work before presenting it.**

### 6. Autonomous Bug Fixing
- When given a bug report: **"just fix it. Don't ask for hand-holding."**
- **Point at logs, errors, failing tests—then resolve them.**
- **Zero context switching required from the user.**
- **Go fix failing CI tests without being told how.**

---

## Task Management

1. **Plan First**: Write a plan in `tasks/todo.md` with checkable items.
2. **Verify Plan**: Check in before starting implementation.
3. **Track Progress**: Mark items complete as you go.
4. **Explain Changes**: Provide a high-level summary at each step.
5. **Document Results**: Add a review section to `tasks/todo.md`.
6. **Capture Lessons**: Update `tasks/lessons.md` after corrections.

---

## Core Principles

- **Simplicity First**: Make every change as simple as possible. Impact minimal code.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.
