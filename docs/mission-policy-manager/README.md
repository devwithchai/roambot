# RoamBot Mission Policy Manager

## Overview

The Mission Policy Manager adds a warehouse-operational policy layer above Nav2. Operators can express constraints such as:

> Avoid aisle B while the forklift is operating. Prefer aisle C instead.

The first iteration uses predefined JSON policies (no local LLM dependency). A deterministic validator checks a policy before a ROS 2 adapter applies supported constraints to Nav2. Nav2's planner and controller remain responsible for path planning and motion control.

This work is inspired by the preprint [Natural-Language to Nav2: Operational Policies for Costmap-Based Navigation](https://arxiv.org/abs/2610.32633) (linked by the project notes as dated 23 October 2026). The preprint is a research reference; this RoamBot implementation is an independent, staged project.

## Design pipeline

```text
Operator instruction (future)
        |
        v
Policy input / JSON file (MVP)
        |
        v
mission_policy_manager
  - parse
  - schema + semantic validation
  - robot / condition / region checks
  - reject invalid or conflicting policy
        |
        v
Policy adapter
  - keepout constraints -> Nav2 costmap filter integration
  - route preferences -> explicit route/waypoint selection (separate mechanism)
        |
        v
Nav2 planner + controller -> robot
```

**Safety boundary:** natural-language or JSON input is never applied directly. The deterministic validation stage must be the gate. Policies should fail closed: unsupported, malformed, stale, or conflicting policies are rejected and reported. Do not bypass Nav2's collision checking or controller safety behavior.

## MVP scope

1. Define a stable JSON policy format.
2. Load predefined policies from files.
3. Validate schema, policy ID, robot targets, event/condition, region identifiers, priority, and supported actions.
4. Start with a static warehouse keepout region using Nav2 costmap filter configuration.
5. Verify that Scout avoids the designated region in simulation.
6. Extend to Service after Scout's behavior is validated.
7. Add event-driven activation/deactivation (e.g. forklift operating state).
8. Add natural-language-to-JSON translation later; it must still pass the same validator.

### Keepout vs. route preference

A keepout filter marks a region as prohibited to the planner. It does **not** express a soft preference for another aisle. A preferred route requires an explicit route/waypoint selection or route-ranking mechanism. Keep these as separate policy action types; do not claim a keepout filter alone implements both.

## Example policy

See [examples/avoid_aisle_b_forklift_operating.json](../../examples/mission_policies/avoid_aisle_b_forklift_operating.json) and [schema](../../config/mission_policy.schema.json).

This example is an illustrative policy contract. Region names such as `aisle_B` and `aisle_C` must be mapped to actual warehouse map geometry/route identifiers in a future adapter. The sample does not yet activate a live Nav2 filter.

## Proposed ROS 2 package

Package name: `roambot_mission_policy_manager`

Suggested responsibilities:

- `policy_loader`: load policy files and track policy lifecycle.
- `policy_validator`: JSON Schema validation plus semantic checks against configured robots, region registry, action support, and policy conflicts.
- `policy_manager_node`: ROS 2 node exposing policy submit/activate/deactivate/status interfaces.
- `nav2_policy_adapter`: map validated keepout policy actions into Nav2 costmap-filter configuration/state.
- `route_preference_adapter`: separately implement explicit route/waypoint preferences.
- `policy_status`: report accepted/rejected/applied status and reason.

Suggested interfaces (to be finalized during implementation):

- Service/action to submit or activate a policy.
- Service to deactivate a policy.
- Topic for active policy status and validation results.
- Parameters for robot namespace and approved region registry.

Do not implement speculative interfaces before reviewing the current RoamBot launch, namespaces, map and Nav2 filter setup.

## Validation requirements

- Reject unknown robot IDs and unknown region IDs.
- Reject unsupported action types.
- Require explicit activation conditions for event-dependent rules.
- Check priority and conflict behavior deterministically.
- Preserve an audit/status message for each accepted or rejected policy.
- Ensure policy removal restores the prior navigation configuration.
- Test malformed JSON, invalid schema, unavailable event source, unknown region, overlapping restrictions, and activation/deactivation.
- Simulate policy behavior for each robot namespace independently.

## Development phases

| Phase | Deliverable |
|---|---|
| 1 | JSON schema and example policy |
| 2 | File loader and deterministic validator |
| 3 | Region registry tied to warehouse map |
| 4 | Scout keepout-filter adapter + simulation test |
| 5 | Event-driven activation and status reporting |
| 6 | Service support and multi-robot policy targeting |
| 7 | Route preference adapter (distinct from keepout) |
| 8 | Optional local/remote LLM translation to JSON, guarded by validator |

## Research question

How reliably can a high-level operator instruction be translated into safe, deterministic navigation constraints, and how can the system explain policy acceptance, rejection, and resulting route behavior?

## Status

Planning/schema stage. No ROS 2 node or live Nav2 filter integration has been implemented yet. This document intentionally separates the design contract from future runtime behavior.
