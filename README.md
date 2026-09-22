# WATonomous ASD Assignment — Autonomous Navigation Stack

Four ROS 2 nodes that let a simulated robot drive to a clicked goal on its own, discovering and avoiding obstacles as it goes. C++17, ROS 2 Humble, Gazebo, run through Docker.

[![Demo — click to watch](https://img.youtube.com/vi/u6Sl_1bKBFA/maxresdefault.jpg)](https://youtu.be/u6Sl_1bKBFA)

*Click a goal; the robot maps, plans and drives there on its own.*

## What it does

```
gazebo ──/lidar──> [costmap] ──/costmap──> [map_memory] ──/map──> [planner] ──/path──> [control] ──/cmd_vel──> gazebo
                                                ▲                     ▲                     ▲
                                          /odom/filtered ─────────────┴─────────────────────┘
```

| Node | Question it answers | How |
|---|---|---|
| **costmap** | What's around me right now? | Lidar beams → occupancy grid. Obstacles marked 100, wrapped in a lethal zone. |
| **map_memory** | What have I seen so far? | Rotates each costmap into the world frame by the robot's heading and merges it into a map that persists. |
| **planner** | Where should I go? | A* over the global map. Replans twice a second. |
| **control** | How do I get there? | Pure pursuit — chase a point ahead on the path, steer along the arc that reaches it. |

It's a loop: control moves the robot, the lidar sees new things, the map grows, the planner reroutes. That's what lets it handle obstacles it hadn't seen when the goal was set.

Each package is split into a `_core` (the algorithm, no ROS) and a `_node` (subscriptions, publishers, timers). The core classes can be tested without ROS running.

## Running it

Requires Docker. On Apple Silicon set `PLATFORM="arm64"` in `watod-config.sh` first — the default base image is amd64 and runs under emulation.

```bash
./watod build      # first time, or after code changes
./watod up
```

Foxglove at `ws://localhost:10020`, layout in `config/`. Set a goal with the 3D panel's publish tool (type: Point, topic: `/goal_point`).

If `/lidar` has no publisher after a rebuild, `./watod restart gazeboserver` — the Gazebo bridge sometimes doesn't survive the robot container cycling.

## How each node works

**costmap.** Each beam's angle is `angle_min + i * angle_increment`; its endpoint is `range·cos`, `range·sin`, converted to a cell with the robot at the grid centre. The grid is cleared every scan — a costmap is what the lidar sees *now*; remembering is map_memory's job. Inflation runs as a second pass over the recorded obstacle cells so the result doesn't depend on beam order.

**map_memory.** Integrates a costmap when the robot has moved 1.5 m (straight-line displacement from the last integration, not distance travelled — circling back to the same spot sees nothing new). Each cell is rotated by the robot's yaw and translated by its position before merging. Merging takes the max, so an obstacle seen once stays seen. The map publishes every tick regardless of whether it changed, because ROS's default QoS is volatile and a subscriber that connects late would otherwise never receive it.

**planner.** A* with a Euclidean heuristic. Step cost is 1 (√2 diagonal) plus the cell's cost value ÷ 5, so inflated cells are expensive and walls (100) are excluded from the neighbour set entirely. Two states: waiting for a goal, and driving toward one. In the second state a 500 ms timer checks arrival and otherwise replans against the newest map.

**control.** Finds the first path point at least `lookahead` metres ahead, rotates it into the robot's frame, and computes curvature as `2y / L²`. Linear speed is a ceiling reached only on straights; it drops in proportion to curvature. Every exit path publishes an explicit zero rather than relying on Gazebo's command timeout.

## Challenges

Everything below actually happened, in roughly this order. Each one changed the code.

### The robot orbited instead of driving

At 2.0 m/s with a 1.0 m lookahead, the log showed the distance-to-goal frozen and `w = 2.66` — a turn radius of 75 cm. The robot was circling a point it could never reach.

Curvature is `2y / L²`. With `L = 1` that's just `2y`, and at speed 2.0 the resulting angular velocity overshoots the carrot every cycle. **Lookahead has to scale with speed** — roughly speed × 2 s. That fixed the orbit but introduced the next problem.

### It cut corners into obstacles

A large lookahead aims across the inside of a bend. At constant speed the robot drove its fastest exactly where that was most dangerous.

Made speed a function of curvature: `speed = max / (1 + 2.5·|curvature|)`, floored at 0.35. Straights run at full speed; a real corner drops to about half. Angular velocity scales by the same factor, or the turn radius shrinks and the robot corkscrews.

### It accelerated before the turn was finished

The limiter reads curvature to the carrot. Near the end of a turn the carrot is already on the straight, so curvature reads zero and speed jumps to max — while the chassis is still rotating. It clipped a wall on its first turn.

Added an acceleration ramp: speed may rise by at most 0.05 per cycle (0.5 m/s²) but can drop instantly. Full speed takes ~1.7 s to reach, longer than the turn takes to complete.

### The path hugged walls

With inflation at 1.0 m, cost reaches zero one metre from a wall and the planner has no reason to stay further out. Raising the inflation radius alone moved the path outward slightly and it hugged the new edge instead.

Two changes: penalty weight from ÷10 to ÷5 so the gradient pushes harder, and a **lethal zone** — cells within a fixed radius of an obstacle become 100, which the planner can't enter at all. Soft cost can always be outbid by a long enough detour; a hard limit can't.

### It still scratched on the big cylinder

The chassis is a 2.0 × 1.0 m box. Its centre follows the path, but a corner is 1.12 m from centre. With a 1.0 m lethal radius the path could sit 1.0 m from a wall and the corner would reach 12 cm past it during turn-in.

Lethal radius to 1.4 m. Path centre ≥ 1.4 m from any obstacle; corner at worst 3.28 m from the 3 m column's centre, 28 cm clear.

### The planner started failing mid-turn

Log showed bursts of `no path found` during turns. The robot's centre was drifting a few cells inside its own lethal band, and `getNeighbors` rejected every cell ≥ 100 — so from inside the band A* had nowhere to expand. Control kept the last good path, but when a fresh one finally arrived the carrot jumped and the robot stuttered.

Rule change: a lethal cell can't be entered from free space, but if the search is already standing in one it may step through lethal cells to get out. The existing penalty makes each such step cost 20, so it leaves by the shortest route.

### It avoided corridors it fit through

Every corridor in the arena is at least 3.75 m wide; the robot needs 2.8 m with the lethal zone. But the soft gradient beyond lethal (0.8 m on each side) covered the whole remaining channel, so every step through a corridor cost something and open floor cost nothing. A* took the long way round.

Removed the gradient — inflation radius set equal to lethal. Hard wall, then free. The lethal zone alone provides the clearance guarantee; the gradient was only ever a preference, and here it was a bad one.

## Final parameters

| | value | why |
|---|---|---|
| linear speed | 1.5 m/s | 3× the spec; the limiter and ramp make it hold |
| lookahead | 3.0 m | speed × 2 s |
| goal tolerance | 0.3 m | one control step at this speed is 15 cm |
| curvature gain | 2.5 | corners land around 0.5–0.7 m/s |
| speed floor | 0.35 m/s | never stall in a hairpin |
| acceleration | 0.5 m/s² | finish turning before reaching full speed |
| lethal radius | 1.4 m | chassis corner sweep is 1.12 m |
| inflation radius | 1.4 m | no soft band — see corridors above |
| planner penalty | cost ÷ 5 | steep enough that inflated cells are avoided when possible |
| map update trigger | 1.5 m | assignment value |

The assignment specifies 0.5 m/s, lookahead 1.0, tolerance 0.1. Those work and are simpler; the values above are what it took to run three times faster without contact.

## Known limitations

- **37% of the arena is lethal.** A 1.4 m band around every obstacle in a 30 m arena leaves less free floor than it sounds like. Turns around the 3 m column are long. A smaller robot could use a smaller band.
- **Paths are jagged.** A* on an 8-connected grid produces staircases. Every diagonal-to-straight transition is a small corner that costs speed. A line-of-sight smoothing pass would help and isn't implemented.
- **The speed limiter reacts, it doesn't anticipate.** It slows when curvature to the carrot rises, which is partly ahead of the corner but not fully. Looking at the path's own bend over the next several waypoints would slow earlier.
- **Goals within 1.4 m of an obstacle fail.** The goal cell is lethal, so `no path found`. Correct, but not obvious to the user.
- **The local costmap is 30 × 30 m** — the same size as the arena. A local costmap should be roughly lidar range; this one is oversized and costs about 9× more per scan than a 10 m one would.
- **Unknown space is treated as free.** The map initialises to 0. The planner will route through unexplored areas optimistically and replan when it finds something. That's intended, but it means the first path to a goal is usually wrong.
- **No unit tests.** The `_core` split was designed for them and none were written.

## Things I'd do next

Path smoothing first — it's the change that makes higher speed realistic. Then move the tuning values into `params.yaml` so they can be changed with `ros2 param set` instead of a rebuild. Then tests on the pure functions: `worldToGrid`/`gridToWorld` round-trips, A* on a small grid with a known answer, the quaternion-to-yaw conversion.
