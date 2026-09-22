# WATonomous ASD Assignment

Four ROS 2 nodes. The robot drives itself to a clicked goal and avoids whatever it finds on the way. C++17, ROS 2 Humble, Gazebo, Docker.

<video src="https://raw.githubusercontent.com/zupersun/Watonomous-ASD-Assignment/main/docs/demo.mp4" controls width="100%"></video>

*Click a goal. The robot maps, plans and drives there on its own. Also on [YouTube](https://youtu.be/u6Sl_1bKBFA).*

## What it does

`lidar > costmap > map memory > planner > control > wheels`

The costmap sees. Map memory remembers. The planner routes. Control drives. Driving reveals more, so the map grows and the route updates. Each package keeps its algorithm in a `_core` class with no ROS in it.

## Running it

```bash
./watod build
./watod up
```

Foxglove at `ws://localhost:10020`. Layout is in `config/`. Click a goal with the publish tool. Type Point, topic `/goal_point`.

Apple Silicon: set `PLATFORM="arm64"` in `watod-config.sh` first. No lidar after a rebuild? `./watod restart gazeboserver`.

## Challenges

All of these happened. In this order. Each one changed the code.

**Cutting corners into obstacles**
- The robot chases a point ahead on the path. That point has to be further away at higher speed. Too close and the robot is always turning toward something right in front of it, so it circles instead of driving. I saw exactly that at 2.0 m/s.
- But a far point aims across the inside of a bend. Constant speed meant fastest exactly where it was least safe.
- Speed now drops with curvature. Full on straights. About half in a real corner. Angular velocity scales the same way so the turn radius holds.

**Accelerating before the turn was done**
- The limiter reads curvature to the carrot. Near the end of a turn the carrot is already on the straight. Speed jumped to max with the chassis still rotating. It clipped a wall on the first turn.
- Added an acceleration ramp. Up by 0.05 per cycle. Down instantly. Full speed takes 1.7 seconds. The turn finishes first.

**Hugging walls**
- Inflation was 1.0 m. Cost hit zero one metre out. The planner rode that line. Widening inflation just moved the line.
- Penalty weight from ÷10 to ÷5. That helped a bit.
- The real fix was something the assignment doesn't ask for. The assignment's inflation is a soft cost that fades with distance. I added a **lethal zone** on top of it. Cells within a set radius of an obstacle become impassable, not just expensive. A soft cost can always be outbid by a long enough detour. A hard limit can't.

**Still scratching the big column**
- The chassis is 2 by 1 m. A corner sweeps 1.12 m from centre. Lethal was 1.0 m. The corner reached 12 cm past the wall on turn in.
- Lethal radius to 1.4 m. The corner clears by 28 cm at the worst angle.

**Planner failing mid turn**
- Bursts of `no path found` during turns. The robot drifted inside its own lethal band. A* rejected every neighbour and had nowhere to go. Control kept the old path, then stuttered when a fresh one landed.
- A lethal cell can't be entered from free space. From inside one, the search may step through lethal cells to get out. The penalty makes it leave by the shortest route.

**Avoiding corridors it fit through**
- Every corridor is at least 3.75 m wide. The robot needs 2.8 m with the lethal zone. But the 0.8 m soft gradient on each side filled the whole channel. Corridors cost something. Open floor cost nothing. A* went around.
- Removed the gradient. Inflation set equal to lethal. Hard wall, then free. The lethal zone handles clearance. The gradient was a preference, and a bad one here.

## Final parameters

| | value | why |
|---|---|---|
| linear speed | 1.5 m/s | 3× the spec, held by the limiter and ramp |
| lookahead | 3.0 m | speed × 2 s |
| goal tolerance | 0.3 m | one control step at this speed is 15 cm |
| curvature gain | 2.5 | corners land around 0.5 to 0.7 m/s |
| speed floor | 0.35 m/s | never stall in a hairpin |
| acceleration | 0.5 m/s² | finish turning before full speed |
| lethal radius | 1.4 m | my addition, not in the assignment. Chassis corner sweep is 1.12 m |
| inflation radius | 1.4 m | no soft band, see corridors above |
| planner penalty | cost ÷ 5 | inflated cells avoided when a free route exists |
| map update trigger | 1.5 m | assignment value |

The assignment values (0.5 m/s, lookahead 1.0, tolerance 0.1) work and are simpler. These are what it took to go three times faster without contact.
