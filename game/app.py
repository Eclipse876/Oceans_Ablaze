import math
from collections import deque
from dataclasses import dataclass
from typing import Tuple

import arcade
from PIL import Image
from pyglet import gl
from pyglet.math import Vec2

from game import config
from game.nav.planner import NavPlanner


@dataclass
class Ship:
    spec: config.ShipSpec
    pos: Vec2
    vel: Vec2


def make_feathered_circle_texture(
    diameter: int,
    feather_px: int = 3,
    color=(255, 255, 255, 255),
) -> Image.Image:
    d = max(4, int(diameter))
    cx, cy = (d - 1) / 2.0, (d - 1) / 2.0
    r = (d - 1) / 2.0
    img = Image.new("RGBA", (d, d), (0, 0, 0, 0))
    px = img.load()
    for y in range(d):
        for x in range(d):
            dist = math.hypot(x - cx, y - cy)
            a = 0.0
            if dist <= r - feather_px:
                a = 1.0
            elif dist <= r:
                a = max(0.0, (r - dist) / max(1.0, feather_px))
            if a > 0.0:
                px[x, y] = (color[0], color[1], color[2], int(color[3] * a))
    return img


def make_feathered_strip_texture(
    height_px: int, feather_px: int = 2, color=(255, 255, 255, 255)
) -> Image.Image:
    h = max(4, int(height_px))
    w = 16
    cy = (h - 1) / 2.0
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    px = img.load()
    for y in range(h):
        for x in range(w):
            dist = abs(y - cy)
            half = cy
            a = max(0.0, 1.0 - (dist / max(1.0, half)))
            if dist > half - feather_px:
                a *= max(0.0, (half - dist) / max(1.0, feather_px))
            if a > 0.0:
                px[x, y] = (color[0], color[1], color[2], int(color[3] * a))
    return img


class NavalGame(arcade.Window):
    """Main Game Window for Naval RTS"""

    # --- Movement dynamics (tweak to taste) ---
    SLOW_RADIUS_FINAL = 50.0  # only used for the FINAL waypoint
    WAYPOINT_RADIUS = 6.0  # when to pop a waypoint if we’re deliberately slowing (final)
    PASS_EPS = 6.0  # how far past an intermediate waypoint counts as “passed”
    A_LONG_MAX = 6.0  # m/s^2 longitudinal accel/decel (already similar in your code)
    # A_LAT_MAX = 3.0  # m/s^2 lateral capability (turning grip)
    LOOK_AHEAD_CORNER = 120.0  # start corner-speed logic within this distance to the corner

    # --- Prototype Tunables ---
    SHIP_RADIUS = 20
    ARRIVAL_RADIUS = 0

    SLOW_RADIUS = 140
    STOP_RADIUS = 0.5

    DEST_LINE_COLOR = arcade.color.WHITE
    DEST_LINE_WIDTH = 7  # was 5

    VELOCITY_VECTOR_COLOR = arcade.color.GREEN
    VELOCITY_VECTOR_WIDTH = 7  # was 5
    VELOCITY_VECTOR_SCALE = 4  # optional: slightly longer arrow for same speed

    # ----- Base Helpers -----
    @staticmethod
    def _unit(v: Vec2) -> Vec2:
        m = math.hypot(v.x, v.y)
        if m > 1e-6:
            return Vec2(v.x / m, v.y / m)
        return Vec2(0, 0)

    # ----- Resolution Helpers -----
    def draw_circle_smooth(self, x: float, y: float, radius: float, color: Tuple[int, int, int]):
        # The base texture is 64px diameter; compute scale so sprite radius == requested radius
        base_d = 64
        scale = (radius * 2) / base_d
        spr = arcade.Sprite(center_x=x, center_y=y, texture=self.circle_tex, scale=scale)
        spr.color = color  # tint to desired color
        spr.draw()

    def draw_line_smooth(
        self, x1: float, y1: float, x2: float, y2: float, width: float, color: Tuple[int, int, int]
    ):
        dx, dy = (x2 - x1), (y2 - y1)
        length = max(1e-6, math.hypot(dx, dy))
        angle_deg = math.degrees(math.atan2(dy, dx))
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        # Our strip texture is 16x32; scale_x stretches along length, scale_y to requested width
        spr = arcade.Sprite(center_x=cx, center_y=cy, texture=self.line_tex)
        spr.angle = angle_deg
        spr.width = length
        spr.height = max(1.0, width)
        spr.color = color
        spr.draw()

    def __init__(self):
        # Now calling the parent class (arcade.Window) so create the game window
        super().__init__(
            width=config.VIEWPORT_WIDTH,
            height=config.VIEWPORT_HEIGHT,
            title="Oceans Ablaze Pre-Pre-Alpha",
            resizable=True,
            antialiasing=True,
        )

        self.friendly = Ship(spec=config.CARRIER, pos=Vec2(200, 150), vel=Vec2(0, 0))
        self.friendly_target = None

        try:
            gl.glEnable(gl.GL_MULTISAMPLE)
        except Exception:
            pass

        arcade.set_background_color(config.OCEAN_COLOR)
        self.setup()

    def setup(self) -> None:
        """
        Initialize game state, build nav planner, and prepare draw helpers.
        Called once at window init and whenever you want to reset.
        """

        # --- Ship initial state ---
        self.friendly.pos = Vec2(200, 150)
        self.friendly.vel = Vec2(0, 0)
        self.friendly_target = None

        # Example island polygon
        self.island_points = [
            (300, 200),
            (500, 250),
            (550, 400),
            (400, 450),
            (250, 350),
        ]

        # Other entities
        self.hostile_pos = Vec2(600, 200)
        self.neutral_pos = Vec2(400, 100)
        self.unknown_pos = Vec2(700, 400)

        # --- Drawing helpers (textures for anti-aliased lines/circles) ---
        circle_img = make_feathered_circle_texture(diameter=self.SHIP_RADIUS * 2)
        self.circle_tex = arcade.Texture(name="circle_fthr", image=circle_img)

        line_img = make_feathered_strip_texture(height_px=5)
        self.line_tex = arcade.Texture(name="line_fthr", image=line_img)

        # --- Path planner setup ---

        self.ship_radius_m = 20.0  # hull radius in meters; tweak per unit type
        self.nav = NavPlanner(
            world_origin=(0.0, 0.0),
            world_size=(float(self.width), float(self.height)),
            cell_m=3.0,  # meters per cell
            land_polys=[[(float(x), float(y)) for (x, y) in self.island_points]],
        )

        #Final Goal of the Current Route
        self.route_goal = None

        # Queue of waypoints (Vec2) returned by planner
        self.path_waypoints = deque()

        # For movement: remember where the current leg started (used in pass-by logic)
        self._segment_start = Vec2(self.friendly.pos.x, self.friendly.pos.y)

    # ----- Input -----
    def on_mouse_press(self, x: float, y: float, button: int, modifiers: int):
        self._segment_start = Vec2(self.friendly.pos.x, self.friendly.pos.y)
        start = (self.friendly.pos.x, self.friendly.pos.y)
        goal = (float(x), float(y))
        plan = self.nav.plan_ship_path(
            start,
            goal,
            hull_radius_m=self.friendly.spec.radius_m,
            safety_m=self.friendly.spec.safety_m,
        )
        self.path_waypoints.clear()
        if plan and plan.waypoints:
            # Store as pyglet Vec2 for mover
            for wx, wy in plan.waypoints:
                self.path_waypoints.append(Vec2(wx, wy))
        else:
            # No path -> stop
            self.friendly_target = None
        print("Planned waypoints:", plan.waypoints if plan else None)
        self.route_goal = Vec2(float(x), float(y))

    def on_resize(self, width: int, height: int):
        super().on_resize(width, height)
        arcade.set_viewport(0, width, 0, height)

    # ----- Simulation -----
    def on_update(self, dt: float):
        # If we have a planned path, set the next waypoint as the current target
        if self.path_waypoints:
            current_wp = self.path_waypoints[0]
            to_wp = current_wp - self.friendly.pos
            distance = to_wp.mag

            # --- NEW: if LOS to the current waypoint is blocked, replan to final goal
            goal_tuple = None
            if self.route_goal is not None:
                goal_tuple = (self.route_goal.x, self.route_goal.y)
            elif len(self.path_waypoints) > 0:
                # fallback: last queued waypoint
                last = self.path_waypoints[-1]
                goal_tuple = (last.x, last.y)

            if goal_tuple is not None:
                if not self.nav.los_clear_for_ship(
                    (self.friendly.pos.x, self.friendly.pos.y),
                    (current_wp.x, current_wp.y),
                    hull_radius_m=self.friendly.spec.radius_m,
                    safety_m=self.friendly.spec.safety_m,
                ):
                    plan = self.nav.plan_ship_path(
                        (self.friendly.pos.x, self.friendly.pos.y),
                        goal_tuple,
                        hull_radius_m=self.friendly.spec.radius_m,
                        safety_m=self.friendly.spec.safety_m,
                    )
                    self.path_waypoints.clear()
                    if plan and plan.waypoints:
                        for wx, wy in plan.waypoints:
                            self.path_waypoints.append(Vec2(wx, wy))
                        # reset segment start for new first leg
                        self._segment_start = Vec2(self.friendly.pos.x, self.friendly.pos.y)
                    else:
                        self.friendly_target = None
                    return  # skip the rest of this update after replanning

            # --- Is this the final waypoint of the route?---
            is_final = len(self.path_waypoints) == 1

            # --- Pass Through if NOT last point.---
            seg = current_wp - self._segment_start
            seg_len = seg.mag
            if not is_final and seg_len > 1e-6:
                seg_dir = self._unit(seg)
                # Have we passed through a waypoint along the same segment direction?
                # If the vector from wp to our position has positive mag on seg_dir, we’re past it
                wp_to_pos = self.friendly.pos - current_wp
                # 1) distance-based pop (close enough = done)
                if distance <= max(self.WAYPOINT_RADIUS, 6.0):
                    self.path_waypoints.popleft()
                    self._segment_start = Vec2(current_wp.x, current_wp.y)
                    return

                # 2) projection-based pop (we've passed it along the path direction)
                if wp_to_pos.dot(seg_dir) > self.PASS_EPS:
                    self.path_waypoints.popleft()
                    self._segment_start = Vec2(current_wp.x, current_wp.y)
                    return

            # ---Desired Decel for FINAL point
            if is_final:
                # Original speed scaling for the last point
                if distance > self.SLOW_RADIUS_FINAL:
                    desired_speed = self.friendly.spec.max_speed_mps
                else:
                    t = distance / max(self.SLOW_RADIUS_FINAL, 1e-6)
                    desired_speed = self.friendly.spec.max_speed_mps * t
            else:
                desired_speed = self.friendly.spec.max_speed_mps

                # -------- Cornering speed cap (hopefully stops beaching on turns... TBD) --------
                if not is_final and len(self.path_waypoints) >= 2:
                    next_wp = self.path_waypoints[1]
                    v_in_raw = current_wp - self._segment_start
                    v_out_raw = next_wp - current_wp
                    if v_in_raw.mag > 1e-6 and v_out_raw.mag > 1e-6:
                        v_in = self._unit(v_in_raw)
                        v_out = self._unit(v_out_raw)
                        dot = max(-1.0, min(1.0, v_in.dot(v_out)))
                        theta = math.acos(dot)


                # -------- Steering toward the current waypoint --------
                desired_dir = self._unit(to_wp)
                desired_vel = Vec2(desired_dir.x * desired_speed, desired_dir.y * desired_speed)

                # --- Coast repulsion: nudge away from shoreline when inside buffer
                clear_need = self.friendly.spec.radius_m + self.friendly.spec.safety_m
                dist_m = self.nav.distance_to_land_m((self.friendly.pos.x, self.friendly.pos.y))
                repel_start = clear_need + 25.0  # begin nudging 25 m before hard limit
                repel_full = clear_need + 5.0  # full push if you're inside this

                if dist_m < repel_start:
                    nx, ny = self.nav.land_normal((self.friendly.pos.x, self.friendly.pos.y))
                    # 0..1 strength as you approach shoreline; clamps at full when very close
                    t = 0.0
                    if dist_m > repel_full:
                        t = (repel_start - dist_m) / max(repel_start - repel_full, 1e-6)
                    else:
                        t = 1.0
                    repel_scale = self.friendly.spec.max_speed_mps * 0.5  # half-speed worth of bias
                    desired_vel = Vec2(
                        desired_vel.x + nx * repel_scale * t, desired_vel.y + ny * repel_scale * t
                    )

                # --- 3) Proactive reroute when too close: widen the route now
                if dist_m < (clear_need + 10.0):
                    goal = (
                        (self.route_goal.x, self.route_goal.y)
                        if self.route_goal
                        else (current_wp.x, current_wp.y)
                    )
                    plan = self.nav.plan_ship_path(
                        (self.friendly.pos.x, self.friendly.pos.y),
                        goal,
                        hull_radius_m=self.friendly.spec.radius_m,
                        safety_m=self.friendly.spec.safety_m + 15.0,  # temporary extra buffer
                    )
                    if plan and plan.waypoints:
                        self.path_waypoints.clear()
                        for wx, wy in plan.waypoints:
                            self.path_waypoints.append(Vec2(wx, wy))
                        self._segment_start = Vec2(self.friendly.pos.x, self.friendly.pos.y)
                        return  # skip steering/integration this frame after replanning


                steer = Vec2(
                    desired_vel.x - self.friendly.vel.x, desired_vel.y - self.friendly.vel.y
                )
                steer_mag = steer.mag
                max_delta_v = self.friendly.spec.max_accel_mps2 * dt
                if steer_mag > max_delta_v and steer_mag > 1e-6:
                    k = max_delta_v / steer_mag
                    steer = Vec2(steer.x * k, steer.y * k)

                self.friendly.vel = Vec2(
                    self.friendly.vel.x + steer.x, self.friendly.vel.y + steer.y
                )

                speed = self.friendly.vel.mag
                if speed > self.friendly.spec.max_speed_mps:
                    s = self.friendly.spec.max_speed_mps / speed
                    self.friendly.vel = Vec2(self.friendly.vel.x * s, self.friendly.vel.y * s)

                # -------- Arrival & popping logic --------
                if is_final:
                    # Same overshoot guard you had, but only for the final point
                    if distance <= speed * dt:
                        self.friendly.pos = Vec2(current_wp.x, current_wp.y)
                        self.friendly.vel = Vec2(0, 0)
                        self.path_waypoints.popleft()
                        # keep segment_start as is; if new path will reset it
                    else:
                        self.friendly.pos = Vec2(
                            self.friendly.pos.x + self.friendly.vel.x * dt,
                            self.friendly.pos.y + self.friendly.vel.y * dt,
                        )
                else:
                    # Intermediate: never slow to stop—just integrate
                    self.friendly.pos = Vec2(
                        self.friendly.pos.x + self.friendly.vel.x * dt,
                        self.friendly.pos.y + self.friendly.vel.y * dt,
                    )
                return

        # --- ORIGINAL “no target” or “no waypoints” branch (idle damping) ---
        if self.friendly_target is None:
            if self.friendly.vel.mag > 0:
                speed = self.friendly.vel.mag
                decel = min(speed, self.friendly.spec.max_accel_mps2 * dt)
                if speed > 1e-6:
                    scale = (speed - decel) / speed
                    self.friendly.vel = Vec2(
                        self.friendly.vel.x * scale, self.friendly.vel.y * scale
                    )
            self.friendly.pos = Vec2(
                self.friendly.pos.x + self.friendly.vel.x * dt,
                self.friendly.pos.y + self.friendly.vel.y * dt,
            )
            return

        # Vector from ship to target
        to_target = self.friendly_target - self.friendly.pos
        distance = to_target.mag

        # Have we arrived?
        if distance <= self.ARRIVAL_RADIUS and self.friendly.vel.mag < 1:
            self.friendly.pos = Vec2(self.friendly_target.x, self.friendly_target.y)
            self.friendly.vel = Vec2(0, 0)
            self.friendly_target = None
            return

        if distance > self.SLOW_RADIUS:
            desired_speed = self.friendly.spec.max_speed_mps
        else:
            t = distance / max(self.SLOW_RADIUS, 1e-6)
            desired_speed = self.friendly.spec.max_speed_mps * t

        # Desired velocity vector
        desired_dir = to_target.normalize() if distance > 1e-6 else Vec2(0, 0)
        desired_vel = Vec2(desired_dir.x * desired_speed, desired_dir.y * desired_speed)

        # Steering = how much we want to change velocity this frame
        steer = Vec2(desired_vel.x - self.friendly.vel.x, desired_vel.y - self.friendly.vel.y)

        # Limit steering strength to max acceleration * dt
        steer_mag = steer.mag
        max_delta_v = self.friendly.spec.max_accel_mps2 * dt
        if steer_mag > max_delta_v and steer_mag > 1e-6:
            k = max_delta_v / steer_mag
            steer = Vec2(steer.x * k, steer.y * k)

        # Apply steering to velocity
        self.friendly.vel = Vec2(self.friendly.vel.x + steer.x, self.friendly.vel.y + steer.y)

        # Optional: cap velocity to top speed (robustness)
        speed = self.friendly.vel.mag
        if speed > self.friendly.spec.max_speed_mps:
            s = self.friendly.spec.max_speed_mps / speed
            self.friendly.vel = Vec2(self.friendly.vel.x * s, self.friendly.vel.y * s)

        # Prevent overshoot: if we’d pass the target this frame, clamp to target and stop
        if distance <= speed * dt:
            self.friendly.pos = Vec2(self.friendly_target.x, self.friendly_target.y)
            self.friendly.vel = Vec2(0, 0)
            self.friendly_target = None
            return

        # Integrate position
        self.friendly.pos = Vec2(
            self.friendly.pos.x + self.friendly.vel.x * dt,
            self.friendly.pos.y + self.friendly.vel.y * dt,
        )

    # ----- Rendering -----
    def on_draw(self):
        """This calls a frame to be generated"""
        self.clear()  # this clears the previous frame and fills the new frame with background color
        if len(self.path_waypoints) >= 1:
            px, py = self.friendly.pos.x, self.friendly.pos.y
            for wp in self.path_waypoints:
                self.draw_line_smooth(
                    px, py, wp.x, wp.y, self.DEST_LINE_WIDTH, color=self.DEST_LINE_COLOR
                )
                px, py = wp.x, wp.y

        # --- Draw Island ---
        pts = self.island_points
        for i in range(len(pts)):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % len(pts)]
            self.draw_line_smooth(x1, y1, x2, y2, self.DEST_LINE_WIDTH, color=self.DEST_LINE_COLOR)

        # Movement Line
        if self.friendly_target is not None:
            self.draw_line_smooth(
                self.friendly.pos.x,
                self.friendly.pos.y,
                self.friendly_target.x,
                self.friendly_target.y,
                self.DEST_LINE_WIDTH,
                self.DEST_LINE_COLOR,
            )

        # Velocity Vector
        speed_now = self.friendly.vel.mag
        if speed_now > 1e-3:
            # scale length with speed
            vec_len = speed_now * self.VELOCITY_VECTOR_SCALE
            # normalize velocity to avoid Vec2 * float - scale
            dir_x = self.friendly.vel.x / speed_now
            dir_y = self.friendly.vel.y / speed_now
            end_x = self.friendly.pos.x + dir_x * vec_len
            end_y = self.friendly.pos.y + dir_y * vec_len

            self.draw_line_smooth(
                self.friendly.pos.x,
                self.friendly.pos.y,
                end_x,
                end_y,
                self.VELOCITY_VECTOR_WIDTH,
                color=self.VELOCITY_VECTOR_COLOR,
            )

        # Friendly (Movable)
        self.draw_circle_smooth(
            self.friendly.pos.x, self.friendly.pos.y, self.friendly.spec.beam_m / 2.0, 
            config.FRIENDLY_COLOR)

        # Hostile / Neutral / Unknown (Not moving yet)
        self.draw_circle_smooth(self.hostile_pos.x, self.hostile_pos.y, 15, config.HOSTILE_COLOR)
        self.draw_circle_smooth(self.neutral_pos.x, self.neutral_pos.y, 12, config.NEUTRAL_COLOR)
        self.draw_circle_smooth(self.unknown_pos.x, self.unknown_pos.y, 10, config.UNKNOWN_COLOR)

        # --- Identifiers ---
        arcade.draw_text(
            "Friendly",
            self.friendly.pos.x - 22,
            self.friendly.pos.y - 32,
            config.FRIENDLY_COLOR,
            12,
        )
        arcade.draw_text(
            "Hostile", self.hostile_pos.x - 20, self.hostile_pos.y - 32, config.HOSTILE_COLOR, 12
        )
        arcade.draw_text(
            "Neutral", self.neutral_pos.x - 22, self.neutral_pos.y - 32, config.NEUTRAL_COLOR, 12
        )
        arcade.draw_text(
            "Unknown", self.unknown_pos.x - 28, self.unknown_pos.y - 32, config.UNKNOWN_COLOR, 12
        )

        arcade.draw_text(
            "Welcome to the Ocean! It is not ablaze yet... Left Click to move the green ship",
            20,
            self.height - 40,
            arcade.color.WHITE,
            18,
        )


def main():
    NavalGame()
    arcade.run()

if __name__ == "__main__":
    main()