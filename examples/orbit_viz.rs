#![allow(missing_docs)]

use macroquad::prelude::*;
use space_sim::{
    Angle, CircularOrbit, Distance, Epoch, EulerIntegrator, GravitationalParameter, Integrator,
    MU_EARTH, OrbitDirection, OrbitPlane, R_EARTH, TimeStep, TwoBodyGravity,
};

const TRAIL_POINTS_MAX: usize = 2_000;

fn main() {
    let window_config = Conf {
        window_title: "Space Sim".to_string(),
        ..Default::default()
    };

    macroquad::Window::from_config(window_config, async_main());
}

async fn async_main() {
    let Some(mut scenario) = OrbitVizScenario::new() else {
        eprintln!("invalid orbit visualization scenario");
        return;
    };

    scenario.run().await;
}

struct OrbitVizScenario {
    earth: TwoBodyGravity,
    integrator: EulerIntegrator,
    state: space_sim::CartesianState,
    t: Epoch,
    dt: TimeStep,
    earth_radius_m: f64,
    meters_per_pixel: f64,
    trail: [Vec2; TRAIL_POINTS_MAX],
    trail_start: usize,
    trail_count: usize,
}

impl OrbitVizScenario {
    fn new() -> Option<Self> {
        let earth = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        };
        let integrator = EulerIntegrator;

        let altitude_m = 400_000.0;
        let earth_radius_m = R_EARTH;
        let orbital_radius_m = earth_radius_m + altitude_m;
        let orbit = CircularOrbit {
            radius: Distance::from_meters(orbital_radius_m)?,
            mu: earth.mu,
            plane: OrbitPlane::Xy,
            phase: Angle::from_radians(0.0)?,
            direction: OrbitDirection::Prograde,
        };

        Some(Self {
            earth,
            integrator,
            state: orbit.to_cartesian_state()?,
            t: Epoch::from_seconds(0.0)?,
            dt: TimeStep::from_seconds(1.0)?,
            earth_radius_m,
            meters_per_pixel: 40_000.0,
            trail: [Vec2::ZERO; TRAIL_POINTS_MAX],
            trail_start: 0,
            trail_count: 0,
        })
    }

    async fn run(&mut self) {
        loop {
            for _ in 0..20 {
                let Ok(next_state) =
                    self.integrator
                        .step(&self.earth, &self.state, self.t, self.dt)
                else {
                    eprintln!("simulation state became invalid");
                    return;
                };
                self.state = next_state;
                let Some(next_epoch) = self.t.advance_by(self.dt) else {
                    eprintln!("simulation epoch became invalid");
                    return;
                };
                self.t = next_epoch;

                let position = self.state.position.vector();
                let trail_next = (self.trail_start + self.trail_count) % TRAIL_POINTS_MAX;
                self.trail[trail_next] =
                    world_to_screen(position.x, position.y, self.meters_per_pixel);

                if self.trail_count < TRAIL_POINTS_MAX {
                    self.trail_count += 1;
                } else {
                    self.trail_start = (self.trail_start + 1) % TRAIL_POINTS_MAX;
                }
            }

            clear_background(BLACK);

            let center = vec2(screen_width() / 2.0, screen_height() / 2.0);
            draw_circle(center.x, center.y, 10.0, BLUE);

            for trail_index in 1..self.trail_count {
                let previous = (self.trail_start + trail_index - 1) % TRAIL_POINTS_MAX;
                let current = (self.trail_start + trail_index) % TRAIL_POINTS_MAX;
                let start = self.trail[previous] + center;
                let end = self.trail[current] + center;
                draw_line(start.x, start.y, end.x, end.y, 1.0, GREEN);
            }

            let position = self.state.position.vector();
            let ship = world_to_screen(position.x, position.y, self.meters_per_pixel) + center;
            draw_circle(ship.x, ship.y, 4.0, WHITE);

            let altitude_m = position.norm() - self.earth_radius_m;
            draw_text(
                &format!(
                    "t = {:.0}s | altitude = {:.0} km",
                    self.t.seconds(),
                    altitude_m / 1000.0
                ),
                20.0,
                30.0,
                24.0,
                WHITE,
            );

            next_frame().await;
        }
    }
}

fn world_to_screen(x_m: f64, y_m: f64, meters_per_pixel: f64) -> Vec2 {
    vec2(
        (x_m / meters_per_pixel) as f32,
        -(y_m / meters_per_pixel) as f32,
    )
}
