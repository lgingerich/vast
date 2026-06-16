#![allow(missing_docs)]

use macroquad::prelude::*;
use vast::{
    CartesianStateBuffer, Distance, Duration, EllipticOrbit, Epoch, Integrator,
    LeapfrogIntegrator, Orbit, OrbitalDiagnostics, R_EARTH, Speed, TimeStep, TwoBodyGravity,
};

const TRAIL_POINTS_MAX: usize = 2_000;

fn main() {
    let window_config = Conf {
        window_title: "VAST".to_string(),
        ..Default::default()
    };

    macroquad::Window::from_config(window_config, async_main());
}

async fn async_main() {
    let Ok(mut scenario) = OrbitVizScenario::new() else {
        eprintln!("invalid orbit visualization scenario");
        return;
    };

    scenario.run().await;
}

struct OrbitVizScenario {
    earth: TwoBodyGravity,
    integrator: LeapfrogIntegrator,
    states: CartesianStateBuffer<1>,
    t: Epoch,
    dt: TimeStep,
    earth_radius_km: f64,
    ideal_orbital_radius: Distance,
    circular_speed: Speed,
    orbital_period: Duration,
    meters_per_pixel: f64,
    trail: [Vec2; TRAIL_POINTS_MAX],
    trail_start: usize,
    trail_count: usize,
}

impl OrbitVizScenario {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let earth = TwoBodyGravity::earth();
        let integrator = LeapfrogIntegrator;
        let altitude = Distance::new((400.0) * 1000.0)?;
        let orbit: EllipticOrbit = Orbit::CircularEquatorial { altitude }.try_into()?;
        let earth_radius_km = R_EARTH / 1000.0;

        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;

        Ok(Self {
            earth,
            integrator,
            states,
            t: Epoch::ZERO,
            dt: TimeStep::new(1.0)?,
            earth_radius_km,
            ideal_orbital_radius: orbit.semi_major_axis(),
            circular_speed: orbit.speed()?,
            orbital_period: orbit.period()?,
            meters_per_pixel: 40_000.0,
            trail: [Vec2::ZERO; TRAIL_POINTS_MAX],
            trail_start: 0,
            trail_count: 0,
        })
    }

    async fn run(&mut self) {
        loop {
            for _ in 0..20 {
                let Ok(()) = self.integrator.step(
                    &self.earth,
                    &mut self.states.view_mut(),
                    0,
                    self.t,
                    self.dt,
                ) else {
                    eprintln!("simulation state became invalid");
                    return;
                };
                self.t = self.t.advance_by(self.dt);

                let Ok(state) = self.states.view().read_state(0) else {
                    eprintln!("simulation state became invalid");
                    return;
                };
                let position = state.position.vector();
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

            let Ok(state) = self.states.view().read_state(0) else {
                eprintln!("simulation state became invalid");
                return;
            };
            let position = state.position.vector();
            let ship = world_to_screen(position.x, position.y, self.meters_per_pixel) + center;
            draw_circle(ship.x, ship.y, 4.0, WHITE);

            self.draw_hud();

            next_frame().await;
        }
    }

    fn draw_hud(&self) {
        let Some(diagnostics) =
            OrbitalDiagnostics::from_states(&self.states.view(), 0, self.earth.mu()).ok()
        else {
            draw_text("orbital diagnostics unavailable", 20.0, 30.0, 24.0, RED);
            return;
        };

        let altitude_km = diagnostics.radius().meters() / 1000.0 - self.earth_radius_km;
        let radius_drift_km = diagnostics.radius().meters() / 1000.0 - self.ideal_orbital_radius.meters() / 1000.0;
        let orbit_progress = self.t.seconds() / self.orbital_period.seconds();
        let orbit_progress_percent = orbit_progress.fract() * 100.0;

        let lines = [
            format!(
                "t = {:.0}s | orbit progress = {:.1}% ({:.2} rev)",
                self.t.seconds(),
                orbit_progress_percent,
                orbit_progress,
            ),
            format!(
                "altitude = {:.0} km | circular radius drift = {:+.3} km",
                altitude_km, radius_drift_km,
            ),
            format!(
                "speed = {:.3} km/s | circular speed = {:.3} km/s",
                diagnostics.speed().meters_per_second() / 1000.0,
                self.circular_speed.meters_per_second() / 1000.0,
            ),
            format!(
                "eccentricity = {:.9} | specific energy = {:.3} MJ/kg",
                diagnostics.eccentricity().scalar(),
                diagnostics
                    .specific_energy()
                    .square_meters_per_square_second()
                    / 1.0e6,
            ),
        ];

        for (index, line) in lines.iter().enumerate() {
            draw_text(line, 20.0, 30.0 + index as f32 * 26.0, 24.0, WHITE);
        }
    }
}

fn world_to_screen(x_m: f64, y_m: f64, meters_per_pixel: f64) -> Vec2 {
    vec2(
        (x_m / meters_per_pixel) as f32,
        -(y_m / meters_per_pixel) as f32,
    )
}
