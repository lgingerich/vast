#![allow(missing_docs)]

use macroquad::prelude::*;
use nalgebra::Vector3;
use space_sim::{
    CartesianState, Duration, Epoch, EulerIntegrator, Integrator, MU_EARTH, Position, R_EARTH,
    TwoBodyGravity, Velocity,
};

fn main() {
    let window_config = Conf {
        window_title: "Space Sim".to_string(),
        ..Default::default()
    };

    macroquad::Window::from_config(window_config, async_main());
}

async fn async_main() {
    let earth = TwoBodyGravity { mu: MU_EARTH };
    let integrator = EulerIntegrator;

    let altitude_m = 400_000.0;
    let earth_radius_m = R_EARTH;
    let orbital_radius = earth_radius_m + altitude_m;

    // Circular orbit speed: v = sqrt(mu / r)
    let orbital_speed = (earth.mu / orbital_radius).sqrt();

    let mut state = CartesianState {
        position: Position(Vector3::new(orbital_radius, 0.0, 0.0)),
        velocity: Velocity(Vector3::new(0.0, 5000.0, orbital_speed)),
    };

    let mut t = Epoch(0.0);
    let dt = Duration(1.0);
    let meters_per_pixel = 40_000.0;
    let mut trail = Vec::new();

    loop {
        for _ in 0..20 {
            state = integrator.step(&earth, &state, t, dt);
            t = Epoch(t.0 + dt.0);

            let position = state.position.0;
            trail.push(world_to_screen(position.x, position.y, meters_per_pixel));

            if trail.len() > 2_000 {
                trail.remove(0);
            }
        }

        clear_background(BLACK);

        let center = vec2(screen_width() / 2.0, screen_height() / 2.0);
        draw_circle(center.x, center.y, 10.0, BLUE);

        for segment in trail.windows(2) {
            let start = segment[0] + center;
            let end = segment[1] + center;
            draw_line(start.x, start.y, end.x, end.y, 1.0, GREEN);
        }

        let position = state.position.0;
        let ship = world_to_screen(position.x, position.y, meters_per_pixel) + center;
        draw_circle(ship.x, ship.y, 4.0, WHITE);

        let altitude = position.norm() - earth_radius_m;
        draw_text(
            &format!("t = {:.0}s | altitude = {:.0} km", t.0, altitude / 1000.0),
            20.0,
            30.0,
            24.0,
            WHITE,
        );

        next_frame().await;
    }
}

fn world_to_screen(x_m: f64, y_m: f64, meters_per_pixel: f64) -> Vec2 {
    vec2(
        (x_m / meters_per_pixel) as f32,
        -(y_m / meters_per_pixel) as f32,
    )
}
