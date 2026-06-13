use std::net::SocketAddr;
use std::path::{Path, PathBuf};

use axum::extract::Path as AxumPath;
use axum::http::StatusCode;
use axum::routing::get;
use axum::{Json, Router};
use clap::{Parser, Subcommand};
use tower_http::cors::{Any, CorsLayer};
use vast::{
    Environment, EpisodeRecord, LeoCircularEnv, LoadedScenario, MissionSegmentMetrics,
    RecordingError, RunManifest, RunRecorder, list_runs, read_trajectory, run_mission,
    stop_reason_name,
};

#[derive(Parser)]
#[command(name = "runner", about = "VAST RL rollout runner and local API")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run headless episodes and write artifacts under runs/.
    Run {
        /// Scenario TOML file path.
        #[arg(long)]
        scenario: PathBuf,
        /// Number of episodes to roll out.
        #[arg(long, default_value_t = 1)]
        episodes: usize,
        /// Output base directory for run folders.
        #[arg(long, default_value = "runs")]
        output: PathBuf,
    },
    /// Serve run artifacts over a local read-only HTTP API.
    Serve {
        /// Directory containing run folders.
        #[arg(long, default_value = "runs")]
        runs: PathBuf,
        /// TCP port for the API server.
        #[arg(long, default_value_t = 3001)]
        port: u16,
    },
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    match cli.command {
        Command::Run {
            scenario,
            episodes,
            output,
        } => run_command(&scenario, episodes, &output)?,
        Command::Serve { runs, port } => serve_command(&runs, port).await?,
    }
    Ok(())
}

fn run_command(
    scenario_path: &Path,
    episodes: usize,
    output_dir: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let loaded = LoadedScenario::from_file(scenario_path)?;
    match loaded {
        LoadedScenario::Rollout(config) => run_rollout(&config, episodes, output_dir),
        LoadedScenario::Mission(config) => run_mission_command(&config, episodes, output_dir),
    }
}

fn run_rollout(
    config: &vast::ScenarioConfig,
    episodes: usize,
    output_dir: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut env = LeoCircularEnv::from_config(config)?;
    let mut recorder = RunRecorder::new(output_dir, config)?;

    for episode_index in 0..episodes {
        let seed = episode_index as u64;
        env.reset(seed)?;
        let result = env.step([0.0, 0.0, 0.0])?;
        recorder.write_episode(EpisodeRecord {
            episode_index,
            seed,
            steps: result.propagation.steps,
            stop_reason: result.stop_reason,
            final_altitude_m: result.observation.altitude_m,
            reward: result.reward,
            samples: result.propagation.samples,
            mission_segments: None,
        })?;
    }

    println!("Run written to {}", recorder.run_dir().display());
    Ok(())
}

fn run_mission_command(
    config: &vast::MissionScenarioConfig,
    episodes: usize,
    output_dir: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut recorder = RunRecorder::new_mission(output_dir, config)?;

    for episode_index in 0..episodes {
        let seed = episode_index as u64;
        let _ = seed;
        let result = run_mission(config)?;

        let final_radius = result.final_state.position.vector().norm();
        let final_altitude_m = final_radius - vast::R_EARTH;
        let total_steps: usize = result
            .segments
            .iter()
            .map(|segment| segment.propagation.steps)
            .sum();
        let final_stop_reason = result
            .segments
            .last()
            .map(|segment| segment.stop_reason)
            .unwrap_or(vast::StopReason::DurationMet);

        let mission_segments = result
            .segments
            .iter()
            .map(|segment| MissionSegmentMetrics {
                segment_index: segment.segment_index,
                steps: segment.propagation.steps,
                stop_reason: stop_reason_name(segment.stop_reason).to_string(),
                burns_applied: segment.burns_applied,
            })
            .collect();

        recorder.write_episode(EpisodeRecord {
            episode_index,
            seed,
            steps: total_steps,
            stop_reason: final_stop_reason,
            final_altitude_m,
            reward: 0.0,
            samples: result.samples,
            mission_segments: Some(mission_segments),
        })?;
    }

    println!("Mission run written to {}", recorder.run_dir().display());
    Ok(())
}

async fn serve_command(runs_dir: &Path, port: u16) -> Result<(), Box<dyn std::error::Error>> {
    let runs_dir = runs_dir.to_path_buf();
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    let app = Router::new()
        .route("/api/runs", get(list_runs_handler))
        .route("/api/runs/{run_id}", get(get_run_handler))
        .route(
            "/api/runs/{run_id}/episodes/{episode_index}/trajectory",
            get(get_trajectory_handler),
        )
        .layer(cors)
        .with_state(AppState { runs_dir });

    let addr = SocketAddr::from(([127, 0, 0, 1], port));
    println!("Serving runs API at http://{addr}");
    let listener = tokio::net::TcpListener::bind(addr).await?;
    axum::serve(listener, app).await?;
    Ok(())
}

#[derive(Clone)]
struct AppState {
    runs_dir: PathBuf,
}

async fn list_runs_handler(
    axum::extract::State(state): axum::extract::State<AppState>,
) -> Result<Json<Vec<RunManifest>>, ApiError> {
    Ok(Json(list_runs(&state.runs_dir)?))
}

async fn get_run_handler(
    axum::extract::State(state): axum::extract::State<AppState>,
    AxumPath(run_id): AxumPath<String>,
) -> Result<Json<RunManifest>, ApiError> {
    let manifest_path = state.runs_dir.join(&run_id).join("manifest.json");
    if !manifest_path.exists() {
        return Err(ApiError::NotFound);
    }
    let file = std::fs::File::open(manifest_path)?;
    Ok(Json(serde_json::from_reader(file)?))
}

async fn get_trajectory_handler(
    axum::extract::State(state): axum::extract::State<AppState>,
    AxumPath((run_id, episode_index)): AxumPath<(String, usize)>,
) -> Result<Json<Vec<vast::TrajectoryPoint>>, ApiError> {
    let run_dir = state.runs_dir.join(run_id);
    if !run_dir.exists() {
        return Err(ApiError::NotFound);
    }
    Ok(Json(read_trajectory(&run_dir, episode_index)?))
}

enum ApiError {
    NotFound,
    Recording(RecordingError),
    Json(serde_json::Error),
    Io(std::io::Error),
}

impl From<RecordingError> for ApiError {
    fn from(value: RecordingError) -> Self {
        Self::Recording(value)
    }
}

impl From<serde_json::Error> for ApiError {
    fn from(value: serde_json::Error) -> Self {
        Self::Json(value)
    }
}

impl From<std::io::Error> for ApiError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

impl axum::response::IntoResponse for ApiError {
    fn into_response(self) -> axum::response::Response {
        match self {
            Self::NotFound => (StatusCode::NOT_FOUND, "not found").into_response(),
            Self::Recording(error) => {
                (StatusCode::INTERNAL_SERVER_ERROR, error.to_string()).into_response()
            }
            Self::Json(error) => {
                (StatusCode::INTERNAL_SERVER_ERROR, error.to_string()).into_response()
            }
            Self::Io(error) => {
                (StatusCode::INTERNAL_SERVER_ERROR, error.to_string()).into_response()
            }
        }
    }
}
