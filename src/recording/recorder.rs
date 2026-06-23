use std::fs::{self, File};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::errors::RecordingError;
use crate::sim::config::ScenarioConfig;
use crate::sim::engine::TrajectorySample;
use crate::sim::termination::StopReason;

const ARTIFACT_VERSION: u32 = 1;

/// Metadata for a completed run directory.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RunManifest {
    /// Artifact schema version.
    pub version: u32,
    /// Unique run identifier.
    pub run_id: String,
    /// ISO-8601 creation timestamp.
    pub created_at: String,
    /// Scenario name.
    pub scenario_name: String,
    /// Full rollout scenario configuration snapshot.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scenario: Option<ScenarioConfig>,
    /// Full mission scenario configuration snapshot.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission: Option<crate::sim::config::MissionScenarioConfig>,
    /// Number of episodes recorded.
    pub episode_count: usize,
}

/// Per-episode data written to run artifacts.
#[derive(Debug, Clone, PartialEq)]
pub struct EpisodeRecord {
    /// Zero-based episode index.
    pub episode_index: usize,
    /// Environment seed.
    pub seed: u64,
    /// Integrator steps taken.
    pub steps: usize,
    /// Why propagation stopped.
    pub stop_reason: StopReason,
    /// Final altitude in meters.
    pub final_altitude_m: f64,
    /// Episode reward.
    pub reward: f64,
    /// Sampled trajectory points.
    pub samples: Vec<TrajectorySample>,
    /// Mission segment summaries when applicable.
    pub mission_segments: Option<Vec<MissionSegmentMetrics>>,
}

/// Per-episode summary line in `metrics.jsonl`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EpisodeMetrics {
    /// Zero-based episode index.
    pub episode_index: usize,
    /// Environment seed used for the episode.
    pub seed: u64,
    /// Integrator steps taken.
    pub steps: usize,
    /// Stop reason name.
    pub stop_reason: String,
    /// Final altitude in meters.
    pub final_altitude_m: f64,
    /// Episode reward (zero in MVP).
    pub reward: f64,
    /// Mission segment summaries when the episode came from a mission run.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mission_segments: Option<Vec<MissionSegmentMetrics>>,
}

/// Per-segment summary for mission episodes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissionSegmentMetrics {
    /// Zero-based segment index.
    pub segment_index: usize,
    /// Integrator steps taken in the segment.
    pub steps: usize,
    /// Stop reason name.
    pub stop_reason: String,
    /// Burns applied after the segment completed.
    pub burns_applied: usize,
}

/// One sampled trajectory point in `trajectory.jsonl`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    /// Simulation time in seconds.
    pub t: f64,
    /// Position components in meters.
    pub pos: [f64; 3],
    /// Velocity components in meters per second.
    pub vel: [f64; 3],
    /// Mission segment index when recorded by a mission runner.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub segment_index: Option<usize>,
}

impl TrajectoryPoint {
    fn from_sample(sample: TrajectorySample) -> Self {
        let position = sample.state.position.vector();
        let velocity = sample.state.velocity.vector();
        Self {
            t: sample.epoch.seconds(),
            pos: [position.x, position.y, position.z],
            vel: [velocity.x, velocity.y, velocity.z],
            segment_index: sample.segment_index,
        }
    }
}

/// Writes run artifacts under a run directory.
pub struct RunRecorder {
    run_dir: PathBuf,
    manifest: RunManifest,
    episodes_written: usize,
}

impl RunRecorder {
    /// Creates a new run directory and manifest for a rollout scenario.
    pub fn new(base_dir: &Path, scenario: &ScenarioConfig) -> Result<Self, RecordingError> {
        Self::new_with_manifest(
            base_dir,
            scenario.name.clone(),
            Some(scenario.clone()),
            None,
        )
    }

    /// Creates a new run directory and manifest for a mission scenario.
    pub fn new_mission(
        base_dir: &Path,
        scenario: &crate::sim::config::MissionScenarioConfig,
    ) -> Result<Self, RecordingError> {
        Self::new_with_manifest(
            base_dir,
            scenario.name.clone(),
            None,
            Some(scenario.clone()),
        )
    }

    fn new_with_manifest(
        base_dir: &Path,
        scenario_name: String,
        scenario: Option<ScenarioConfig>,
        mission: Option<crate::sim::config::MissionScenarioConfig>,
    ) -> Result<Self, RecordingError> {
        let run_id = generate_run_id();
        let run_dir = base_dir.join(&run_id);
        fs::create_dir_all(run_dir.join("episodes"))?;

        let manifest = RunManifest {
            version: ARTIFACT_VERSION,
            run_id: run_id.clone(),
            created_at: iso_timestamp(),
            scenario_name,
            scenario,
            mission,
            episode_count: 0,
        };

        let recorder = Self {
            run_dir,
            manifest,
            episodes_written: 0,
        };
        recorder.write_manifest()?;
        Ok(recorder)
    }

    /// Opens an existing run directory for reading.
    pub fn open(run_dir: &Path) -> Result<Self, RecordingError> {
        let manifest_path = run_dir.join("manifest.json");
        let manifest = read_manifest(&manifest_path)?;
        Ok(Self {
            run_dir: run_dir.to_path_buf(),
            manifest: manifest.clone(),
            episodes_written: manifest.episode_count,
        })
    }

    /// Returns the run directory path.
    #[must_use]
    pub fn run_dir(&self) -> &Path {
        &self.run_dir
    }

    /// Returns the run manifest.
    #[must_use]
    pub fn manifest(&self) -> &RunManifest {
        &self.manifest
    }

    /// Records one episode's trajectory and metrics.
    pub fn write_episode(&mut self, episode: EpisodeRecord) -> Result<(), RecordingError> {
        let episode_dir = self
            .run_dir
            .join("episodes")
            .join(format!("{:04}", episode.episode_index));
        fs::create_dir_all(&episode_dir)?;

        let trajectory_path = episode_dir.join("trajectory.jsonl");
        let mut trajectory_file = File::create(trajectory_path)?;
        for sample in &episode.samples {
            let point = TrajectoryPoint::from_sample(sample.clone());
            serde_json::to_writer(&mut trajectory_file, &point)?;
            trajectory_file.write_all(b"\n")?;
        }

        let metrics = EpisodeMetrics {
            episode_index: episode.episode_index,
            seed: episode.seed,
            steps: episode.steps,
            stop_reason: stop_reason_name(episode.stop_reason).to_string(),
            final_altitude_m: episode.final_altitude_m,
            reward: episode.reward,
            mission_segments: episode.mission_segments,
        };
        append_jsonl(&self.run_dir.join("metrics.jsonl"), &metrics)?;

        self.episodes_written += 1;
        self.manifest.episode_count = self.episodes_written;
        self.write_manifest()?;
        Ok(())
    }

    fn write_manifest(&self) -> Result<(), RecordingError> {
        let manifest_path = self.run_dir.join("manifest.json");
        let json = serde_json::to_string_pretty(&self.manifest)?;
        fs::write(manifest_path, json)?;
        Ok(())
    }
}

/// Lists manifests for run directories under `base_dir`.
pub fn list_runs(base_dir: &Path) -> Result<Vec<RunManifest>, RecordingError> {
    let mut manifests = Vec::new();
    if !base_dir.exists() {
        return Ok(manifests);
    }

    for entry in fs::read_dir(base_dir)? {
        let entry = entry?;
        if entry.file_type()?.is_dir() {
            let manifest_path = entry.path().join("manifest.json");
            if manifest_path.exists() {
                manifests.push(read_manifest(&manifest_path)?);
            }
        }
    }

    manifests.sort_by(|left, right| right.created_at.cmp(&left.created_at));
    Ok(manifests)
}

/// Reads trajectory points for an episode.
pub fn read_trajectory(
    run_dir: &Path,
    episode_index: usize,
) -> Result<Vec<TrajectoryPoint>, RecordingError> {
    let trajectory_path = run_dir
        .join("episodes")
        .join(format!("{episode_index:04}"))
        .join("trajectory.jsonl");
    read_jsonl(&trajectory_path)
}

fn read_manifest(path: &Path) -> Result<RunManifest, RecordingError> {
    let file = File::open(path)?;
    Ok(serde_json::from_reader(file)?)
}

fn append_jsonl<T: Serialize>(path: &Path, value: &T) -> Result<(), RecordingError> {
    let mut file = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)?;
    serde_json::to_writer(&mut file, value)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn read_jsonl<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<Vec<T>, RecordingError> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let mut values = Vec::new();
    for line in reader.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        values.push(serde_json::from_str(&line)?);
    }
    Ok(values)
}

fn generate_run_id() -> String {
    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|duration| duration.as_secs())
        .unwrap_or(0);
    let suffix = (timestamp % 10_000) as u32;
    format!("{timestamp}-{suffix:04}")
}

fn iso_timestamp() -> String {
    // Stable enough for MVP manifests without adding a chrono dependency.
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|duration| duration.as_secs())
        .unwrap_or(0);
    format!("{secs}")
}

/// Returns a stable artifact name for a stop reason.
pub fn stop_reason_name(reason: StopReason) -> &'static str {
    match reason {
        StopReason::DurationMet => "duration_met",
        StopReason::StepCountMet => "step_count_met",
        StopReason::StepBudgetExceeded => "step_budget_exceeded",
        StopReason::ApoapsisMet => "apoapsis_met",
        StopReason::PeriapsisMet => "periapsis_met",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::frame::Frame;
    use crate::{CartesianState, Distance, Epoch, Orbit, TrajectorySample};

    fn sample_state() -> CartesianState {
        Orbit::circular_equatorial(Distance::new(400_000.0))
            .to_cartesian_state(&Frame::earth_inertial())
            .expect("state")
    }

    #[test]
    fn writes_manifest_metrics_and_trajectory() {
        let temp = std::env::temp_dir().join(format!("vast-recording-{}", std::process::id()));
        let _ = fs::remove_dir_all(&temp);
        fs::create_dir_all(&temp).expect("temp dir");

        let scenario = ScenarioConfig::from_toml(
            r#"
            name = "test"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = { kind = "two_body" }
            sample_every_steps = 5
            "#,
        )
        .expect("scenario");

        let mut recorder = RunRecorder::new(&temp, &scenario).expect("recorder");
        let samples = vec![TrajectorySample::new(Epoch::ZERO, sample_state(), None)];
        recorder
            .write_episode(EpisodeRecord {
                episode_index: 0,
                seed: 1,
                steps: 10,
                stop_reason: StopReason::DurationMet,
                final_altitude_m: 400_000.0,
                reward: 0.0,
                samples,
                mission_segments: None,
            })
            .expect("write episode");

        let trajectory = read_trajectory(recorder.run_dir(), 0).expect("trajectory");
        assert_eq!(trajectory.len(), 1);
        let runs = list_runs(&temp).expect("list");
        assert_eq!(runs.len(), 1);

        let _ = fs::remove_dir_all(temp);
    }
}
