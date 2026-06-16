import React from "react";
import {
  Stack,
  Row,
  Grid,
  Card,
  CardHeader,
  CardBody,
  H1,
  H2,
  H3,
  Text,
  Pill,
  Divider,
  Table,
  Callout,
  Stat,
  Code,
  useHostTheme,
  mergeStyle,
} from "cursor/canvas";

export default function ExamplesDevExReview() {
  const theme = useHostTheme();

  // Custom styles using host theme tokens
  const containerStyle = {
    padding: "24px",
    backgroundColor: theme.background.primary,
    minHeight: "100vh",
    color: theme.text.primary,
    fontFamily: "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif",
  };

  const codeBlockStyle = {
    backgroundColor: theme.background.secondary,
    padding: "12px",
    borderRadius: "6px",
    border: `1px solid ${theme.stroke.secondary}`,
    fontFamily: "ui-monospace, SFMono-Regular, SF Mono, Menlo, Consolas, Liberation Mono, monospace",
    fontSize: "13px",
    lineHeight: "1.5",
    overflowX: "auto" as const,
  };

  return (
    <div style={containerStyle}>
      <Stack gap={24}>
        {/* Header Section */}
        <Row justify="space-between" align="center">
          <Stack gap={4}>
            <H1>VAST Examples & DevEx Audit</H1>
            <Text tone="secondary" size="body">
              A comprehensive review of developer experience, API ergonomics, and correctness in the VAST examples.
            </Text>
          </Stack>
          <Pill tone="warning" size="lg">Review Completed</Pill>
        </Row>

        <Divider />

        {/* Stats Grid */}
        <Grid columns={4} gap={16}>
          <Card>
            <CardBody>
              <Stat
                value="5"
                label="Total Examples Reviewed"
                tone="neutral"
              />
            </CardBody>
          </Card>
          <Card>
            <CardBody>
              <Stat
                value="4 / 5"
                label="Examples with Critical Bug"
                tone="danger"
              />
            </CardBody>
          </Card>
          <Card>
            <CardBody>
              <Stat
                value="Medium-High"
                label="Ergonomic Friction"
                tone="warning"
              />
            </CardBody>
          </Card>
          <Card>
            <CardBody>
              <Stat
                value="Excellent"
                label="Physical Fidelity Potential"
                tone="success"
              />
            </CardBody>
          </Card>
        </Grid>

        {/* Main Content Grid */}
        <Grid columns="1fr 1fr" gap={24}>
          {/* Left Column: Correctness & Bugs */}
          <Stack gap={24}>
            <Card>
              <CardHeader>
                <Row justify="space-between" align="center" style={{ width: "100%" }}>
                  <Text weight="semibold">CRITICAL CORRECTNESS BUG: Altitude vs. Radius</Text>
                  <Pill tone="danger">High Priority</Pill>
                </Row>
              </CardHeader>
              <CardBody>
                <Stack gap={16}>
                  <Callout tone="danger">
                    <Text weight="semibold">Double-Adding Earth's Radius (R_EARTH)</Text>
                    <Text size="small" tone="secondary" style={{ marginTop: 4 }}>
                      In 4 out of 5 examples, the code defines a constant <Code>ALTITUDE_KM</Code> as <Code>(R_EARTH + target_altitude) / 1000.0</Code> (which is actually the orbital radius). It then passes this value multiplied by 1000 to <Code>Orbit::CircularEquatorial</Code> or <Code>Orbit::CircularInclined</Code>'s <Code>altitude</Code> field.
                    </Text>
                  </Callout>

                  <Text>
                    The <Code>Orbit</Code> constructors internally add <Code>R_EARTH</Code> to the provided <Code>altitude</Code> to compute the semi-major axis. Because the examples passed the orbital radius as the altitude, <Code>R_EARTH</Code> is added twice!
                  </Text>

                  <div style={codeBlockStyle}>
                    {`// What the code does:
const ALTITUDE_KM: f64 = (R_EARTH + 250_000.0) / 1000.0; // = 6628.137 km (Radius)
altitude: Distance::new(ALTITUDE_KM * 1000.0) // = 6,628,137 m passed as altitude

// What the library does internally (Orbit::CircularEquatorial):
let semi_major_axis = R_EARTH + altitude; // = 6,378,137 + 6,628,137 = 13,006,274 m!
// Result: Orbit is at 13,006 km radius (~6,628 km altitude) instead of 250 km altitude!`}
                  </div>

                  <H3>Physical Consequences</H3>
                  <Table
                    headers={["Example File", "Intended Altitude", "Actual Simulated Altitude", "Physical Impact"]}
                    rows={[
                      [
                        "drag_decay.rs",
                        "250 km",
                        "6,628 km",
                        "Atmospheric density is 0 at 6,600km. Drag decay is completely non-existent (0.000 km decay over 6 hours), masking the drag physics!"
                      ],
                      [
                        "inclined_orbit.rs",
                        "600 km",
                        "6,978 km",
                        "Simulates a medium Earth orbit instead of a Low Earth Orbit (LEO)."
                      ],
                      [
                        "orbit_period.rs",
                        "400 km",
                        "6,778 km",
                        "Integrator comparisons are run at double the intended altitude, reducing gravity gradients and errors."
                      ],
                      [
                        "j2_precession.rs",
                        "600 km",
                        "6,978 km",
                        "J2 precession rate is significantly weaker at 13,356 km radius, though the analytical comparison still matches because both use the same radius."
                      ]
                    ]}
                  />
                </Stack>
              </CardBody>
            </Card>

            <Card>
              <CardHeader>
                <Text weight="semibold">Example Review & Analysis</Text>
              </CardHeader>
              <CardBody>
                <Stack gap={16}>
                  <Text weight="semibold">1. inclined_orbit.rs (Circular Inclined Orbit)</Text>
                  <Text size="small" tone="secondary">
                    Demonstrates circular inclined orbit setup and simple drift analysis. Good baseline, but suffers from the R_EARTH bug and has redundant double parentheses in constructors.
                  </Text>

                  <Text weight="semibold">2. orbit_period.rs (Integrator Sweep)</Text>
                  <Text size="small" tone="secondary">
                    Excellent physical comparison of Euler, RK4, Verlet, and Leapfrog integrators. Shows energy and angular momentum conservation properties. Highly educational!
                  </Text>

                  <Text weight="semibold">3. j2_precession.rs (Oblate Body J2 Drift)</Text>
                  <Text size="small" tone="secondary">
                    Compares two-body gravity with J2 oblate gravity. Compares simulated RAAN drift against analytical predictions. Excellent verification example.
                  </Text>

                  <Text weight="semibold">4. drag_decay.rs (Atmospheric Drag)</Text>
                  <Text size="small" tone="secondary">
                    Combines two-body gravity and exponential atmospheric drag using <Code>CompositeDynamics</Code>. Completely broken physically due to the R_EARTH bug (shows 0 decay).
                  </Text>

                  <Text weight="semibold">5. orbit_viz.rs (Macroquad Visualization)</Text>
                  <Text size="small" tone="secondary">
                    The only example that defines altitude correctly (400 km). However, it is extremely low-level, requiring manual buffer management and integrator stepping.
                  </Text>
                </Stack>
              </CardBody>
            </Card>
          </Stack>

          {/* Right Column: Ergonomics & Recommendations */}
          <Stack gap={24}>
            <Card>
              <CardHeader>
                <Text weight="semibold">ERGONOMIC FRICTION POINTS (User Complaints)</Text>
              </CardHeader>
              <CardBody>
                <Stack gap={16}>
                  <H3>1. Verbose Type Safety & Boilerplate</H3>
                  <Text>
                    While type-safe newtypes like <Code>Distance</Code>, <Code>Angle</Code>, <Code>TimeStep</Code>, and <Code>Duration</Code> prevent unit and coordinate system errors, they introduce massive verbosity in user code:
                  </Text>
                  <div style={codeBlockStyle}>
                    {`let spec = Orbit::CircularInclined {
    altitude: Distance::new(ALTITUDE_KM * 1000.0)?,
    inclination: Angle::from_degrees(INCLINATION_DEG)?,
    raan: Angle::from_degrees(RAAN_DEG)?,
};`}
                  </div>
                  <Text size="small" tone="secondary">
                    Every single scalar must be wrapped in a <Code>Result</Code>-returning constructor, forcing <Code>?</Code> or <Code>.unwrap()</Code> everywhere.
                  </Text>

                  <Divider />

                  <H3>2. Inconsistent Unit Accessors</H3>
                  <Text>
                    There is a jarring difference between how units are accessed in the time series versus diagnostics:
                  </Text>
                  <Grid columns={2} gap={12}>
                    <Stack gap={4}>
                      <Text weight="semibold" size="small">DiagnosticSeries (Raw f64):</Text>
                      <Code>s.column(Column::Radius)[0]</Code>
                    </Stack>
                    <Stack gap={4}>
                      <Text weight="semibold" size="small">OrbitalDiagnostics (Newtype):</Text>
                      <Code>diagnostics.radius().meters()</Code>
                    </Stack>
                  </Grid>
                  <Text size="small" tone="secondary">
                    Users will complain about having to remember which API returns raw <Code>f64</Code> meters and which returns a typed <Code>Distance</Code> requiring <Code>.meters()</Code>.
                  </Text>

                  <Divider />

                  <H3>3. DiagnosticSeries Naming Confusion</H3>
                  <Text>
                    The <Code>series</Code> field on <Code>SimResult</Code> is called a "series" and has a <Code>column</Code> method, but it is hardcoded to only contain exactly <Code>2</Code> elements (initial and final).
                  </Text>
                  <Text size="small" tone="secondary">
                    Calling a pair of endpoints a "series" is highly misleading. Users will expect a full step-by-step history of the simulation when they access <Code>result.series</Code>.
                  </Text>

                  <Divider />

                  <H3>4. Low-Level Custom Loop Boilerplate</H3>
                  <Text>
                    If a user wants to run a custom loop (e.g. for visualization or custom control policies), they are forced to drop down to the extremely verbose low-level API shown in <Code>orbit_viz.rs</Code>:
                  </Text>
                  <div style={codeBlockStyle}>
                    {`self.integrator.step(
    &self.earth,
    &mut self.states.view_mut(),
    0,
    self.t,
    self.dt,
)?;
self.t = self.t.advance_by(self.dt);
let state = self.states.view().read_state(0)?;`}
                  </div>
                </Stack>
              </CardBody>
            </Card>

            <Card>
              <CardHeader>
                <Text weight="semibold">RECOMMENDED IMPROVEMENTS</Text>
              </CardHeader>
              <CardBody>
                <Stack gap={16}>
                  <H3>1. Fix the Altitude vs. Radius Bug</H3>
                  <Text size="small">
                    Update the 4 affected examples to define <Code>ALTITUDE_KM</Code> as a simple altitude (e.g., <Code>250.0</Code>, <Code>600.0</Code>, <Code>400.0</Code>) and pass it directly. This will also fix the <Code>drag_decay.rs</Code> physics so that drag is actually visible!
                  </Text>

                  <H3>2. Introduce Ergonomic Builder Helpers</H3>
                  <Text size="small">
                    Add helper methods or let <Code>SimBuilder</Code> accept raw numbers for common units (e.g., <Code>altitude_km(600.0)</Code>, <Code>timestep_seconds(10.0)</Code>, <Code>duration_days(30.0)</Code>). This keeps the internal types safe while making the public-facing API beautiful and clean.
                  </Text>

                  <H3>3. Rename or Expand SimResult's Series</H3>
                  <Text size="small">
                    Rename <Code>SimResult::series</Code> to <Code>endpoints</Code> or <Code>initial_final</Code> if it only contains 2 elements. Alternatively, allow <Code>SimBuilder</Code> to configure a recording cadence so that <Code>series</Code> actually contains the full trajectory history.
                  </Text>

                  <H3>4. Provide an Intermediate Step Iterator API</H3>
                  <Text size="small">
                    Add a step-by-step iterator to <Code>Sim</Code> (e.g., <Code>sim.steps()</Code> or a callback <Code>sim.step_with(|state, t| { ... })</Code>) so that custom loops (like visualization or RL environment stepping) don't require manual buffer view management.
                  </Text>

                  <H3>5. Clean up Code Quality Issues</H3>
                  <Text size="small">
                    Remove redundant double parentheses like <Code>((ALTITUDE_KM) * 1000.0)</Code>. Replace <Code>Box&lt;dyn std::error::Error&gt;</Code> in examples with <Code>anyhow</Code> or propagate the specific <Code>PropagationError</Code> to align with Rust guidelines.
                  </Text>
                </Stack>
              </CardBody>
            </Card>
          </Stack>
        </Grid>
      </Stack>
    </div>
  );
}
