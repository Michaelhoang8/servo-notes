import test from "node:test";
import assert from "node:assert/strict";
import {
  analyze,
  simulate,
  validate,
  noiseSequence,
  example,
} from "../core.js";
const near = (a, b, t = 1e-10) =>
  assert.ok(Math.abs(a - b) < t, `${a} != ${b}`);
test("uncontrolled plant decay matches the continuous closed form at every sample", () => {
  const c = {
    ...example,
    kp: 0,
    ki: 0,
    kd: 0,
    kb: 0,
    initial: 3,
    minimum: -1,
    maximum: 1,
    disturbance: 0,
    delay: 0,
  };
  const r = simulate(c, "none");
  for (const row of r.rows) {
    near(row.y, 3 * Math.exp(-row.time / c.tau));
    near(row.nextY, 3 * Math.exp(-(row.time + c.dt) / c.tau));
  }
});
test("constant actuator input follows the exact zero-order-hold step response", () => {
  const c = {
    ...example,
    kp: 50,
    ki: 0,
    kd: 0,
    kb: 0,
    highTarget: 10,
    returnTarget: 10,
    stepAt: 0,
    disturbance: 0,
    delay: 0,
  };
  const r = simulate(c, "none");
  for (const row of r.rows) {
    near(row.command, 1);
    near(row.nextY, c.gain * (1 - Math.exp(-(row.time + c.dt) / c.tau)));
  }
});
test("integer command delay postpones the first plant response by exactly d samples", () => {
  const c = {
    ...example,
    kp: 50,
    ki: 0,
    kd: 0,
    kb: 0,
    highTarget: 10,
    returnTarget: 10,
    stepAt: 0,
    disturbance: 0,
    delay: 7,
  };
  const r = simulate(c, "none");
  for (let k = 0; k < 7; k++) near(r.rows[k].nextY, 0);
  assert.ok(r.rows[7].nextY > 0);
  for (let k = 7; k < 100; k++) near(r.rows[k].applied, r.rows[k - 7].command);
});
test("derivative acts on measurement, avoiding setpoint derivative kick", () => {
  const c = {
    ...example,
    kp: 0,
    ki: 0,
    kd: 1,
    disturbance: 0,
    noise: 0,
    initial: 0,
  };
  const r = simulate(c, "none");
  r.rows.forEach((row) => near(row.D, 0));
  assert.equal(r.rows[50].target, 1.5);
});
test("conditional integration freezes outward accumulation under saturation", () => {
  const c = {
    ...example,
    kp: 2,
    ki: 1,
    kd: 0,
    highTarget: 10,
    returnTarget: 10,
    stepAt: 0,
    disturbance: 0,
  };
  const r = simulate(c, "conditional");
  for (const row of r.rows) {
    assert.equal(row.integrationHeld, true);
    near(row.integralAfter, 0);
    near(row.command, 1);
  }
});
test("back-calculation applies the documented discrete feedback recurrence", () => {
  const r = simulate(example, "backCalculation");
  for (const row of r.rows)
    near(
      row.integralAfter,
      row.integralBefore +
        example.ki * row.error * example.dt +
        example.kb * (row.command - row.rawCommand) * example.dt,
    );
});
test("zero back-calculation gain is identical to ordinary integration", () => {
  const a = simulate({ ...example, kb: 0 }, "none"),
    b = simulate({ ...example, kb: 0 }, "backCalculation");
  assert.deepEqual(a.rows, b.rows);
  assert.deepEqual(a.metrics, b.metrics);
});
test("all variants agree when there is no saturation", () => {
  const c = {
    ...example,
    highTarget: 0.2,
    returnTarget: 0.1,
    minimum: -10,
    maximum: 10,
    disturbance: 0,
  };
  const r = analyze(c);
  assert.deepEqual(r.runs[0].rows, r.runs[2].rows);
  assert.deepEqual(r.runs[0].rows, r.runs[1].rows);
});
test("default anti-windup variants improve recovery error under unreachable initial target", () => {
  const r = analyze(example);
  assert.ok(r.runs[1].metrics.recoveryIAE < r.runs[0].metrics.recoveryIAE);
  assert.ok(r.runs[2].metrics.recoveryIAE < r.runs[0].metrics.recoveryIAE);
  assert.ok(r.runs.every((x) => x.completed));
});
test("seeded noise is reproducible and shared across all variants", () => {
  const c = { ...example, noise: 0.1 };
  const r = analyze(c),
    noise = noiseSequence(c);
  assert.deepEqual(r, analyze(c));
  assert.notDeepEqual(noise, noiseSequence({ ...c, seed: 74 }));
  for (const run of r.runs)
    assert.deepEqual(
      run.rows.map((r) => r.noise),
      noise,
    );
  assert.ok(noise.every((x) => Math.abs(x) <= 0.1));
});
test("metric accounting uses true tracking error and interval-aligned samples", () => {
  const r = simulate({ ...example, noise: 0.2 }, "none");
  near(
    r.metrics.iae,
    r.rows.reduce((s, x) => s + Math.abs(x.target - x.y) * example.dt, 0),
  );
  near(r.metrics.finalOutput, r.rows.at(-1).nextY);
  assert.equal(r.metrics.steps, 1000);
  near(r.metrics.duration, 20);
  assert.ok(r.rows.every((x) => x.command >= 0 && x.command <= 1));
});
test("unstable controller arithmetic stops explicitly with a finite valid prefix", () => {
  const c = {
    ...example,
    dt: 0.2,
    kb: 50,
    kp: 50,
    highTarget: 10,
    returnTarget: 10,
    stepAt: 0,
  };
  const r = simulate(c, "backCalculation");
  assert.equal(r.completed, false);
  assert.match(r.reason, /numerical bound/);
  assert.ok(r.rows.length > 0 && r.rows.length < c.steps);
  assert.doesNotThrow(() => JSON.stringify(r));
});
test("strict input validation rejects ambiguous times, gains and limits", () => {
  for (const change of [
    { steps: 49 },
    { dt: 0 },
    { kp: true },
    { minimum: 1, maximum: 0 },
    { stepAt: 500, returnAt: 500 },
    { returnAt: 1000 },
    { disturbanceAt: 1000 },
    { seed: 0 },
    { delay: 1.5 },
  ])
    assert.throws(() => validate({ ...example, ...change }));
  assert.throws(() => simulate(example, "other"));
  const before = structuredClone(example);
  analyze(example);
  assert.deepEqual(before, example);
});
