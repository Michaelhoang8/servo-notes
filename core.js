export const modes = ["none", "conditional", "backCalculation"];
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
export function validate(raw) {
  if (!raw || typeof raw !== "object")
    throw Error("Expected an experiment object.");
  const limits = {
    dt: [0.002, 0.2],
    steps: [50, 5000],
    gain: [0.1, 5],
    tau: [0.05, 10],
    initial: [-10, 10],
    kp: [0, 50],
    ki: [0, 50],
    kd: [0, 20],
    filterTau: [0.002, 2],
    kb: [0, 50],
    minimum: [-10, 10],
    maximum: [-10, 10],
    delay: [0, 20],
    highTarget: [-10, 10],
    returnTarget: [-10, 10],
    stepAt: [0, 4999],
    returnAt: [1, 4999],
    disturbance: [-5, 5],
    disturbanceAt: [0, 4999],
    noise: [0, 1],
    seed: [1, 0xffffffff],
  };
  const integral = new Set([
      "steps",
      "delay",
      "stepAt",
      "returnAt",
      "disturbanceAt",
      "seed",
    ]),
    input = {};
  for (const [key, [min, max]] of Object.entries(limits)) {
    const x = raw[key];
    if (
      typeof x !== "number" ||
      !Number.isFinite(x) ||
      x < min ||
      x > max ||
      (integral.has(key) && !Number.isInteger(x))
    )
      throw Error(
        `${key} must be ${integral.has(key) ? "an integer" : "a number"} from ${min} through ${max}.`,
      );
    input[key] = x;
  }
  if (input.minimum >= input.maximum)
    throw Error("Minimum actuator command must be below maximum.");
  if (
    input.stepAt >= input.returnAt ||
    input.returnAt >= input.steps ||
    input.disturbanceAt >= input.steps
  )
    throw Error("Require stepAt < returnAt < steps and disturbanceAt < steps.");
  return input;
}
export function noiseSequence(input) {
  let x = input.seed >>> 0;
  return Array.from({ length: input.steps }, () => {
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    return (((x >>> 0) / 4294967296) * 2 - 1) * input.noise;
  });
}
export function targetAt(input, k) {
  return k < input.stepAt
    ? 0
    : k < input.returnAt
      ? input.highTarget
      : input.returnTarget;
}
export function simulate(raw, mode) {
  const input = validate(raw);
  if (!modes.includes(mode)) throw Error("Unknown anti-windup mode.");
  const c = input,
    noise = noiseSequence(c),
    decay = Math.exp(-c.dt / c.tau),
    alpha = Math.exp(-c.dt / c.filterTau),
    queue = Array(c.delay).fill(clamp(0, c.minimum, c.maximum));
  let y = c.initial,
    I = 0,
    derivative = 0,
    previousMeasurement = y + noise[0],
    previousCommand = clamp(0, c.minimum, c.maximum),
    reason = null;
  const rows = [];
  for (let k = 0; k < c.steps; k++) {
    const target = targetAt(c, k),
      measurement = y + noise[k],
      error = target - measurement;
    derivative =
      alpha * derivative +
      ((1 - alpha) * (measurement - previousMeasurement)) / c.dt;
    const P = c.kp * error,
      D = -c.kd * derivative,
      previousI = I,
      candidate = I + c.ki * error * c.dt;
    let integralUsed = candidate,
      rawCommand = P + candidate + D,
      command = clamp(rawCommand, c.minimum, c.maximum),
      held = false;
    if (
      mode === "conditional" &&
      ((rawCommand > c.maximum && error > 0) ||
        (rawCommand < c.minimum && error < 0))
    ) {
      integralUsed = previousI;
      rawCommand = P + integralUsed + D;
      command = clamp(rawCommand, c.minimum, c.maximum);
      held = true;
    }
    I =
      mode === "backCalculation"
        ? candidate + c.kb * (command - rawCommand) * c.dt
        : integralUsed;
    if (
      ![I, rawCommand, command, derivative].every(
        (x) => Number.isFinite(x) && Math.abs(x) <= 1e9,
      )
    ) {
      reason = `Controller state exceeded the numerical bound before step ${k + 1}.`;
      break;
    }
    queue.push(command);
    const applied = queue.shift(),
      disturbance = k >= c.disturbanceAt ? c.disturbance : 0;
    const nextY = decay * y + (1 - decay) * c.gain * (applied + disturbance);
    rows.push({
      step: k,
      time: k * c.dt,
      target,
      y,
      measurement,
      error,
      noise: noise[k],
      P,
      integralBefore: previousI,
      integralUsed,
      integralAfter: I,
      D,
      rawCommand,
      command,
      applied,
      disturbance,
      nextY,
      saturated: rawCommand !== command,
      integrationHeld: held,
      variation: Math.abs(command - previousCommand),
    });
    y = nextY;
    previousMeasurement = measurement;
    previousCommand = command;
  }
  const count = rows.length,
    completed = count === c.steps;
  const iae = rows.reduce((s, r) => s + Math.abs(r.target - r.y) * c.dt, 0),
    recoveryIAE = rows
      .filter((r) => r.step >= c.returnAt)
      .reduce((s, r) => s + Math.abs(r.target - r.y) * c.dt, 0);
  return {
    mode,
    completed,
    reason,
    rows,
    metrics: {
      steps: count,
      duration: count * c.dt,
      iae,
      recoveryIAE,
      rmse: count
        ? Math.sqrt(rows.reduce((s, r) => s + (r.target - r.y) ** 2, 0) / count)
        : null,
      peakIntegral: rows.reduce(
        (m, r) => Math.max(m, Math.abs(r.integralAfter)),
        0,
      ),
      saturatedSteps: rows.filter((r) => r.saturated).length,
      commandVariation: rows.reduce((s, r) => s + r.variation, 0),
      finalOutput: y,
      finalError: count ? targetAt(c, count - 1) - y : null,
    },
  };
}
export function analyze(raw) {
  const input = validate(raw);
  return {
    format: "servo-notes-evidence-v1",
    input,
    runs: modes.map((mode) => simulate(input, mode)),
  };
}
export const example = {
  dt: 0.02,
  steps: 1000,
  gain: 1,
  tau: 1.5,
  initial: 0,
  kp: 2,
  ki: 1,
  kd: 0.1,
  filterTau: 0.1,
  kb: 2,
  minimum: 0,
  maximum: 1,
  delay: 2,
  highTarget: 1.5,
  returnTarget: 0.5,
  stepAt: 50,
  returnAt: 500,
  disturbance: -0.2,
  disturbanceAt: 700,
  noise: 0,
  seed: 73,
};
