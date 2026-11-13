import { example, validate } from "./core.js";
const $ = (id) => document.getElementById(id),
  NS = "http://www.w3.org/2000/svg";
const names = {
    none: "Ordinary PID",
    conditional: "Conditional integration",
    backCalculation: "Back-calculation",
  },
  colors = {
    none: "#697574",
    conditional: "#b77b52",
    backCalculation: "#47796d",
  };
let result = null,
  worker = null,
  timer = null;
const el = (tag, text, cls) => {
  const n = document.createElement(tag);
  if (text !== undefined) n.textContent = text;
  if (cls) n.className = cls;
  return n;
};
const num = (x) =>
  x === null
    ? "Unavailable"
    : Math.abs(x) >= 10000
      ? x.toExponential(3)
      : x.toFixed(4);
function shape(tag, attrs, text) {
  const n = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, String(v));
  if (text !== undefined) n.textContent = text;
  return n;
}
function stop() {
  clearInterval(timer);
  timer = null;
  $("play").textContent = "Play cursor";
}
function run(raw) {
  let input;
  try {
    input = validate(raw);
  } catch (e) {
    $("status").textContent = e.message;
    return;
  }
  stop();
  worker?.terminate();
  $("result").hidden = true;
  $("status").textContent =
    "Simulating three controllers with a shared disturbance and noise schedule…";
  worker = new Worker("./worker.js", { type: "module" });
  worker.onmessage = ({ data }) => {
    worker.terminate();
    worker = null;
    if (data.error) {
      $("status").textContent = data.error;
      return;
    }
    result = data.result;
    $("result").hidden = false;
    $("json").value = JSON.stringify(input, null, 2);
    for (const key of ["kp", "ki", "kd", "kb"]) $(key).value = input[key];
    $("step").max = input.steps - 1;
    $("step").value = input.returnAt;
    $("status").textContent = result.runs.every((r) => r.completed)
      ? `Completed all three ${input.steps}-step runs. Duration ${(input.steps * input.dt).toFixed(3)} seconds; identical seeded noise across variants.`
      : result.runs
          .filter((r) => !r.completed)
          .map(
            (r) =>
              `${names[r.mode]}: ${r.reason} Metrics describe its valid prefix.`,
          )
          .join(" ");
    $("model-note").textContent =
      `Plant gain ${input.gain}, τ ${input.tau}s · dt ${input.dt}s · command delay ${input.delay} samples`;
    $("metrics").replaceChildren(
      ...result.runs.map((r) => {
        const tr = el("tr");
        tr.append(
          el(
            "td",
            `${names[r.mode]}${r.completed ? "" : " (partial)"}`,
            r.mode,
          ),
        );
        for (const [index, x] of [
          r.metrics.iae,
          r.metrics.recoveryIAE,
          r.metrics.rmse,
          r.metrics.saturatedSteps,
          r.metrics.peakIntegral,
          r.metrics.finalError,
        ].entries())
          tr.append(
            el(
              "td",
              index === 3
                ? String(x)
                : typeof x === "number"
                  ? num(x)
                  : "Unavailable",
            ),
          );
        return tr;
      }),
    );
    drawChart();
    inspect();
  };
  worker.onerror = () => {
    $("status").textContent = "The simulation worker failed. Reload and retry.";
    worker?.terminate();
    worker = null;
  };
  worker.postMessage(input);
}
function drawChart() {
  const chart = $("chart");
  chart.replaceChildren();
  chart.append(shape("rect", { width: 1000, height: 680, fill: "#fafcf8" }));
  chart.append(
    shape("title", {}, "Servo Notes: output, command and integral comparison"),
  );
  const duration = result.input.steps * result.input.dt,
    X = (t) => 70 + (t / duration) * 890;
  const panels = [
    { key: "y", label: "TRUE PLANT OUTPUT", top: 65, height: 210 },
    { key: "command", label: "ACTUATOR COMMAND", top: 340, height: 110 },
    {
      key: "integralAfter",
      label: "INTEGRAL STATE AFTER UPDATE",
      top: 515,
      height: 110,
    },
  ];
  Object.entries(names).forEach(([mode, name], i) => {
    chart.append(
      shape("line", {
        x1: 70 + i * 280,
        y1: 20,
        x2: 90 + i * 280,
        y2: 20,
        stroke: colors[mode],
        "stroke-width": 2,
      }),
    );
    chart.append(
      shape(
        "text",
        {
          x: 97 + i * 280,
          y: 24,
          fill: colors[mode],
          "font-family": "Arial",
          "font-size": 11,
        },
        name,
      ),
    );
  });
  for (const panel of panels) {
    const values = result.runs.flatMap((r) =>
      r.rows.map((row) => row[panel.key]),
    );
    if (panel.key === "y")
      values.push(
        ...result.runs[0].rows.map((r) => r.target),
        ...result.runs.map((r) => r.metrics.finalOutput),
      );
    if (panel.key === "command")
      values.push(result.input.minimum, result.input.maximum);
    values.push(0);
    let low = Math.min(...values),
      high = Math.max(...values);
    const pad = Math.max((high - low) * 0.1, 0.1);
    low -= pad;
    high += pad;
    const Y = (v) =>
      panel.top + panel.height - ((v - low) / (high - low)) * panel.height;
    chart.append(
      shape(
        "text",
        {
          x: 70,
          y: panel.top - 17,
          fill: "#7b8983",
          "font-family": "Arial",
          "font-size": 10,
          "letter-spacing": 1,
        },
        panel.label,
      ),
    );
    for (let j = 0; j <= 4; j++) {
      const value = low + ((high - low) * j) / 4,
        y = Y(value);
      chart.append(
        shape("line", {
          x1: 70,
          y1: y,
          x2: 960,
          y2: y,
          stroke: "#e2e7e0",
          "stroke-width": 1,
        }),
      );
      chart.append(
        shape(
          "text",
          {
            x: 60,
            y: y + 4,
            "text-anchor": "end",
            fill: "#7b8983",
            "font-family": "Arial",
            "font-size": 10,
          },
          Math.abs(value) > 1000 ? value.toExponential(1) : value.toFixed(2),
        ),
      );
    }
    for (const at of [
      result.input.stepAt,
      result.input.returnAt,
      ...(result.input.disturbance !== 0 ? [result.input.disturbanceAt] : []),
    ])
      chart.append(
        shape("line", {
          x1: X(at * result.input.dt),
          y1: panel.top,
          x2: X(at * result.input.dt),
          y2: panel.top + panel.height,
          stroke: "#c5cec6",
          "stroke-dasharray": "3 5",
        }),
      );
    if (panel.key === "y") {
      const rows = result.runs[0].rows;
      let d = "";
      for (const row of rows) {
        d += `${d ? " L" : "M"}${X(row.time)} ${Y(row.target)} L${X(row.time + result.input.dt)} ${Y(row.target)}`;
      }
      chart.append(
        shape("path", {
          d,
          fill: "none",
          stroke: "#a7a199",
          "stroke-width": 1.5,
          "stroke-dasharray": "5 4",
        }),
      );
    }
    for (const r of result.runs) {
      const points = r.rows.map((row) => [row.time, row[panel.key]]);
      if (panel.key === "y" && r.rows.length)
        points.push([r.metrics.duration, r.metrics.finalOutput]);
      const d = points
        .map(
          ([t, v], i) =>
            `${i ? "L" : "M"}${X(t).toFixed(3)} ${Y(v).toFixed(3)}`,
        )
        .join(" ");
      chart.append(
        shape("path", {
          d,
          fill: "none",
          stroke: colors[r.mode],
          "stroke-width": 1.8,
          "stroke-linejoin": "round",
        }),
      );
    }
    chart.append(
      shape("line", {
        "data-cursor": "true",
        x1: 70,
        y1: panel.top,
        x2: 70,
        y2: panel.top + panel.height,
        stroke: "#344945",
        "stroke-width": 1,
        "stroke-dasharray": "2 3",
      }),
    );
  }
  for (let j = 0; j <= 5; j++)
    chart.append(
      shape(
        "text",
        {
          x: X((duration * j) / 5),
          y: 650,
          "text-anchor": "middle",
          fill: "#7b8983",
          "font-family": "Arial",
          "font-size": 11,
        },
        `${((duration * j) / 5).toFixed(1)}s`,
      ),
    );
  chart.append(
    shape(
      "text",
      {
        x: 960,
        y: 673,
        "text-anchor": "end",
        fill: "#7b8983",
        "font-family": "Arial",
        "font-size": 9,
      },
      "Dashed setpoint · vertical event markers · sampled numerical experiment",
    ),
  );
}
function inspect() {
  if (!result) return;
  const k = Number($("step").value),
    run = result.runs.find((r) => r.mode === $("mode").value),
    row = run.rows[k];
  $("time").textContent = `${(k * result.input.dt).toFixed(3)} s`;
  const x = 70 + (k / result.input.steps) * 890;
  $("chart")
    .querySelectorAll("[data-cursor]")
    .forEach((line) => {
      line.setAttribute("x1", x);
      line.setAttribute("x2", x);
    });
  $("sample").replaceChildren();
  if (!row) {
    $("sample-note").textContent =
      "This controller stopped before the selected sample. Select an earlier time to inspect its valid prefix.";
    return;
  }
  for (const [label, value] of [
    ["Setpoint", row.target],
    ["True output", row.y],
    ["Measured error", row.error],
    ["P contribution", row.P],
    ["D contribution", row.D],
    ["Integral used", row.integralUsed],
    ["Integral after", row.integralAfter],
    ["Raw command", row.rawCommand],
    ["Clipped command", row.command],
    ["Delayed input", row.applied],
  ]) {
    const item = el("div");
    item.append(el("strong", num(value)), el("span", label));
    $("sample").append(item);
  }
  $("sample-note").textContent =
    `Sample ${k} · ${row.saturated ? "Actuator saturated" : "Command within limits"}${row.integrationHeld ? " · outward integration held" : ""}. Input disturbance ${num(row.disturbance)}; measurement noise ${num(row.noise)}. Output at the end of this interval: ${num(row.nextY)}.`;
}
$("step").oninput = () => {
  stop();
  inspect();
};
$("mode").onchange = inspect;
$("chart").onclick = (e) => {
  if (!result) return;
  stop();
  const box = $("chart").getBoundingClientRect(),
    x = ((e.clientX - box.left) / box.width) * 1000;
  $("step").value = Math.max(
    0,
    Math.min(
      result.input.steps - 1,
      Math.round(((x - 70) / 890) * result.input.steps),
    ),
  );
  inspect();
};
$("play").onclick = () => {
  if (timer) {
    stop();
    return;
  }
  if (Number($("step").value) >= result.input.steps - 1) $("step").value = 0;
  $("play").textContent = "Pause cursor";
  timer = setInterval(() => {
    const k = Math.min(
      result.input.steps - 1,
      Number($("step").value) +
        Math.max(1, Math.floor(result.input.steps / 200)),
    );
    $("step").value = k;
    inspect();
    if (k === result.input.steps - 1) stop();
  }, 60);
};
$("tuning").onsubmit = (e) => {
  e.preventDefault();
  const input = { ...result.input };
  for (const key of ["kp", "ki", "kd", "kb"]) input[key] = Number($(key).value);
  run(input);
};
$("example").onclick = () => run(example);
$("apply").onclick = () => {
  try {
    run(JSON.parse($("json").value));
  } catch (e) {
    $("status").textContent = e.message;
  }
};
function download(name, content, type) {
  const url = URL.createObjectURL(new Blob([content], { type })),
    a = el("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
$("save").onclick = () => {
  if (result)
    download(
      "servo-notes-experiment.json",
      JSON.stringify(result.input, null, 2),
      "application/json",
    );
};
$("export").onclick = () => {
  if (result)
    download(
      "servo-notes-evidence.json",
      JSON.stringify(
        {
          ...result,
          inspectedStep: Number($("step").value),
          inspectedMode: $("mode").value,
        },
        null,
        2,
      ),
      "application/json",
    );
};
$("csv").onclick = () => {
  const columns = [
    "step",
    "time",
    "target",
    "y",
    "measurement",
    "error",
    "noise",
    "P",
    "integralBefore",
    "integralUsed",
    "integralAfter",
    "D",
    "rawCommand",
    "command",
    "applied",
    "disturbance",
    "nextY",
    "saturated",
    "integrationHeld",
  ];
  download(
    "servo-notes-traces.csv",
    [
      "mode," + columns.join(","),
      ...result.runs.flatMap((r) =>
        r.rows.map((row) => [r.mode, ...columns.map((c) => row[c])].join(",")),
      ),
    ].join("\r\n"),
    "text/csv",
  );
};
$("svg").onclick = () =>
  download(
    "servo-notes-chart.svg",
    new XMLSerializer().serializeToString($("chart")),
    "image/svg+xml",
  );
$("open").onclick = () => $("file").click();
$("file").onchange = async () => {
  const file = $("file").files[0];
  if (!file) return;
  try {
    if (file.size > 100000)
      throw Error("Experiment files must be at most 100 KB.");
    const raw = JSON.parse(await file.text());
    run(raw.input ?? raw);
  } catch (e) {
    $("status").textContent = e.message;
  } finally {
    $("file").value = "";
  }
};
run(example);
