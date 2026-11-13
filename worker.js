import { analyze } from "./core.js";
self.onmessage = ({ data }) => {
  try {
    self.postMessage({ result: analyze(data) });
  } catch (e) {
    self.postMessage({ error: e.message });
  }
};
