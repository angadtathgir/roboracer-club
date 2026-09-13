import type { AlgorithmSpec } from "@/lib/algorithms";

const STORAGE_KEY = "roboracer.playground.algoParams";

function readStore(): Record<string, Record<string, number>> {
  if (typeof window === "undefined") return {};
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return {};
    const parsed = JSON.parse(raw) as unknown;
    if (!parsed || typeof parsed !== "object") return {};
    return parsed as Record<string, Record<string, number>>;
  } catch {
    return {};
  }
}

function writeStore(all: Record<string, Record<string, number>>) {
  if (typeof window === "undefined") return;
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(all));
  } catch {
    /* quota / private mode — keep going with in-memory values */
  }
}

export function defaultAlgoParams(spec: AlgorithmSpec): Record<string, number> {
  return Object.fromEntries(spec.params.map((s) => [s.name, s.default]));
}

/** Defaults overlaid with the last values the user set for this algorithm. */
export function loadAlgoParams(spec: AlgorithmSpec): Record<string, number> {
  const values = defaultAlgoParams(spec);
  const saved = readStore()[spec.id];
  if (!saved) return values;
  for (const p of spec.params) {
    const v = saved[p.name];
    if (typeof v === "number" && Number.isFinite(v)) values[p.name] = v;
  }
  return values;
}

export function patchAlgoParams(
  algoId: string,
  patch: Record<string, number>
) {
  const all = readStore();
  const next = { ...(all[algoId] ?? {}) };
  for (const [k, v] of Object.entries(patch)) {
    if (typeof v === "number" && Number.isFinite(v)) next[k] = v;
  }
  all[algoId] = next;
  writeStore(all);
}
