"use client";

import { useState } from "react";
import type { GatewayStatus } from "@/lib/useGateway";
import { algorithmSpec } from "@/lib/algorithms";

interface Props {
  status: GatewayStatus;
  gatewayUrl: string;
  simAddress: string;
  uiRttMs: number | null;
  onGatewayUrlChange: (url: string) => void;
  onShowSetup: () => void;
}

function formatMs(ms: number | null): string {
  if (ms == null || !Number.isFinite(ms)) return "—";
  return ms < 10 ? `${ms.toFixed(1)} ms` : `${Math.round(ms)} ms`;
}

function lagTone(ms: number | null): "" | "good" | "warn" | "bad" {
  if (ms == null || !Number.isFinite(ms)) return "";
  if (ms < 40) return "good";
  if (ms < 120) return "warn";
  return "bad";
}

export default function StatusTab({
  status,
  gatewayUrl,
  simAddress,
  uiRttMs,
  onGatewayUrlChange,
  onShowSetup,
}: Props) {
  const [editUrl, setEditUrl] = useState(gatewayUrl);
  const [copied, setCopied] = useState(false);

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(simAddress);
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    } catch {
      /* clipboard unavailable */
    }
  };

  const algorithmLabel = status.algorithm
    ? algorithmSpec(status.algorithm).label
    : "Stopped";

  const simRtt = status.lag.sim_rtt_ms;
  const simHz = status.lag.sim_hz;
  let simHint = "Not connected";
  if (status.simConnected) {
    const parts: string[] = [];
    parts.push(simRtt != null ? formatMs(simRtt) : "Streaming");
    if (simHz != null) parts.push(`${simHz.toFixed(0)} Hz`);
    simHint = parts.join(" · ");
  }

  return (
    <div>
      <div className="status-rows">
        <div className="status-row">
          <span className={`dot ${status.wsConnected ? "on" : ""}`} />
          Gateway
          <span className={`hint ${lagTone(uiRttMs)}`}>
            {status.wsConnected
              ? uiRttMs != null
                ? formatMs(uiRttMs)
                : "Connected"
              : "Disconnected"}
          </span>
        </div>
        <div className="status-row">
          <span className={`dot ${status.simConnected ? "on" : ""}`} />
          Simulator
          <span
            className={`hint ${status.simConnected ? lagTone(simRtt) : ""}`}
            title={
              simRtt == null && status.simConnected
                ? "Control RTT appears after the car starts steering"
                : "Command echo through Unity (control-loop RTT)"
            }
          >
            {simHint}
          </span>
        </div>
        <div className="status-row">
          <span className={`dot ${status.algorithm ? "on" : ""}`} />
          Algorithm
          <span className="hint">{algorithmLabel}</span>
        </div>
      </div>
      <div className="lag-caption">
        Gateway is browser RTT. Sim is command echo — start driving to sample.
      </div>

      <div className="field-label">Gateway URL</div>
      <input
        className="text"
        value={editUrl}
        onChange={(e) => setEditUrl(e.target.value)}
        onBlur={() => onGatewayUrlChange(editUrl)}
        onKeyDown={(e) => {
          if (e.key === "Enter") onGatewayUrlChange(editUrl);
        }}
        placeholder="ws://localhost:8000/ws"
        spellCheck={false}
      />

      <div className="field-label">Simulator address</div>
      <div className="copy-row">
        <span className="mono">{simAddress}</span>
        <button className="btn small" onClick={copy}>
          {copied ? "Copied" : "Copy"}
        </button>
      </div>

      <div style={{ marginTop: 16 }}>
        <button className="btn" style={{ width: "100%" }} onClick={onShowSetup}>
          Setup instructions
        </button>
      </div>
    </div>
  );
}
