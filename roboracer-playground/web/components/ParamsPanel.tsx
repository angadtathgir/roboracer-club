"use client";

import { useEffect, useRef, useState } from "react";
import type { ParamsAck } from "@/lib/useGateway";
import { algorithmSpec, type AlgorithmSpec } from "@/lib/algorithms";
import {
  defaultAlgoParams,
  loadAlgoParams,
  patchAlgoParams,
} from "@/lib/algoParams";

interface Props {
  algorithm: AlgorithmSpec;
  onSetParams: (params: Record<string, number>) => void;
  lastAck: ParamsAck | null;
  /** Grey out sliders (e.g. no websocket). Draft values still persist. */
  disabled: boolean;
  /** When true, slider moves are sent to the running ROS node. */
  live: boolean;
}

export default function ParamsPanel({
  algorithm,
  onSetParams,
  lastAck,
  disabled,
  live,
}: Props) {
  const algoId = algorithm.id;
  // Defaults only on first paint so SSR and hydration match. localStorage
  // is applied in the effect below (after mount, and when the algo changes).
  const [values, setValues] = useState<Record<string, number>>(() =>
    defaultAlgoParams(algorithm)
  );
  const pending = useRef<Record<string, number>>({});
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    setValues(loadAlgoParams(algorithmSpec(algoId)));
    pending.current = {};
  }, [algoId]);

  // Don't deliver a delayed set_params after Stop.
  useEffect(() => {
    if (live) return;
    if (timer.current) {
      clearTimeout(timer.current);
      timer.current = null;
    }
    pending.current = {};
  }, [live]);

  // Debounced live send (~80ms) so sliders feel real-time without spamming.
  const queueSend = (name: string, value: number) => {
    pending.current[name] = value;
    if (!live) return;
    if (timer.current) clearTimeout(timer.current);
    timer.current = setTimeout(() => {
      onSetParams({ ...pending.current });
      pending.current = {};
    }, 80);
  };

  useEffect(
    () => () => {
      if (timer.current) clearTimeout(timer.current);
    },
    []
  );

  return (
    <div>
      <div className="field-label">Parameters</div>
      {algorithm.params.map((s) => (
        <div className="slider-group" key={s.name}>
          <div className="slider-label">
            <span className="slider-name">
              {s.label}
              <span className="info-tip" tabIndex={0} aria-label={s.info}>
                <span className="info-icon" aria-hidden>
                  i
                </span>
                <span className="info-bubble" role="tooltip">
                  {s.info}
                </span>
              </span>
            </span>
            <span className="val">
              {(values[s.name] ?? s.default).toFixed(s.step < 0.01 ? 3 : 2)}
            </span>
          </div>
          <input
            type="range"
            min={s.min}
            max={s.max}
            step={s.step}
            value={values[s.name] ?? s.default}
            disabled={disabled}
            onChange={(e) => {
              const v = parseFloat(e.target.value);
              setValues((prev) => ({ ...prev, [s.name]: v }));
              patchAlgoParams(algoId, { [s.name]: v });
              queueSend(s.name, v);
            }}
          />
        </div>
      ))}
      {lastAck && !lastAck.ok && (
        <div className="ack fail">Rejected: {lastAck.reason}</div>
      )}
      {!live && (
        <div className="note-inline">
          Values are kept when you stop. Start to apply them on the car.
        </div>
      )}
    </div>
  );
}
