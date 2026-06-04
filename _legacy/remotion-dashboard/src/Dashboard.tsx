import React from "react";
import {
  AbsoluteFill,
  OffthreadVideo,
  staticFile,
  useCurrentFrame,
  useVideoConfig,
  Img,
} from "remotion";
import keyframes, { Keyframe } from "./timeline";
import timelineData from "../public/timeline.json";

const C = {
  bg: "#0d1117",
  panel: "#161b22",
  border: "#30363d",
  accent: "#58a6ff",
  green: "#3fb950",
  orange: "#d29922",
  red: "#f85149",
  text: "#c9d1d9",
  dim: "#8b949e",
  white: "#f0f6fc",
};

const STATE_COLOR: Record<string, string> = {
  IDLE: C.dim,
  SCANNING: C.accent,
  APPROACHING: C.orange,
  HARVESTING: C.green,
};

function getActiveKeyframe(frame: number, fps: number): Keyframe {
  const sec = frame / fps;
  let active = keyframes[0];
  for (const kf of keyframes) {
    if (sec >= kf.sec) active = kf;
    else break;
  }
  return active;
}

function getSection(id: string) {
  return timelineData.sections.find((s: any) => s.id === id);
}

// ─── PREDICTION IMAGE ──────────────────────────────────────

// Detection stats per image (from YOLO logs)
const IMAGE_STATS: Record<string, { bolls: number; conf: string }> = {
  "cluster1.png":    { bolls: 12, conf: "0.73–0.92" },
  "cluster2.png":    { bolls: 12, conf: "0.73–0.93" },
  "cluster3.png":    { bolls: 12, conf: "0.73–0.92" },
};

const PredictionImage: React.FC<{ image?: string }> = ({ image }) => {
  const stats = image ? IMAGE_STATS[image] : null;
  return (
    <div
      style={{
        flex: 1,
        background: C.panel,
        borderRadius: 10,
        border: `2px solid ${C.border}`,
        overflow: "hidden",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        position: "relative",
        minHeight: 0,
      }}
    >
      {/* Top-left: title */}
      <div
        style={{
          position: "absolute",
          top: 10,
          left: 12,
          fontSize: 16,
          color: C.white,
          fontWeight: 700,
          letterSpacing: 1.5,
          textTransform: "uppercase",
          zIndex: 1,
          textShadow: "0 1px 6px rgba(0,0,0,0.9)",
          background: "rgba(0,0,0,0.6)",
          padding: "3px 10px",
          borderRadius: 4,
        }}
      >
        YOLO Detection
      </div>
      {/* Bottom-left: stats overlay */}
      {stats && (
        <div
          style={{
            position: "absolute",
            bottom: 10,
            left: 12,
            zIndex: 1,
            display: "flex",
            gap: 8,
          }}
        >
          <div
            style={{
              background: "rgba(0,0,0,0.7)",
              padding: "4px 10px",
              borderRadius: 4,
              fontSize: 15,
              fontWeight: 700,
              color: C.green,
            }}
          >
            {stats.bolls} bolls
          </div>
          <div
            style={{
              background: "rgba(0,0,0,0.7)",
              padding: "4px 10px",
              borderRadius: 4,
              fontSize: 15,
              fontWeight: 600,
              color: C.accent,
            }}
          >
            conf {stats.conf}
          </div>
        </div>
      )}
      {image ? (
        <Img
          src={staticFile(image)}
          style={{ width: "100%", height: "100%", objectFit: "contain" }}
        />
      ) : (
        <span style={{ color: C.dim, fontSize: 24 }}>
          Awaiting detection...
        </span>
      )}
    </div>
  );
};

// ─── STEP INFO ─────────────────────────────────────────────

const StepInfo: React.FC<{
  section: any;
  stepHighlight?: string;
  pickNumber?: number;
  pickTotal?: number;
}> = ({ section, stepHighlight, pickNumber, pickTotal }) => {
  const steps = (section?.events || []).filter((e: any) => e.type === "step");
  const hasSteps = steps.length > 0;

  return (
    <div
      style={{
        flex: "0 0 auto",
        background: C.panel,
        borderRadius: 10,
        border: `2px solid ${C.border}`,
        padding: "8px 16px",
        display: "flex",
        flexDirection: "column",
        gap: 0,
      }}
    >
      <div
        style={{
          display: "flex",
          alignItems: "center",
          justifyContent: "space-between",
          marginBottom: 2,
        }}
      >
        <span
          style={{
            fontSize: 16,
            color: C.white,
            fontWeight: 700,
            letterSpacing: 1.5,
            textTransform: "uppercase",
          }}
        >
          Pick Steps
        </span>
        {pickNumber && pickTotal && (
          <span
            style={{
              fontSize: 15,
              fontWeight: 700,
              color: C.green,
              fontFamily: "monospace",
            }}
          >
            Boll {pickNumber}/{pickTotal}
          </span>
        )}
      </div>
      {hasSteps ? (
        steps.map((step: any, i: number) => {
          const isActive = stepHighlight === step.step;
          const isPast =
            stepHighlight && parseInt(step.step) < parseInt(stepHighlight);
          return (
            <div
              key={i}
              style={{
                display: "flex",
                alignItems: "center",
                gap: 8,
                padding: "2px 8px",
                borderRadius: 4,
                background: isActive ? "rgba(88,166,255,0.12)" : "transparent",
                borderLeft: isActive
                  ? `3px solid ${C.accent}`
                  : "3px solid transparent",
              }}
            >
              <span
                style={{
                  fontSize: 15,
                  fontWeight: 700,
                  color: isActive ? C.accent : isPast ? C.green : C.dim,
                  fontFamily: "monospace",
                  minWidth: 32,
                }}
              >
                {step.step}
              </span>
              <span
                style={{
                  fontSize: 15,
                  color: isActive ? C.white : C.dim,
                  fontWeight: isActive ? 600 : 400,
                }}
              >
                {step.label}
              </span>
            </div>
          );
        })
      ) : (
        <div style={{ color: C.text, fontSize: 18, fontWeight: 500 }}>
          {section?.state === "SCANNING"
            ? "Scan in progress..."
            : section?.state === "APPROACHING"
            ? "Moving to cluster..."
            : "Waiting..."}
        </div>
      )}
    </div>
  );
};

// ─── PIPELINE INFO ─────────────────────────────────────────

const PipelineInfo: React.FC<{ section: any }> = ({ section }) => {
  const state = section?.state || "IDLE";
  const label = section?.label || "System Ready";
  const cluster = section?.cluster;
  const transition = section?.transition;

  const infoEvents = (section?.events || []).filter(
    (e: any) => e.type !== "step"
  );

  return (
    <div
      style={{
        flex: "0 0 auto",
        background: C.panel,
        borderRadius: 10,
        border: `2px solid ${C.border}`,
        padding: "10px 16px",
        display: "flex",
        flexDirection: "column",
        gap: 6,
        overflow: "hidden",
      }}
    >
      {/* Header */}
      <div
        style={{
          fontSize: 18,
          color: C.white,
          fontWeight: 700,
          letterSpacing: 1.5,
          textTransform: "uppercase",
        }}
      >
        Pipeline Status
      </div>

      {/* State badge */}
      <div style={{ display: "flex", alignItems: "center", gap: 14 }}>
        <div
          style={{
            padding: "4px 14px",
            borderRadius: 5,
            background: STATE_COLOR[state] || C.dim,
            color: C.bg,
            fontSize: 18,
            fontWeight: 800,
            letterSpacing: 1,
          }}
        >
          {state}
        </div>
        <span style={{ color: C.white, fontSize: 18, fontWeight: 600 }}>
          {label}
        </span>
      </div>

      {/* Cluster target */}
      {cluster && (
        <div style={{ fontSize: 16, color: C.text, lineHeight: 1.3 }}>
          Target:{" "}
          <span style={{ color: C.accent, fontWeight: 700 }}>{cluster}</span>
          {section?.cluster_pos && (
            <span
              style={{
                marginLeft: 8,
                fontFamily: "monospace",
                fontSize: 14,
                color: C.dim,
              }}
            >
              ({section.cluster_pos.map((v: number) => v.toFixed(2)).join(", ")})
            </span>
          )}
        </div>
      )}

      {/* Transition arrow */}
      {transition && (
        <div style={{ fontSize: 14, color: C.dim, fontFamily: "monospace" }}>
          {transition}
        </div>
      )}

      {/* Events — max 3 */}
      <div
        style={{
          overflow: "hidden",
          display: "flex",
          flexDirection: "column",
          gap: 3,
        }}
      >
        {infoEvents.slice(-3).map((ev: any, i: number) => (
          <div
            key={i}
            style={{
              fontSize: 14,
              fontWeight: 500,
              lineHeight: 1.3,
              color:
                ev.type === "error"
                  ? C.red
                  : ev.type === "warn"
                  ? C.orange
                  : ev.type === "result"
                  ? C.green
                  : C.text,
              overflow: "hidden",
              textOverflow: "ellipsis",
              display: "-webkit-box",
              WebkitLineClamp: 2,
              WebkitBoxOrient: "vertical",
            }}
          >
            {ev.msg}
          </div>
        ))}
      </div>
    </div>
  );
};

// ─── MAIN DASHBOARD ────────────────────────────────────────

export const Dashboard: React.FC<{
  timeline: any;
  videoSrc: string;
}> = ({ videoSrc }) => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();

  const kf = getActiveKeyframe(frame, fps);
  const section = getSection(kf.sectionId);

  return (
    <AbsoluteFill
      style={{
        background: C.bg,
        fontFamily:
          '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
        padding: 10,
        display: "flex",
        flexDirection: "row",
        gap: 10,
      }}
    >
      {/* LEFT — 33% */}
      <div
        style={{
          flex: "0 0 33%",
          display: "flex",
          flexDirection: "column",
          gap: 8,
        }}
      >
        <PredictionImage image={kf.image} />
        <StepInfo
          section={section}
          stepHighlight={kf.stepHighlight}
          pickNumber={kf.pickNumber}
          pickTotal={kf.pickTotal}
        />
        <PipelineInfo section={section} />
      </div>

      {/* RIGHT — VIDEO full cover */}
      <div
        style={{
          flex: 1,
          borderRadius: 10,
          overflow: "hidden",
          border: `2px solid ${C.border}`,
          background: "#000",
        }}
      >
        <OffthreadVideo
          src={videoSrc}
          style={{ width: "100%", height: "100%", objectFit: "cover" }}
        />
      </div>
    </AbsoluteFill>
  );
};
