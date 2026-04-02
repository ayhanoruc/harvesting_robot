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

const PredictionImage: React.FC<{ image?: string }> = ({ image }) => (
  <div
    style={{
      flex: "0 0 44%",
      background: C.panel,
      borderRadius: 10,
      border: `2px solid ${C.border}`,
      overflow: "hidden",
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      position: "relative",
    }}
  >
    <div
      style={{
        position: "absolute",
        top: 12,
        left: 16,
        fontSize: 18,
        color: C.white,
        fontWeight: 700,
        letterSpacing: 1.5,
        textTransform: "uppercase",
        zIndex: 1,
        textShadow: "0 1px 6px rgba(0,0,0,0.9)",
        background: "rgba(0,0,0,0.5)",
        padding: "4px 10px",
        borderRadius: 4,
      }}
    >
      YOLO Detection
    </div>
    {image ? (
      <Img
        src={staticFile(image)}
        style={{ width: "100%", height: "100%", objectFit: "cover" }}
      />
    ) : (
      <span style={{ color: C.dim, fontSize: 24 }}>
        Awaiting detection...
      </span>
    )}
  </div>
);

// ─── STEP INFO ─────────────────────────────────────────────

const StepInfo: React.FC<{ section: any; stepHighlight?: string }> = ({
  section,
  stepHighlight,
}) => {
  const steps = (section?.events || []).filter((e: any) => e.type === "step");
  const hasSteps = steps.length > 0;

  return (
    <div
      style={{
        flex: "0 0 24%",
        background: C.panel,
        borderRadius: 10,
        border: `2px solid ${C.border}`,
        padding: "14px 20px",
        display: "flex",
        flexDirection: "column",
        gap: 5,
        overflow: "hidden",
      }}
    >
      <div
        style={{
          fontSize: 18,
          color: C.white,
          fontWeight: 700,
          letterSpacing: 1.5,
          textTransform: "uppercase",
          marginBottom: 2,
        }}
      >
        Pick Steps
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
                gap: 10,
                padding: "4px 10px",
                borderRadius: 5,
                background: isActive ? "rgba(88,166,255,0.12)" : "transparent",
                borderLeft: isActive
                  ? `3px solid ${C.accent}`
                  : "3px solid transparent",
              }}
            >
              <span
                style={{
                  fontSize: 18,
                  fontWeight: 700,
                  color: isActive ? C.accent : isPast ? C.green : C.dim,
                  fontFamily: "monospace",
                  minWidth: 40,
                }}
              >
                {step.step}
              </span>
              <span
                style={{
                  fontSize: 18,
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
        <div style={{ color: C.text, fontSize: 22, fontWeight: 500 }}>
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
        flex: 1,
        background: C.panel,
        borderRadius: 10,
        border: `2px solid ${C.border}`,
        padding: "14px 20px",
        display: "flex",
        flexDirection: "column",
        gap: 8,
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
            padding: "6px 16px",
            borderRadius: 5,
            background: STATE_COLOR[state] || C.dim,
            color: C.bg,
            fontSize: 22,
            fontWeight: 800,
            letterSpacing: 1,
          }}
        >
          {state}
        </div>
        <span style={{ color: C.white, fontSize: 22, fontWeight: 600 }}>
          {label}
        </span>
      </div>

      {/* Cluster target */}
      {cluster && (
        <div style={{ fontSize: 20, color: C.text, lineHeight: 1.4 }}>
          Target:{" "}
          <span style={{ color: C.accent, fontWeight: 700 }}>{cluster}</span>
          {section?.cluster_pos && (
            <span
              style={{
                marginLeft: 8,
                fontFamily: "monospace",
                fontSize: 17,
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
        <div style={{ fontSize: 17, color: C.dim, fontFamily: "monospace" }}>
          {transition}
        </div>
      )}

      {/* Events — wrap enabled, max 3 */}
      <div
        style={{
          flex: 1,
          overflow: "hidden",
          display: "flex",
          flexDirection: "column",
          gap: 5,
          marginTop: 2,
        }}
      >
        {infoEvents.slice(-3).map((ev: any, i: number) => (
          <div
            key={i}
            style={{
              fontSize: 16,
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
        <StepInfo section={section} stepHighlight={kf.stepHighlight} />
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
