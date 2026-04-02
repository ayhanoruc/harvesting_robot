import React, { useRef, useState } from 'react';

const ArchLeft = () => {
  const diagramRef = useRef(null);
  const [status, setStatus] = useState('');

  const handleDownload = async () => {
    setStatus('Generating...');
    try {
      if (!window.html2canvas) {
        await new Promise((resolve, reject) => {
          const s = document.createElement('script');
          s.src = 'https://cdn.jsdelivr.net/npm/html2canvas@1.4.1/dist/html2canvas.min.js';
          s.onload = resolve; s.onerror = reject;
          document.head.appendChild(s);
        });
      }
      const canvas = await window.html2canvas(diagramRef.current, { scale: 3, backgroundColor: '#ffffff', useCORS: true });
      const link = document.createElement('a');
      link.download = 'arch-left-software.png';
      link.href = canvas.toDataURL('image/png');
      link.click();
      setStatus('Done!');
    } catch (err) { setStatus('Error: ' + err.message); }
  };

  const c = {
    green: '#22c55e', greenDark: '#16a34a', greenBorder: '#15803d', greenLight: '#dcfce7',
    orange: '#f97316', orangeBorder: '#ea580c',
    blue: '#2563eb', blueDark: '#1d4ed8', blueBorder: '#1e40af', blueLight: '#dbeafe',
    purple: '#8b5cf6', purpleBorder: '#7c3aed', purpleLight: '#ede9fe',
    gray100: '#f3f4f6', gray300: '#d1d5db', gray400: '#9ca3af', gray500: '#6b7280',
    gray600: '#4b5563', gray700: '#374151', gray800: '#1f2937', white: '#ffffff',
  };

  const SectionTitle = ({ children }) => (
    <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '8px', fontSize: '13px' }}>{children}</div>
  );

  return (
    <div style={{ backgroundColor: c.white, padding: '4px', fontFamily: 'system-ui, -apple-system, sans-serif' }}>
      <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', marginBottom: '4px' }}>
        <button onClick={handleDownload} style={{ backgroundColor: c.blue, color: c.white, fontWeight: '600', padding: '8px 20px', borderRadius: '6px', border: 'none', cursor: 'pointer', fontSize: '13px' }}>
          Download Left PNG
        </button>
        {status && <span style={{ color: c.gray600, fontSize: '13px' }}>{status}</span>}
      </div>

      <div ref={diagramRef} style={{ backgroundColor: c.gray100, padding: '0px', display: 'flex', flexDirection: 'row', alignItems: 'stretch' }}>

        {/* Main content column */}
        <div style={{ flex: 1, minWidth: 0 }}>
          <div style={{ backgroundColor: c.white, padding: '6px 8px', marginBottom: '2px' }}>
            <h2 style={{ fontSize: '16px', fontWeight: 'bold', textAlign: 'center', color: c.gray800, margin: '0 0 1px 0' }}>
              RoboCot: System Architecture
            </h2>
            <div style={{ textAlign: 'center', color: c.gray500, fontSize: '10px' }}>
              ME492 — Software & Compute Stack
            </div>
          </div>

          {/* REMOTE LAYER */}
          <div style={{ backgroundColor: c.gray100, border: `1px solid ${c.gray300}`, padding: '8px', marginBottom: '2px' }}>
            <SectionTitle>REMOTE / OPERATOR LAYER</SectionTitle>
            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', flexWrap: 'wrap' }}>
              {[
                { t: 'Monitoring &\nControl App', s: '' },
                { t: 'Remote Access', s: 'SSH / VNC' },
                { t: 'System Updates', s: 'Git → Docker rebuild' }
              ].map((item, i) => (
                <div key={i} style={{ backgroundColor: c.green, border: `1px solid ${c.greenBorder}`, borderRadius: '4px', padding: '6px', width: '145px', textAlign: 'center' }}>
                  <div style={{ fontWeight: '600', color: c.white, fontSize: '11px', whiteSpace: 'pre-line' }}>{item.t}</div>
                  {item.s && <div style={{ color: 'rgba(255,255,255,0.7)', fontSize: '9px' }}>{item.s}</div>}
                </div>
              ))}
            </div>
          </div>

          {/* Connector */}
          <div style={{ textAlign: 'center', padding: '1px 0' }}>
            <div style={{ fontSize: '14px', color: c.gray500, lineHeight: 1 }}>↕</div>
            <div style={{ display: 'inline-block', backgroundColor: c.gray500, color: c.white, fontSize: '9px', padding: '1px 6px', borderRadius: '6px' }}>WiFi / Ethernet</div>
          </div>

          {/* COMPUTE LAYER */}
          <div style={{ backgroundColor: c.gray100, border: `1px solid ${c.gray300}`, padding: '8px' }}>
            <SectionTitle>COMPUTE LAYER — ROS2 Humble</SectionTitle>

            {/* Sim/Real badges */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '6px', marginBottom: '6px' }}>
              <div style={{ backgroundColor: c.blueLight, border: `1px solid ${c.blueBorder}`, borderRadius: '4px', padding: '3px 8px', fontSize: '9px', fontWeight: '600', color: c.blueDark }}>
                SIM: WSL2 / Ubuntu
              </div>
              <div style={{ backgroundColor: c.purpleLight, border: `1px solid ${c.purpleBorder}`, borderRadius: '4px', padding: '3px 8px', fontSize: '9px', fontWeight: '600', color: c.purple }}>
                REAL: Docker on Xavier
              </div>
            </div>

            {/* Orchestrator */}
            <div style={{ backgroundColor: c.greenDark, border: `2px solid ${c.greenBorder}`, borderRadius: '5px', padding: '6px', maxWidth: '440px', margin: '0 auto 4px auto' }}>
              <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.white, fontSize: '13px' }}>ORCHESTRATOR NODE (FSM)</div>
              <div style={{ textAlign: 'center', color: c.greenLight, fontSize: '9px', marginTop: '1px' }}>
                IDLE → SCANNING → APPROACHING → HARVESTING → RETURNING
              </div>
            </div>

            {/* Arrows */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '24px', marginBottom: '4px' }}>
              <span style={{ color: c.gray600, fontWeight: '600', fontSize: '11px' }}>↑ events</span>
              <span style={{ color: c.gray600, fontWeight: '600', fontSize: '11px' }}>commands ↓</span>
            </div>

            {/* Perception + Planning */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '6px', marginBottom: '6px', flexWrap: 'wrap' }}>
              <div style={{ backgroundColor: c.white, border: `1px solid ${c.gray300}`, borderRadius: '5px', padding: '6px', width: '215px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '4px', fontSize: '11px' }}>PERCEPTION</div>
                {[
                  { t: 'YOLO11 Detector', s: 'cotton_boll + cluster' },
                  { t: '3D Positioning', s: 'Depth + TF → World' },
                  { t: 'World-Space Clustering', s: 'Complete-linkage' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.green, border: `1px solid ${c.greenBorder}`, borderRadius: '3px', padding: '4px', textAlign: 'center', marginBottom: '2px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '11px' }}>{item.t}</div>
                    <div style={{ color: c.greenLight, fontSize: '9px' }}>{item.s}</div>
                  </div>
                ))}
              </div>

              <div style={{ backgroundColor: c.white, border: `1px solid ${c.gray300}`, borderRadius: '5px', padding: '6px', width: '215px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '4px', fontSize: '11px' }}>PLANNING & CONTROL</div>
                {[
                  { t: 'IK Multi-Seed + Joint Goal', s: 'arm_commander' },
                  { t: 'MoveIt2 / OMPL', s: 'path planning fallback' },
                  { t: 'Gripper Controller', s: 'open/close + stall detect' },
                  { t: 'Harvest Executor', s: '8-step pick-and-place' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.green, border: `1px solid ${c.greenBorder}`, borderRadius: '3px', padding: '4px', textAlign: 'center', marginBottom: '2px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '11px' }}>{item.t}</div>
                    <div style={{ color: c.greenLight, fontSize: '9px' }}>{item.s}</div>
                  </div>
                ))}
              </div>
            </div>

            {/* Safety + Logging */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '6px', flexWrap: 'wrap' }}>
              <div style={{ backgroundColor: c.white, border: `1px solid ${c.gray300}`, borderRadius: '5px', padding: '6px', width: '215px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '4px', fontSize: '11px' }}>SAFETY</div>
                {['Joint Limit Checks', 'Workspace Validation', 'E-Stop Handler'].map((t, i) => (
                  <div key={i} style={{ backgroundColor: c.orange, border: `1px solid ${c.orangeBorder}`, borderRadius: '3px', padding: '4px', textAlign: 'center', marginBottom: '2px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '11px' }}>{t}</div>
                  </div>
                ))}
              </div>

              <div style={{ backgroundColor: c.white, border: `1px solid ${c.gray300}`, borderRadius: '5px', padding: '6px', width: '215px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '4px', fontSize: '11px' }}>LOGGING & TELEMETRY</div>
                {['Per-Phase Timing Logs', 'Pick Success Tracking', 'Monitoring Feed'].map((t, i) => (
                  <div key={i} style={{ backgroundColor: c.green, border: `1px solid ${c.greenBorder}`, borderRadius: '3px', padding: '4px', textAlign: 'center', marginBottom: '2px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '11px' }}>{t}</div>
                  </div>
                ))}
              </div>
            </div>
          </div>

          {/* Legend */}
          <div style={{ display: 'flex', justifyContent: 'center', gap: '12px', marginTop: '8px', flexWrap: 'wrap' }}>
            {[
              { color: c.green, label: 'Functional' },
              { color: c.orange, label: 'Safety' },
              { color: c.blue, label: 'Sim Path' },
              { color: c.purple, label: 'Real Path' }
            ].map((item, i) => (
              <div key={i} style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                <div style={{ width: '10px', height: '10px', backgroundColor: item.color, borderRadius: '2px' }}></div>
                <span style={{ color: c.gray600, fontSize: '10px' }}>{item.label}</span>
              </div>
            ))}
          </div>
        </div>

        {/* Right edge: horizontal arrow pointing right */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', alignItems: 'center', paddingLeft: '8px', minWidth: '50px' }}>
          <div style={{ writingMode: 'vertical-rl', textOrientation: 'mixed', transform: 'rotate(180deg)', color: c.gray500, fontSize: '10px', fontWeight: '600', marginBottom: '4px' }}>
            ROS2 Topics / Actions
          </div>
          <div style={{ fontSize: '24px', color: c.gray500, lineHeight: 1 }}>→</div>
        </div>
      </div>
    </div>
  );
};

export default ArchLeft;
