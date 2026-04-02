import React, { useRef, useState } from 'react';

const SystemArchitecture = () => {
  const diagramRef = useRef(null);
  const [status, setStatus] = useState('');

  const handleDownload = async () => {
    setStatus('Generating...');
    try {
      // Use html2canvas via CDN script injection
      if (!window.html2canvas) {
        await new Promise((resolve, reject) => {
          const script = document.createElement('script');
          script.src = 'https://cdn.jsdelivr.net/npm/html2canvas@1.4.1/dist/html2canvas.min.js';
          script.onload = resolve;
          script.onerror = reject;
          document.head.appendChild(script);
        });
      }
      const canvas = await window.html2canvas(diagramRef.current, {
        scale: 3,
        backgroundColor: '#ffffff',
        useCORS: true
      });
      const link = document.createElement('a');
      link.download = 'robocot-system-architecture-v2.png';
      link.href = canvas.toDataURL('image/png');
      link.click();
      setStatus('Downloaded!');
    } catch (err) {
      console.error(err);
      setStatus('Error: ' + err.message);
    }
  };

  const c = {
    green: '#22c55e',
    greenDark: '#16a34a',
    greenBorder: '#15803d',
    greenLight: '#dcfce7',
    orange: '#f97316',
    orangeBorder: '#ea580c',
    blue: '#2563eb',
    blueDark: '#1d4ed8',
    blueBorder: '#1e40af',
    blueLight: '#dbeafe',
    purple: '#8b5cf6',
    purpleBorder: '#7c3aed',
    purpleLight: '#ede9fe',
    gray100: '#f3f4f6',
    gray300: '#d1d5db',
    gray400: '#9ca3af',
    gray500: '#6b7280',
    gray600: '#4b5563',
    gray700: '#374151',
    gray800: '#1f2937',
    white: '#ffffff',
  };

  const Badge = ({ color, borderColor, text, sub }) => (
    <div style={{ backgroundColor: color, border: `1px solid ${borderColor}`, borderRadius: '6px', padding: '10px', width: '170px', textAlign: 'center' }}>
      <div style={{ fontWeight: '600', color: c.white, fontSize: '12px', whiteSpace: 'pre-line' }}>{text}</div>
      {sub && <div style={{ color: 'rgba(255,255,255,0.75)', fontSize: '10px', marginTop: '2px' }}>{sub}</div>}
    </div>
  );

  const Connector = ({ label }) => (
    <div style={{ display: 'flex', justifyContent: 'center', padding: '2px 0' }}>
      <div style={{ textAlign: 'center' }}>
        <div style={{ fontSize: '20px', color: c.gray500, lineHeight: 1 }}>↕</div>
        <div style={{ backgroundColor: c.gray500, color: c.white, fontSize: '11px', padding: '3px 10px', borderRadius: '10px', fontWeight: '500' }}>{label}</div>
      </div>
    </div>
  );

  const SectionTitle = ({ children }) => (
    <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '12px', fontSize: '13px' }}>{children}</div>
  );

  return (
    <div style={{ backgroundColor: c.white, padding: '16px', fontFamily: 'system-ui, -apple-system, sans-serif' }}>
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', gap: '12px', marginBottom: '16px' }}>
        <button onClick={handleDownload} style={{ backgroundColor: c.blue, color: c.white, fontWeight: '600', padding: '10px 24px', borderRadius: '8px', border: 'none', cursor: 'pointer', fontSize: '14px' }}>
          ⬇️ Download as PNG
        </button>
        {status && <span style={{ color: c.gray600, fontSize: '14px' }}>{status}</span>}
      </div>

      <div ref={diagramRef} style={{ backgroundColor: c.white, padding: '24px' }}>
        <h1 style={{ fontSize: '22px', fontWeight: 'bold', textAlign: 'center', color: c.gray800, marginBottom: '4px', marginTop: 0 }}>
          RoboCot: System Architecture
        </h1>
        <div style={{ textAlign: 'center', color: c.gray500, fontSize: '12px', marginBottom: '20px' }}>
          ME492 — Simulation + Real Robot Deployment
        </div>

        <div style={{ maxWidth: '920px', margin: '0 auto' }}>

          {/* ============ REMOTE / OPERATOR LAYER ============ */}
          <div style={{ backgroundColor: c.gray100, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '16px', marginBottom: '4px' }}>
            <SectionTitle>REMOTE / OPERATOR LAYER</SectionTitle>
            <div style={{ display: 'flex', justifyContent: 'center', gap: '12px', flexWrap: 'wrap' }}>
              <Badge color={c.green} borderColor={c.greenBorder} text={'Monitoring &\nControl App'} />
              <Badge color={c.green} borderColor={c.greenBorder} text={'Remote Access'} sub={'SSH / VNC into Xavier'} />
              <Badge color={c.green} borderColor={c.greenBorder} text={'System Updates'} sub={'Git pull → Docker rebuild'} />
            </div>
          </div>

          <Connector label="WiFi / Ethernet" />

          {/* ============ COMPUTE LAYER ============ */}
          <div style={{ backgroundColor: c.gray100, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '16px', marginBottom: '4px' }}>
            <SectionTitle>COMPUTE LAYER — ROS2 Humble Runtime</SectionTitle>

            {/* Sim vs Real tabs */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', marginBottom: '12px' }}>
              <div style={{ backgroundColor: c.blueLight, border: `1px solid ${c.blueBorder}`, borderRadius: '6px', padding: '6px 14px', fontSize: '11px', fontWeight: '600', color: c.blueDark }}>
                SIM: WSL2 / Ubuntu on Laptop
              </div>
              <div style={{ backgroundColor: c.purpleLight, border: `1px solid ${c.purpleBorder}`, borderRadius: '6px', padding: '6px 14px', fontSize: '11px', fontWeight: '600', color: c.purple }}>
                REAL: Docker on Jetson Xavier
              </div>
            </div>

            {/* Orchestrator */}
            <div style={{ backgroundColor: c.greenDark, border: `2px solid ${c.greenBorder}`, borderRadius: '8px', padding: '10px', maxWidth: '520px', margin: '0 auto 10px auto' }}>
              <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.white, fontSize: '15px' }}>ORCHESTRATOR NODE (FSM)</div>
              <div style={{ textAlign: 'center', color: c.greenLight, fontSize: '11px', marginTop: '4px' }}>
                IDLE → SCANNING → APPROACHING → HARVESTING → RETURNING
              </div>
            </div>

            <div style={{ display: 'flex', justifyContent: 'center', gap: '40px', marginBottom: '10px' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: '6px', color: c.gray600, fontWeight: '600', fontSize: '13px' }}>
                <span style={{ fontSize: '18px' }}>↑</span> events
              </div>
              <div style={{ display: 'flex', alignItems: 'center', gap: '6px', color: c.gray600, fontWeight: '600', fontSize: '13px' }}>
                commands <span style={{ fontSize: '18px' }}>↓</span>
              </div>
            </div>

            {/* Middle Row: Perception + Planning */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '12px', marginBottom: '10px', flexWrap: 'wrap' }}>
              <div style={{ backgroundColor: c.white, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '10px', width: '250px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '8px', fontSize: '13px' }}>PERCEPTION</div>
                {[
                  { t: 'YOLO11 Detector', s: '(cotton_boll + cluster)' },
                  { t: '3D Positioning', s: '(Depth + TF → World)' },
                  { t: 'World-Space Clustering', s: '(Complete-linkage)' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.green, border: `1px solid ${c.greenBorder}`, borderRadius: '5px', padding: '8px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '12px' }}>{item.t}</div>
                    {item.s && <div style={{ color: c.greenLight, fontSize: '10px' }}>{item.s}</div>}
                  </div>
                ))}
              </div>

              <div style={{ backgroundColor: c.white, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '10px', width: '250px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '8px', fontSize: '13px' }}>PLANNING & CONTROL</div>
                {[
                  { t: 'IK Multi-Seed + Joint Goal', s: '(arm_commander)' },
                  { t: 'MoveIt2 / OMPL', s: '(path planning fallback)' },
                  { t: 'Gripper Controller', s: '(open / close + stall detect)' },
                  { t: 'Harvest Executor', s: '(8-step pick-and-place)' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.green, border: `1px solid ${c.greenBorder}`, borderRadius: '5px', padding: '8px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '12px' }}>{item.t}</div>
                    {item.s && <div style={{ color: c.greenLight, fontSize: '10px' }}>{item.s}</div>}
                  </div>
                ))}
              </div>
            </div>

            {/* Bottom Row: Safety + Logging */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '12px', flexWrap: 'wrap' }}>
              <div style={{ backgroundColor: c.white, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '10px', width: '250px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '8px', fontSize: '13px' }}>SAFETY</div>
                {['Joint Limit Checks', 'Workspace Validation', 'E-Stop Handler'].map((t, i) => (
                  <div key={i} style={{ backgroundColor: c.orange, border: `1px solid ${c.orangeBorder}`, borderRadius: '5px', padding: '8px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '12px' }}>{t}</div>
                  </div>
                ))}
              </div>

              <div style={{ backgroundColor: c.white, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '10px', width: '250px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.gray700, marginBottom: '8px', fontSize: '13px' }}>LOGGING & TELEMETRY</div>
                {[
                  { t: 'Per-Phase Timing Logs', s: '' },
                  { t: 'Pick Success Tracking', s: '' },
                  { t: 'Monitoring Feed', s: '' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.green, border: `1px solid ${c.greenBorder}`, borderRadius: '5px', padding: '8px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '500', color: c.white, fontSize: '12px' }}>{item.t}</div>
                  </div>
                ))}
              </div>
            </div>
          </div>

          {/* ============ HARDWARE INTERFACE LAYER (NEW) ============ */}
          <Connector label="ROS2 Topics / Actions" />

          <div style={{ backgroundColor: '#faf5ff', border: `2px solid ${c.purpleBorder}`, borderRadius: '8px', padding: '16px', marginBottom: '4px' }}>
            <SectionTitle>HARDWARE INTERFACE LAYER</SectionTitle>

            <div style={{ display: 'flex', justifyContent: 'center', gap: '10px', flexWrap: 'wrap', marginBottom: '10px' }}>
              {/* Sim path */}
              <div style={{ backgroundColor: c.blueLight, border: `2px solid ${c.blueBorder}`, borderRadius: '8px', padding: '12px', width: '260px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.blueDark, marginBottom: '8px', fontSize: '12px' }}>SIMULATION PATH</div>
                {[
                  { t: 'Gazebo Ignition Fortress', s: 'Physics + Rendering' },
                  { t: 'gz_ros2_control', s: 'Joint position interface' },
                  { t: 'Gazebo RGB-D Sensor', s: '/camera/* topics' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.white, border: `1px solid ${c.blueBorder}`, borderRadius: '5px', padding: '7px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '600', color: c.blueDark, fontSize: '11px' }}>{item.t}</div>
                    <div style={{ color: c.gray500, fontSize: '10px' }}>{item.s}</div>
                  </div>
                ))}
              </div>

              {/* Real path */}
              <div style={{ backgroundColor: c.purpleLight, border: `2px solid ${c.purpleBorder}`, borderRadius: '8px', padding: '12px', width: '260px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.purple, marginBottom: '8px', fontSize: '12px' }}>REAL ROBOT PATH</div>
                {[
                  { t: 'doosan-robot2 Driver', s: 'TCP → 192.168.3.5:12345' },
                  { t: 'Hand-E Modbus RTU', s: 'RS485-USB → /dev/ttyUSB0' },
                  { t: 'RealSense Camera Driver', s: 'USB3 → /camera/* topics' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.white, border: `1px solid ${c.purpleBorder}`, borderRadius: '5px', padding: '7px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '600', color: c.purple, fontSize: '11px' }}>{item.t}</div>
                    <div style={{ color: c.gray500, fontSize: '10px' }}>{item.s}</div>
                  </div>
                ))}
              </div>
            </div>

            {/* Docker note */}
            <div style={{ textAlign: 'center', backgroundColor: c.white, border: `1px dashed ${c.purpleBorder}`, borderRadius: '6px', padding: '6px 12px', maxWidth: '400px', margin: '0 auto' }}>
              <span style={{ fontSize: '11px', color: c.purple, fontWeight: '500' }}>
                Real: Docker container on Xavier — <code style={{ fontSize: '10px' }}>--network host --runtime nvidia --privileged</code>
              </span>
            </div>
          </div>

          {/* ============ Connector to Physical ============ */}
          <Connector label="Ethernet / USB / EtherCAT" />

          {/* ============ PHYSICAL LAYER ============ */}
          <div style={{ backgroundColor: c.gray100, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '16px' }}>
            <SectionTitle>PHYSICAL LAYER</SectionTitle>
            <div style={{ display: 'flex', justifyContent: 'center', gap: '10px', flexWrap: 'wrap' }}>
              {[
                { name: 'Doosan M1013', desc: '6-DOF Industrial Arm', type: 'ACTUATOR', note: '0.05mm repeatability' },
                { name: 'Robotiq Hand-E', desc: 'Parallel Jaw Gripper', type: 'ACTUATOR', note: '0-25mm stroke' },
                { name: 'Wrist RGB-D', desc: 'Eye-in-Hand Camera', type: 'SENSOR', note: '640×480 @ 30fps' },
                { name: 'Doosan Controller', desc: 'AC/DC Servo Box', type: 'CONTROLLER', note: '192.168.3.5' },
                { name: 'Reservoir Bin', desc: 'Collection Box', type: 'STORAGE', note: '150×150×150mm' }
              ].map((item, i) => (
                <div key={i} style={{ backgroundColor: c.green, border: `2px solid ${c.greenBorder}`, borderRadius: '6px', padding: '10px', width: '140px', textAlign: 'center' }}>
                  <div style={{ fontWeight: 'bold', color: c.white, fontSize: '12px' }}>{item.name}</div>
                  <div style={{ color: c.greenLight, fontSize: '10px' }}>{item.desc}</div>
                  <div style={{ color: 'rgba(255,255,255,0.6)', fontSize: '9px', marginTop: '2px' }}>{item.note}</div>
                  <div style={{ marginTop: '6px', backgroundColor: c.white, color: c.greenDark, fontSize: '10px', padding: '2px 6px', borderRadius: '4px', fontWeight: '600', display: 'inline-block' }}>{item.type}</div>
                </div>
              ))}
            </div>
          </div>

          {/* ============ LEGEND ============ */}
          <div style={{ display: 'flex', justifyContent: 'center', gap: '16px', marginTop: '14px', paddingTop: '12px', borderTop: `1px solid ${c.gray300}`, flexWrap: 'wrap' }}>
            {[
              { color: c.green, label: 'Functional Module' },
              { color: c.orange, label: 'Safety Module' },
              { color: c.blue, label: 'Simulation Path' },
              { color: c.purple, label: 'Real Robot Path' }
            ].map((item, i) => (
              <div key={i} style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: '14px', height: '14px', backgroundColor: item.color, borderRadius: '3px' }}></div>
                <span style={{ color: c.gray600, fontSize: '12px' }}>{item.label}</span>
              </div>
            ))}
          </div>

          <div style={{ textAlign: 'center', color: c.gray400, fontSize: '10px', marginTop: '8px' }}>
            ME492 Mechanical and Thermal Design — Group 6 — Boğaziçi University
          </div>
        </div>
      </div>
    </div>
  );
};

export default SystemArchitecture;
