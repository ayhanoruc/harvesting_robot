import React, { useRef, useState } from 'react';

// RoboCot — System Architecture (4 layers)
// Updated 2026-06-04 for Husky + Doosan M1013 + Hand-E stack with the
// layered orchestrator pipeline (row_navigator → cluster_harvester →
// cluster_scanner + simple_cluster_harvester → arm_commander/etc.).
//
// Render in any React playground (e.g. claude.ai/code, codesandbox).
// Click "Download as PNG" — html2canvas captures the diagram div at
// 3× scale and saves it as docs/figures/robocot_system_architecture.png.

const SystemArchitecture = () => {
  const diagramRef = useRef(null);
  const [status, setStatus] = useState('');

  const handleDownload = async () => {
    setStatus('Generating...');
    try {
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
      link.download = 'robocot_system_architecture.png';
      link.href = canvas.toDataURL('image/png');
      link.click();
      setStatus('Downloaded!');
    } catch (err) {
      console.error(err);
      setStatus('Error: ' + err.message);
    }
  };

  const c = {
    green: '#22c55e', greenDark: '#16a34a', greenBorder: '#15803d', greenLight: '#dcfce7',
    orange: '#f97316', orangeBorder: '#ea580c',
    blue: '#2563eb', blueDark: '#1d4ed8', blueBorder: '#1e40af', blueLight: '#dbeafe',
    purple: '#8b5cf6', purpleBorder: '#7c3aed', purpleLight: '#ede9fe',
    teal: '#0d9488', tealBorder: '#0f766e', tealLight: '#ccfbf1',
    gray100: '#f3f4f6', gray300: '#d1d5db', gray400: '#9ca3af', gray500: '#6b7280',
    gray600: '#4b5563', gray700: '#374151', gray800: '#1f2937', white: '#ffffff',
  };

  const Badge = ({ color, borderColor, text, sub, width = '170px' }) => (
    <div style={{ backgroundColor: color, border: `1px solid ${borderColor}`, borderRadius: '6px', padding: '10px', width, textAlign: 'center' }}>
      <div style={{ fontWeight: '600', color: c.white, fontSize: '12px', whiteSpace: 'pre-line' }}>{text}</div>
      {sub && <div style={{ color: 'rgba(255,255,255,0.78)', fontSize: '10px', marginTop: '2px' }}>{sub}</div>}
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

  const Tag = ({ children, color }) => (
    <span style={{ background: color, color: c.white, fontSize: '9px', padding: '1px 6px', borderRadius: '3px', marginRight: '4px', fontWeight: 600 }}>{children}</span>
  );

  // Pipeline node (orchestrator-package layered pipeline rows)
  const PipeNode = ({ tag, name, role, color = c.greenDark, border = c.greenBorder, width = '460px' }) => (
    <div style={{ background: color, border: `2px solid ${border}`, borderRadius: '6px', padding: '8px 10px', width, margin: '0 auto 4px auto' }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '8px' }}>
        <Tag color="rgba(0,0,0,0.35)">{tag}</Tag>
        <div style={{ fontWeight: 700, color: c.white, fontSize: '13px' }}>{name}</div>
      </div>
      <div style={{ textAlign: 'center', color: 'rgba(255,255,255,0.85)', fontSize: '10px', marginTop: '2px' }}>{role}</div>
    </div>
  );

  return (
    <div style={{ backgroundColor: c.white, padding: '16px', fontFamily: 'system-ui, -apple-system, sans-serif' }}>
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', gap: '12px', marginBottom: '16px' }}>
        <button onClick={handleDownload} style={{ backgroundColor: c.blue, color: c.white, fontWeight: '600', padding: '10px 24px', borderRadius: '8px', border: 'none', cursor: 'pointer', fontSize: '14px' }}>
          Download as PNG
        </button>
        {status && <span style={{ color: c.gray600, fontSize: '14px' }}>{status}</span>}
      </div>

      <div ref={diagramRef} style={{ backgroundColor: c.white, padding: '24px' }}>
        <h1 style={{ fontSize: '22px', fontWeight: 'bold', textAlign: 'center', color: c.gray800, marginBottom: '4px', marginTop: 0 }}>
          RoboCot — System Architecture
        </h1>
        <div style={{ textAlign: 'center', color: c.gray500, fontSize: '12px', marginBottom: '20px' }}>
          Husky A200 + Doosan M1013 + Robotiq Hand-E · ME492 Bog̃aziçi University
        </div>

        <div style={{ maxWidth: '960px', margin: '0 auto' }}>

          {/* ============ REMOTE / OPERATOR LAYER ============ */}
          <div style={{ backgroundColor: c.gray100, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '16px', marginBottom: '4px' }}>
            <SectionTitle>REMOTE / OPERATOR LAYER</SectionTitle>
            <div style={{ display: 'flex', justifyContent: 'center', gap: '12px', flexWrap: 'wrap' }}>
              <Badge color={c.green} borderColor={c.greenBorder} text={'PyQt5 Control Panel'} sub={'auto-launches sim · scan/run · WASD + arm teleop · live camera + telemetry'} width="240px" />
              <Badge color={c.green} borderColor={c.greenBorder} text={'Remote Access'} sub={'SSH / VNC into Jetson Xavier'} />
              <Badge color={c.green} borderColor={c.greenBorder} text={'Repo / Deploy'} sub={'git pull → docker build → run'} />
            </div>
          </div>

          <Connector label="WiFi / Ethernet · std_srvs/Trigger" />

          {/* ============ COMPUTE LAYER ============ */}
          <div style={{ backgroundColor: c.gray100, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '16px', marginBottom: '4px' }}>
            <SectionTitle>COMPUTE LAYER — ROS 2 Humble Runtime</SectionTitle>

            {/* Sim vs Real tabs */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', marginBottom: '12px' }}>
              <div style={{ backgroundColor: c.blueLight, border: `1px solid ${c.blueBorder}`, borderRadius: '6px', padding: '6px 14px', fontSize: '11px', fontWeight: '600', color: c.blueDark }}>
                DEV: WSL2 / Ubuntu 22.04 (laptop)
              </div>
              <div style={{ backgroundColor: c.purpleLight, border: `1px solid ${c.purpleBorder}`, borderRadius: '6px', padding: '6px 14px', fontSize: '11px', fontWeight: '600', color: c.purple }}>
                DEPLOY: Docker on Jetson Xavier
              </div>
            </div>

            {/* Layered pipeline */}
            <div style={{ background: c.white, border: `1px dashed ${c.gray400}`, borderRadius: '6px', padding: '10px', marginBottom: '10px' }}>
              <div style={{ textAlign: 'center', fontWeight: 700, color: c.gray700, fontSize: '12px', marginBottom: '6px' }}>
                Orchestrator pipeline (layered)
              </div>
              <PipeNode tag="L3" name="row_navigator" role="route loop — drive Husky to each scout pose (cmd_vel + TF + Gazebo-truth odom recal) → trigger cluster_harvester" />
              <div style={{ textAlign: 'center', color: c.gray500, fontSize: '10px' }}>↓ /cluster_harvester/run</div>
              <PipeNode tag="L2" name="cluster_harvester" role="scan → match detections to YAML boll ids → sort by reach → pick batch → re-scan until empty" />
              <div style={{ textAlign: 'center', color: c.gray500, fontSize: '10px' }}>↓ /cluster_scan/run · /simple_harvest/start</div>
              <div style={{ display: 'flex', justifyContent: 'center', gap: '8px' }}>
                <PipeNode tag="L1" name="cluster_scanner" role="pan/tilt sweep · /yolo/detect + /pixel_to_3d · 3D dedup · gap-rule cluster bbox" color={c.teal} border={c.tealBorder} width="225px" />
                <PipeNode tag="L1" name="simple_cluster_harvester" role="per-boll pick · carry threads keep dropped bolls glued to reservoir as base moves" color={c.teal} border={c.tealBorder} width="225px" />
              </div>
              <div style={{ textAlign: 'center', color: c.gray500, fontSize: '10px' }}>↓ /yolo/detect · /pixel_to_3d · /go_to_pose · /go_to_reservoir · /gripper/&#123;open,close&#125;</div>
              <div style={{ display: 'flex', justifyContent: 'center', gap: '6px', flexWrap: 'wrap' }}>
                <PipeNode tag="L0" name="real_yolo_detector" role="YOLO11 (best.pt)" color={c.greenDark} border={c.greenBorder} width="148px" />
                <PipeNode tag="L0" name="depth_processor" role="K-back-project + TF→world" color={c.greenDark} border={c.greenBorder} width="148px" />
                <PipeNode tag="L0" name="arm_commander" role="IK + JointTrajectory" color={c.greenDark} border={c.greenBorder} width="148px" />
                <PipeNode tag="L0" name="gripper_controller" role="open / close + settle" color={c.greenDark} border={c.greenBorder} width="148px" />
              </div>
            </div>

            {/* External ROS dependencies */}
            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', flexWrap: 'wrap' }}>
              <div style={{ background: c.white, border: `1px solid ${c.gray300}`, borderRadius: '6px', padding: '8px 12px', fontSize: '11px' }}>
                <b style={{ color: c.gray700 }}>MoveIt 2</b> <span style={{ color: c.gray500 }}>· move_group · OMPL RRTstar · KDL IK</span>
              </div>
              <div style={{ background: c.white, border: `1px solid ${c.gray300}`, borderRadius: '6px', padding: '8px 12px', fontSize: '11px' }}>
                <b style={{ color: c.gray700 }}>ros2_control</b> <span style={{ color: c.gray500 }}>· arm_controller · gripper_controller · joint_state_broadcaster</span>
              </div>
              <div style={{ background: c.white, border: `1px solid ${c.gray300}`, borderRadius: '6px', padding: '8px 12px', fontSize: '11px' }}>
                <b style={{ color: c.gray700 }}>harvester_interfaces</b> <span style={{ color: c.gray500 }}>· YoloDetect · PixelTo3D · BoundingBox · DetectedCluster</span>
              </div>
            </div>
          </div>

          {/* ============ HARDWARE INTERFACE LAYER ============ */}
          <Connector label="ROS 2 Topics / Actions / Services" />

          <div style={{ backgroundColor: '#faf5ff', border: `2px solid ${c.purpleBorder}`, borderRadius: '8px', padding: '16px', marginBottom: '4px' }}>
            <SectionTitle>HARDWARE INTERFACE LAYER</SectionTitle>

            <div style={{ display: 'flex', justifyContent: 'center', gap: '10px', flexWrap: 'wrap', marginBottom: '10px' }}>
              {/* Sim path */}
              <div style={{ backgroundColor: c.blueLight, border: `2px solid ${c.blueBorder}`, borderRadius: '8px', padding: '12px', width: '300px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.blueDark, marginBottom: '8px', fontSize: '12px' }}>SIM PATH (current demo)</div>
                {[
                  { t: 'Gazebo Sim · Fortress 6.16', s: 'physics + rendering' },
                  { t: 'gz_ros2_control plugin', s: '6 arm joints + 1 gripper joint' },
                  { t: 'DiffDrive plugin', s: '/cmd_vel → 4 wheel joints (skid-steer)' },
                  { t: 'RGB-D camera plugin', s: '640×480, 90° HFOV, 10 Hz · K matrix' },
                  { t: 'set_pose teleport (bolls)', s: 'mock grip — boll → TCP / reservoir' },
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.white, border: `1px solid ${c.blueBorder}`, borderRadius: '5px', padding: '6px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '600', color: c.blueDark, fontSize: '11px' }}>{item.t}</div>
                    <div style={{ color: c.gray500, fontSize: '10px' }}>{item.s}</div>
                  </div>
                ))}
              </div>

              {/* Real path */}
              <div style={{ backgroundColor: c.purpleLight, border: `2px solid ${c.purpleBorder}`, borderRadius: '8px', padding: '12px', width: '300px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.purple, marginBottom: '8px', fontSize: '12px' }}>REAL PATH (deployment plan)</div>
                {[
                  { t: 'doosan-robot2 driver', s: 'TCP 192.168.3.5:12345 (DRFL)' },
                  { t: 'Hand-E Modbus RTU', s: 'RS485-USB → /dev/ttyUSB0 (pymodbus)' },
                  { t: 'Husky base controller', s: 'A200 firmware via /cmd_vel + /odom' },
                  { t: 'RealSense / ZED X Mini', s: 'USB3 → /camera/* topics' },
                  { t: 'Docker container', s: '--network host · --runtime nvidia · --privileged' },
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.white, border: `1px solid ${c.purpleBorder}`, borderRadius: '5px', padding: '6px', textAlign: 'center', marginBottom: '4px' }}>
                    <div style={{ fontWeight: '600', color: c.purple, fontSize: '11px' }}>{item.t}</div>
                    <div style={{ color: c.gray500, fontSize: '10px' }}>{item.s}</div>
                  </div>
                ))}
              </div>
            </div>
          </div>

          {/* ============ Connector to Physical ============ */}
          <Connector label="Ethernet · USB · servo cabling" />

          {/* ============ PHYSICAL LAYER ============ */}
          <div style={{ backgroundColor: c.gray100, border: `2px solid ${c.gray300}`, borderRadius: '8px', padding: '16px' }}>
            <SectionTitle>PHYSICAL LAYER</SectionTitle>
            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', flexWrap: 'wrap' }}>
              {[
                { name: 'Husky A200', desc: '4-wheel skid-steer base', type: 'PLATFORM', note: '0.99×0.67×0.30 m · 50 kg' },
                { name: 'Doosan M1013', desc: '6-DOF industrial arm', type: 'ACTUATOR', note: '1.3 m reach · ±0.03 mm rep.' },
                { name: 'Doosan Controller', desc: 'AC/DC servo box', type: 'CONTROLLER', note: '192.168.3.5:12345' },
                { name: 'Robotiq Hand-E', desc: 'parallel jaw gripper', type: 'ACTUATOR', note: '0–25 mm stroke · 1.1 kg' },
                { name: 'Wrist RGB-D', desc: 'eye-in-hand camera', type: 'SENSOR', note: 'ZED X Mini target' },
                { name: 'Reservoir Bin', desc: 'open-top collection box', type: 'STORAGE', note: '0.4×0.4×0.2 m · deck-mounted' },
                { name: 'Cotton Orchard', desc: 'mock field environment', type: 'ENV', note: '~3 m rows · 1.5 m tree pitch' },
              ].map((item, i) => (
                <div key={i} style={{ backgroundColor: c.green, border: `2px solid ${c.greenBorder}`, borderRadius: '6px', padding: '8px', width: '128px', textAlign: 'center' }}>
                  <div style={{ fontWeight: 'bold', color: c.white, fontSize: '11px' }}>{item.name}</div>
                  <div style={{ color: c.greenLight, fontSize: '9.5px' }}>{item.desc}</div>
                  <div style={{ color: 'rgba(255,255,255,0.65)', fontSize: '9px', marginTop: '2px' }}>{item.note}</div>
                  <div style={{ marginTop: '6px', backgroundColor: c.white, color: c.greenDark, fontSize: '9px', padding: '2px 5px', borderRadius: '4px', fontWeight: '600', display: 'inline-block' }}>{item.type}</div>
                </div>
              ))}
            </div>
          </div>

          {/* ============ LEGEND ============ */}
          <div style={{ display: 'flex', justifyContent: 'center', gap: '16px', marginTop: '14px', paddingTop: '12px', borderTop: `1px solid ${c.gray300}`, flexWrap: 'wrap' }}>
            {[
              { color: c.green, label: 'Orchestrator package' },
              { color: c.teal, label: 'L1/L2 service node' },
              { color: c.blue, label: 'Sim path' },
              { color: c.purple, label: 'Real / deploy path' },
            ].map((item, i) => (
              <div key={i} style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <div style={{ width: '14px', height: '14px', backgroundColor: item.color, borderRadius: '3px' }}></div>
                <span style={{ color: c.gray600, fontSize: '12px' }}>{item.label}</span>
              </div>
            ))}
          </div>

          <div style={{ textAlign: 'center', color: c.gray400, fontSize: '10px', marginTop: '8px' }}>
            ME492 Mechanical Design — Group 6 — Bog̃aziçi University · 2026
          </div>
        </div>
      </div>
    </div>
  );
};

export default SystemArchitecture;
