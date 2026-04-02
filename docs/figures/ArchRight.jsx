import React, { useRef, useState } from 'react';

const ArchRight = () => {
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
      link.download = 'arch-right-hardware.png';
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
    <div style={{ backgroundColor: c.white, padding: '8px', fontFamily: 'system-ui, -apple-system, sans-serif' }}>
      <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', marginBottom: '8px' }}>
        <button onClick={handleDownload} style={{ backgroundColor: c.purple, color: c.white, fontWeight: '600', padding: '8px 20px', borderRadius: '6px', border: 'none', cursor: 'pointer', fontSize: '13px' }}>
          Download Right PNG
        </button>
        {status && <span style={{ color: c.gray600, fontSize: '13px' }}>{status}</span>}
      </div>

      <div ref={diagramRef} style={{ backgroundColor: c.white, padding: '8px', display: 'flex', flexDirection: 'row', alignItems: 'stretch' }}>

        {/* Left edge: horizontal arrow pointing right (from left diagram) */}
        <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', alignItems: 'center', paddingRight: '8px', minWidth: '50px' }}>
          <div style={{ writingMode: 'vertical-rl', textOrientation: 'mixed', transform: 'rotate(180deg)', color: c.gray500, fontSize: '10px', fontWeight: '600', marginBottom: '4px' }}>
            ROS2 Topics / Actions
          </div>
          <div style={{ fontSize: '24px', color: c.gray500, lineHeight: 1 }}>→</div>
        </div>

        {/* Main content column */}
        <div style={{ flex: 1, minWidth: 0 }}>
          <h2 style={{ fontSize: '18px', fontWeight: 'bold', textAlign: 'center', color: c.gray800, margin: '0 0 2px 0' }}>
            Hardware & Deployment
          </h2>
          <div style={{ textAlign: 'center', color: c.gray500, fontSize: '11px', marginBottom: '10px' }}>
            Sim ↔ Real: Same ROS2 interface, swap drivers
          </div>

          {/* HARDWARE INTERFACE LAYER */}
          <div style={{ backgroundColor: '#faf5ff', border: `2px solid ${c.purpleBorder}`, borderRadius: '6px', padding: '10px', marginBottom: '4px' }}>
            <SectionTitle>HARDWARE INTERFACE LAYER</SectionTitle>

            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', flexWrap: 'wrap', marginBottom: '8px' }}>
              {/* Sim path */}
              <div style={{ backgroundColor: c.blueLight, border: `2px solid ${c.blueBorder}`, borderRadius: '6px', padding: '10px', width: '220px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.blueDark, marginBottom: '6px', fontSize: '11px' }}>SIMULATION PATH</div>
                {[
                  { t: 'Gazebo Ign Fortress', s: 'Physics + Rendering' },
                  { t: 'gz_ros2_control', s: 'Joint position interface' },
                  { t: 'Gazebo RGB-D Sensor', s: '/camera/* topics' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.white, border: `1px solid ${c.blueBorder}`, borderRadius: '4px', padding: '6px', textAlign: 'center', marginBottom: '3px' }}>
                    <div style={{ fontWeight: '600', color: c.blueDark, fontSize: '10px' }}>{item.t}</div>
                    <div style={{ color: c.gray500, fontSize: '9px' }}>{item.s}</div>
                  </div>
                ))}
              </div>

              {/* Real path */}
              <div style={{ backgroundColor: c.purpleLight, border: `2px solid ${c.purpleBorder}`, borderRadius: '6px', padding: '10px', width: '220px' }}>
                <div style={{ textAlign: 'center', fontWeight: 'bold', color: c.purple, marginBottom: '6px', fontSize: '11px' }}>REAL ROBOT PATH</div>
                {[
                  { t: 'doosan-robot2 Driver', s: 'TCP → 192.168.3.5:12345' },
                  { t: 'Hand-E Modbus RTU', s: 'RS485 → /dev/ttyUSB0' },
                  { t: 'RealSense Driver', s: 'USB3 → /camera/* topics' }
                ].map((item, i) => (
                  <div key={i} style={{ backgroundColor: c.white, border: `1px solid ${c.purpleBorder}`, borderRadius: '4px', padding: '6px', textAlign: 'center', marginBottom: '3px' }}>
                    <div style={{ fontWeight: '600', color: c.purple, fontSize: '10px' }}>{item.t}</div>
                    <div style={{ color: c.gray500, fontSize: '9px' }}>{item.s}</div>
                  </div>
                ))}
              </div>
            </div>

            {/* Docker note */}
            <div style={{ textAlign: 'center', backgroundColor: c.white, border: `1px dashed ${c.purpleBorder}`, borderRadius: '5px', padding: '5px 10px', maxWidth: '380px', margin: '0 auto' }}>
              <span style={{ fontSize: '10px', color: c.purple, fontWeight: '500' }}>
                Real: Docker on Jetson Xavier — <code style={{ fontSize: '9px' }}>--network host --runtime nvidia</code>
              </span>
            </div>
          </div>

          {/* Connector */}
          <div style={{ textAlign: 'center', padding: '1px 0' }}>
            <div style={{ fontSize: '16px', color: c.gray500, lineHeight: 1 }}>↕</div>
            <div style={{ display: 'inline-block', backgroundColor: c.gray500, color: c.white, fontSize: '10px', padding: '2px 8px', borderRadius: '8px' }}>Ethernet / USB / RS485</div>
          </div>

          {/* PHYSICAL LAYER */}
          <div style={{ backgroundColor: c.gray100, border: `2px solid ${c.gray300}`, borderRadius: '6px', padding: '10px', marginBottom: '4px' }}>
            <SectionTitle>PHYSICAL LAYER</SectionTitle>
            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', flexWrap: 'wrap' }}>
              {[
                { name: 'Doosan M1013', desc: '6-DOF Industrial Arm', type: 'ACTUATOR', note: '0.05mm repeat.' },
                { name: 'Robotiq Hand-E', desc: 'Parallel Jaw Gripper', type: 'ACTUATOR', note: '0-25mm stroke' },
                { name: 'Wrist RGB-D', desc: 'Eye-in-Hand Camera', type: 'SENSOR', note: '640×480 @30fps' },
              ].map((item, i) => (
                <div key={i} style={{ backgroundColor: c.green, border: `2px solid ${c.greenBorder}`, borderRadius: '5px', padding: '8px', width: '130px', textAlign: 'center' }}>
                  <div style={{ fontWeight: 'bold', color: c.white, fontSize: '11px' }}>{item.name}</div>
                  <div style={{ color: c.greenLight, fontSize: '9px' }}>{item.desc}</div>
                  <div style={{ color: 'rgba(255,255,255,0.6)', fontSize: '8px', marginTop: '1px' }}>{item.note}</div>
                  <div style={{ marginTop: '4px', backgroundColor: c.white, color: c.greenDark, fontSize: '9px', padding: '1px 5px', borderRadius: '3px', fontWeight: '600', display: 'inline-block' }}>{item.type}</div>
                </div>
              ))}
            </div>
            <div style={{ display: 'flex', justifyContent: 'center', gap: '8px', flexWrap: 'wrap', marginTop: '6px' }}>
              {[
                { name: 'Doosan Controller', desc: 'Servo AC/DC Box', type: 'CONTROLLER', note: '192.168.3.5' },
                { name: 'Reservoir Bin', desc: 'Collection Box', type: 'STORAGE', note: '150×150×150mm' }
              ].map((item, i) => (
                <div key={i} style={{ backgroundColor: c.green, border: `2px solid ${c.greenBorder}`, borderRadius: '5px', padding: '8px', width: '130px', textAlign: 'center' }}>
                  <div style={{ fontWeight: 'bold', color: c.white, fontSize: '11px' }}>{item.name}</div>
                  <div style={{ color: c.greenLight, fontSize: '9px' }}>{item.desc}</div>
                  <div style={{ color: 'rgba(255,255,255,0.6)', fontSize: '8px', marginTop: '1px' }}>{item.note}</div>
                  <div style={{ marginTop: '4px', backgroundColor: c.white, color: c.greenDark, fontSize: '9px', padding: '1px 5px', borderRadius: '3px', fontWeight: '600', display: 'inline-block' }}>{item.type}</div>
                </div>
              ))}
            </div>
          </div>

          {/* Xavier detail box */}
          <div style={{ backgroundColor: c.purpleLight, border: `2px solid ${c.purpleBorder}`, borderRadius: '6px', padding: '10px' }}>
            <SectionTitle>DEPLOYMENT: Jetson AGX Xavier</SectionTitle>
            <div style={{ display: 'flex', gap: '8px', flexWrap: 'wrap', justifyContent: 'center' }}>
              <div style={{ backgroundColor: c.white, border: `1px solid ${c.purpleBorder}`, borderRadius: '5px', padding: '8px', width: '200px' }}>
                <div style={{ fontWeight: 'bold', color: c.purple, fontSize: '11px', marginBottom: '4px' }}>Docker Container</div>
                {['ROS2 Humble', 'MoveIt2 + OMPL', 'YOLO11 (CUDA)', 'doosan-robot2', 'Our orchestrator'].map((t, i) => (
                  <div key={i} style={{ color: c.gray700, fontSize: '10px', padding: '1px 0' }}>• {t}</div>
                ))}
              </div>
              <div style={{ backgroundColor: c.white, border: `1px solid ${c.purpleBorder}`, borderRadius: '5px', padding: '8px', width: '200px' }}>
                <div style={{ fontWeight: 'bold', color: c.purple, fontSize: '11px', marginBottom: '4px' }}>Host (Ubuntu 20.04)</div>
                {['NVIDIA CUDA runtime', 'Docker engine', 'ROBOCOB ROS1 stack', '(untouched, coexists)'].map((t, i) => (
                  <div key={i} style={{ color: c.gray700, fontSize: '10px', padding: '1px 0' }}>• {t}</div>
                ))}
              </div>
            </div>
            <div style={{ textAlign: 'center', marginTop: '6px' }}>
              <span style={{ fontSize: '10px', color: c.gray500 }}>512 CUDA cores • 64GB unified memory • Same 192.168.3.x subnet</span>
            </div>
          </div>

          {/* Legend */}
          <div style={{ display: 'flex', justifyContent: 'center', gap: '12px', marginTop: '8px', flexWrap: 'wrap' }}>
            {[
              { color: c.green, label: 'Functional' },
              { color: c.blue, label: 'Sim Path' },
              { color: c.purple, label: 'Real Path' }
            ].map((item, i) => (
              <div key={i} style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
                <div style={{ width: '10px', height: '10px', backgroundColor: item.color, borderRadius: '2px' }}></div>
                <span style={{ color: c.gray600, fontSize: '10px' }}>{item.label}</span>
              </div>
            ))}
          </div>

          <div style={{ textAlign: 'center', color: c.gray400, fontSize: '9px', marginTop: '4px' }}>
            ME492 — Group 6 — Boğaziçi University
          </div>
        </div>
      </div>
    </div>
  );
};

export default ArchRight;
