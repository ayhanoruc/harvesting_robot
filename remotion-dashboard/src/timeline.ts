// Timeline keyframes — maps video seconds to dashboard state.
// User will fill in exact seconds after editing the video in CapCut.
// Each keyframe says: "at this second in the video, show this state."

export interface Keyframe {
  sec: number;           // second in the final video
  sectionId: string;     // which section from timeline.json
  eventIndex?: number;   // which event within that section to highlight
  image?: string;        // override image to show (filename in public/)
  stepHighlight?: string; // e.g. "3/8" to highlight a specific step
}

// PLACEHOLDER keyframes — user will replace with real timings
const keyframes: Keyframe[] = [
  { sec: 0,  sectionId: "startup" },
  { sec: 5,  sectionId: "scanning", image: "scan_left.png" },
  { sec: 15, sectionId: "scanning", image: "scan_center.png" },
  { sec: 25, sectionId: "scanning", image: "scan_right.png" },
  { sec: 35, sectionId: "approach_cluster3", image: "cluster3.png" },
  { sec: 45, sectionId: "harvest_cluster3", stepHighlight: "1/8" },
  { sec: 50, sectionId: "harvest_cluster3", stepHighlight: "3/8" },
  { sec: 55, sectionId: "harvest_cluster3", stepHighlight: "5/8" },
  { sec: 60, sectionId: "harvest_cluster3", stepHighlight: "6/8" },
  { sec: 65, sectionId: "approach_cluster2", image: "cluster2.png" },
  { sec: 75, sectionId: "harvest_cluster2", stepHighlight: "1/8" },
  { sec: 80, sectionId: "harvest_cluster2", stepHighlight: "3/8" },
  { sec: 85, sectionId: "harvest_cluster2", stepHighlight: "6/8" },
  { sec: 90, sectionId: "approach_cluster1", image: "cluster1.png" },
  { sec: 100, sectionId: "harvest_cluster1", stepHighlight: "1/8" },
  { sec: 105, sectionId: "harvest_cluster1", stepHighlight: "3/8" },
  { sec: 110, sectionId: "harvest_cluster1", stepHighlight: "6/8" },
  { sec: 115, sectionId: "complete" },
];

export default keyframes;
