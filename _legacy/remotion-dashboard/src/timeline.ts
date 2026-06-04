export interface Keyframe {
  sec: number;
  sectionId: string;
  eventIndex?: number;
  image?: string;
  stepHighlight?: string;
  pickNumber?: number;  // which boll (1-12)
  pickTotal?: number;   // total bolls in cluster
}

const STEPS = ["1/8", "2/8", "3/8", "4/8", "5/8", "6/8", "7/8", "8/8"];

// Generate repeating pick cycles for a cluster
function generatePicks(
  startSec: number,
  endSec: number,
  sectionId: string,
  image: string,
  totalBolls: number,
  approachDelay: number = 5,
): Keyframe[] {
  const duration = endSec - startSec;
  const pickStart = startSec + approachDelay;
  const pickDuration = endSec - pickStart;
  const secPerPick = pickDuration / totalBolls;

  const kfs: Keyframe[] = [
    { sec: startSec, sectionId: sectionId.replace("harvest_", "approach_"), image },
  ];

  for (let pick = 0; pick < totalBolls; pick++) {
    const cycleStart = pickStart + pick * secPerPick;
    const stepDur = secPerPick / STEPS.length;

    for (let s = 0; s < STEPS.length; s++) {
      kfs.push({
        sec: Math.round((cycleStart + s * stepDur) * 100) / 100,
        sectionId,
        image,
        stepHighlight: STEPS[s],
        pickNumber: pick + 1,
        pickTotal: totalBolls,
      });
    }
  }

  return kfs;
}

const keyframes: Keyframe[] = [
  // ── Startup ──
  { sec: 0, sectionId: "startup" },

  // ── Scanning ──
  { sec: 3,  sectionId: "scanning", image: "scan_right.png" },
  { sec: 9,  sectionId: "scanning", image: "scan_center.png" },
  { sec: 12, sectionId: "scanning", image: "scan_left.png" },

  // ── Cluster 1: 17–42 (25s), 12 bolls ──
  ...generatePicks(17, 42, "harvest_cluster1", "cluster1.png", 12, 8),

  // ── Cluster 2: 42–65 (23s), 12 bolls ──
  ...generatePicks(42, 65, "harvest_cluster2", "cluster2.png", 12, 5),

  // ── Cluster 3: 65–85 (20s), 12 bolls ──
  ...generatePicks(65, 88, "harvest_cluster3", "cluster3.png", 12, 11),
];

export default keyframes;
