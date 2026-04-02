import { Composition, staticFile } from "remotion";
import { Dashboard } from "./Dashboard";
import timeline from "./timeline";

// Video is 30fps. Duration will be set after user provides final video length.
// For now, use a placeholder — user will adjust.
const FPS = 30;
const DURATION_SEC = 120; // placeholder — will match final edited video

export const Root: React.FC = () => {
  return (
    <Composition
      id="Dashboard"
      component={Dashboard}
      durationInFrames={DURATION_SEC * FPS}
      fps={FPS}
      width={1920}
      height={1080}
      defaultProps={{
        timeline,
        videoSrc: staticFile("demo.mp4"),
      }}
    />
  );
};
