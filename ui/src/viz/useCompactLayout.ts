import { useEffect, useState } from "react";

const COMPACT_QUERY = "(max-height: 900px), (max-width: 1280px)";

export function useCompactLayout(): boolean {
  const [compact, setCompact] = useState(
    () => typeof window !== "undefined" && window.matchMedia(COMPACT_QUERY).matches,
  );

  useEffect(() => {
    const media = window.matchMedia(COMPACT_QUERY);
    const onChange = () => setCompact(media.matches);
    onChange();
    media.addEventListener("change", onChange);
    return () => media.removeEventListener("change", onChange);
  }, []);

  return compact;
}

export function useViewportHeight(): number {
  const [height, setHeight] = useState(
    () => typeof window !== "undefined" && window.visualViewport?.height
      ? window.visualViewport.height
      : window.innerHeight,
  );

  useEffect(() => {
    const update = () => {
      setHeight(window.visualViewport?.height ?? window.innerHeight);
    };
    update();
    window.visualViewport?.addEventListener("resize", update);
    window.addEventListener("resize", update);
    return () => {
      window.visualViewport?.removeEventListener("resize", update);
      window.removeEventListener("resize", update);
    };
  }, []);

  return height;
}
