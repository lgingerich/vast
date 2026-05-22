import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { BrowserRouter, Route, Routes } from "react-router-dom";
import { CompareReplayPage } from "./pages/CompareReplayPage";
import { ReplayPage } from "./pages/ReplayPage";
import { RunDetailPage } from "./pages/RunDetailPage";
import { RunsPage } from "./pages/RunsPage";
import "./styles.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<RunsPage />} />
        <Route path="/runs/:runId" element={<RunDetailPage />} />
        <Route path="/runs/:runId/compare" element={<CompareReplayPage />} />
        <Route path="/runs/:runId/episodes/:episodeIndex" element={<ReplayPage />} />
      </Routes>
    </BrowserRouter>
  </StrictMode>,
);
