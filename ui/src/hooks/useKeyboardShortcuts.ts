import { useEffect } from "react";

export interface ShortcutHandler {
  /** Key code (e.g. "Space", "ArrowLeft") or single character (case-sensitive). */
  code: string;
  handler: (event: KeyboardEvent) => void;
}

function isEditableTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) {
    return false;
  }
  if (target.isContentEditable) {
    return true;
  }
  const tag = target.tagName;
  return tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA";
}

/**
 * Bind window-level keyboard shortcuts that are suppressed while the user is
 * focused on form controls. Shortcuts are matched by KeyboardEvent.code.
 */
export function useKeyboardShortcuts(shortcuts: ShortcutHandler[], deps: unknown[] = []): void {
  // eslint-disable-next-line react-hooks/exhaustive-deps
  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (isEditableTarget(event.target)) {
        return;
      }
      for (const shortcut of shortcuts) {
        if (shortcut.code === event.code) {
          event.preventDefault();
          shortcut.handler(event);
          return;
        }
      }
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
    // Caller controls dependency tracking.
  }, deps);
}
